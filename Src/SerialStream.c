#include "SerialStream.h"

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <poll.h>
    #include <errno.h>
    #include <string.h>
    #include <sys/ioctl.h>
    #include <stdio.h>
    #include <stdlib.h>
    #include <errno.h>
#endif

#if SERIALSTREAM_LIB_LOG
    #include "Log.h"
#else
    #define  logInfo(...)
    #define  logError(...)
#endif

// Internal functions
static void SerialStream_errorHandle(SerialStream* stream);
static Stream_Result SerialStream_transmit(StreamOut* stream, uint8_t* buff, Stream_LenType len);
static void* SerialStream_pollThread(void* arg);
#if defined(_WIN32) || defined(_WIN64)
    static DWORD WINAPI SerialStream_pollThreadWin(LPVOID arg) { SerialStream_pollThread(arg); return 0; }
#endif
static int32_t SerialStream_convertBaudrate(int32_t br);

#if STREAM_MUTEX
static Stream_MutexResult SerialStream_mutexInit(StreamBuffer* stream, Stream_Mutex* mutex);
static Stream_MutexResult SerialStream_mutexLock(StreamBuffer* stream, Stream_Mutex* mutex);
static Stream_MutexResult SerialStream_mutexUnlock(StreamBuffer* stream, Stream_Mutex* mutex);
static Stream_MutexResult SerialStream_mutexDeInit(StreamBuffer* stream, Stream_Mutex* mutex);
#endif

// Mutex macro mapping (unchanged semantics)
#if STREAM_MUTEX
    #if STREAM_MUTEX == STREAM_MUTEX_CUSTOM
        #define __initMutex(STREAM)                 IStream_setMutex(&(STREAM)->Input, SerialStream_mutexInit, SerialStream_mutexLock, SerialStream_mutexUnlock, SerialStream_mutexDeInit); \
                                                    OStream_setMutex(&(STREAM)->Output, SerialStream_mutexInit, SerialStream_mutexLock, SerialStream_mutexUnlock, SerialStream_mutexDeInit); \
                                                    IStream_mutexInit(&(STREAM)->Input); \
                                                    OStream_mutexInit(&(STREAM)->Output)
    #elif STREAM_MUTEX == STREAM_MUTEX_DRIVER
        static const Stream_MutexDriver SerialStream_MutexDriver = {
            .init = SerialStream_mutexInit,
            .lock = SerialStream_mutexLock,
            .unlock = SerialStream_mutexUnlock,
            .deinit = SerialStream_mutexDeInit
        };

        #define __initMutex(STREAM)                 IStream_setMutex(&(STREAM)->Input, &SerialStream_MutexDriver); \
                                                    OStream_setMutex(&(STREAM)->Output, &SerialStream_MutexDriver); \
                                                    IStream_mutexInit(&(STREAM)->Input); \
                                                    OStream_mutexInit(&(STREAM)->Output)
    #elif STREAM_MUTEX == STREAM_MUTEX_GLOBAL_DRIVER
        static const Stream_MutexDriver SerialStream_MutexDriver = {
            .init = SerialStream_mutexInit,
            .lock = SerialStream_mutexLock,
            .unlock = SerialStream_mutexUnlock,
            .deinit = SerialStream_mutexDeInit
        };

        #define __initMutex(STREAM)                 IStream_setMutex(&SerialStream_MutexDriver); \
                                                    OStream_setMutex(&SerialStream_MutexDriver); \
                                                    IStream_mutexInit(&(STREAM)->Input); \
                                                    OStream_mutexInit(&(STREAM)->Output)
    #endif

    #define __lockMutex(STREAM)                 Stream_mutexLock(&(STREAM)->Buffer)
    #define __unlockMutex(STREAM)               Stream_mutexUnlock(&(STREAM)->Buffer)
    #define __deinitMutex(STREAM)               Stream_mutexDeInit(&(STREAM)->Buffer)
#else
    #define __initMutex(STREAM)                 // No mutex
    #define __lockMutex(STREAM)                 // No mutex
    #define __unlockMutex(STREAM)               // No mutex
    #define __deinitMutex(STREAM)               // No mutex
#endif

// ------------------------------
// Public API
// ------------------------------
uint8_t SerialStream_init(
    SerialStream*   stream,
    const char*     port,
    int32_t         baudrate,
    uint8_t*        rxBuff,
    Stream_LenType  rxBuffSize,
    uint8_t*        txBuff,
    Stream_LenType  txBuffSize
) {
    if (!stream || !port) return 0;

    stream->Context = -1;
#if defined(_WIN32) || defined(_WIN64)
    stream->PollThread = NULL;
    stream->StopEvent = NULL;
#endif

#if defined(_WIN32) || defined(_WIN64)
    // Open COM port: names like "COM1" or "\\\\.\\COM10"
    char winPortName[64];
    if (strlen(port) > 4 && strncmp(port, "\\\\.\\", 4) == 0) {
        strncpy(winPortName, port, sizeof(winPortName)-1);
        winPortName[sizeof(winPortName)-1] = 0;
    } else {
        // try to accept both "COM3" and "\\\\.\\COM3"
        if (strlen(port) > 3 && (port[0] == '\\')) {
            strncpy(winPortName, port, sizeof(winPortName)-1);
        } else {
            snprintf(winPortName, sizeof(winPortName), "%s", port);
        }
    }

    HANDLE h = CreateFileA(winPortName, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
                           FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        // Try prefixing with \\.\ for high-numbered COM ports
        char prefixed[80];
        snprintf(prefixed, sizeof(prefixed), "\\\\.\\%s", port);
        h = CreateFileA(prefixed, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
                        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, NULL);
        if (h == INVALID_HANDLE_VALUE) {
            logError("Error opening serial port %s (CreateFile failed)", port);
            return 0;
        }
    }

    // Configure DCB
    DCB dcb;
    SecureZeroMemory(&dcb, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(h, &dcb)) {
        logError("GetCommState failed");
        CloseHandle(h);
        return 0;
    }

    dcb.BaudRate = (DWORD) SerialStream_convertBaudrate(baudrate);
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fBinary = TRUE;

    if (!SetCommState(h, &dcb)) {
        logError("SetCommState failed");
        CloseHandle(h);
        return 0;
    }

    // Timeouts - make ReadFile return on timeout to let thread poll-like behavior
    COMMTIMEOUTS timeouts;
    timeouts.ReadIntervalTimeout = 50; // 50 ms
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 50; // 50 ms
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 50;
    SetCommTimeouts(h, &timeouts);

    // Purge
    PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT);

    stream->Context = (intptr_t)h;

#else
    // POSIX open
    stream->Context = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(stream->Context < 0) {
        perror("Error opening serial port");
        return 0;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);
    int32_t br = SerialStream_convertBaudrate(baudrate);
    if(tcgetattr(stream->Context, &tty) != 0) {
        perror("Error getting termios");
        close(stream->Context);
        stream->Context = -1;
        return 0;
    }

    cfsetospeed(&tty, br);
    cfsetispeed(&tty, br);

    tty.c_cflag &= ~PARENB;     // No parity
    tty.c_cflag &= ~CSTOPB;     // 1 stop bit
    tty.c_cflag &= ~CSIZE;      // Clear data size
    tty.c_cflag |= CS8;         // 8 data bits
    tty.c_cflag &= ~CRTSCTS;    // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL; // Enable reading

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    tty.c_oflag &= ~OPOST;      // Raw output

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Disable software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_cc[VMIN] = 0;         // Non-blocking read
    tty.c_cc[VTIME] = 0;

    if(tcsetattr(stream->Context, TCSANOW, &tty) != 0) {
        perror("Error setting termios");
        close(stream->Context);
        stream->Context = -1;
        return 0;
    }
#endif

    // Initialize streams
    IStream_init(&stream->Input, NULL, rxBuff, rxBuffSize);
    IStream_setDriverArgs(&stream->Input, stream);

    OStream_init(&stream->Output, SerialStream_transmit, txBuff, txBuffSize);
    OStream_setDriverArgs(&stream->Output, stream);

    __initMutex(stream);

    // Start the polling thread
#if defined(_WIN32) || defined(_WIN64)
    // Create an event to signal stop
    stream->StopEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (!stream->StopEvent) {
        logError("CreateEvent failed");
        SerialStream_close(stream);
        return 0;
    }
    // Create thread (overlapped I/O is used in ReadFile within the thread)
    stream->PollThread = CreateThread(NULL, 0, SerialStream_pollThreadWin, stream, 0, NULL);
    if (!stream->PollThread) {
        logError("CreateThread failed");
        SerialStream_close(stream);
        return 0;
    }
#else
    if(pthread_create(&stream->PollThread, NULL, SerialStream_pollThread, stream) != 0) {
        perror("Error creating poll thread");
#if defined(_WIN32) || defined(_WIN64)
        // handled above
#endif
        close((int)stream->Context);
        stream->Context = -1;
        return 0;
    }
    pthread_detach(stream->PollThread);
#endif

    logInfo("Serial stream initialized on %s with baudrate %d", port, baudrate);
    return 1;
}

uint8_t SerialStream_close(SerialStream* stream) {
    if (!stream) return 0;

#if defined(_WIN32) || defined(_WIN64)
    if (stream->Context != (intptr_t)-1 && stream->Context != 0) {
        HANDLE h = (HANDLE)stream->Context;

        // Signal thread to stop
        if (stream->StopEvent) SetEvent(stream->StopEvent);

        // Wait for thread to finish
        if (stream->PollThread) {
            WaitForSingleObject(stream->PollThread, 200);
            CloseHandle(stream->PollThread);
            stream->PollThread = NULL;
        }

        CloseHandle(h);
        stream->Context = -1;
    }

    if (stream->StopEvent) {
        CloseHandle(stream->StopEvent);
        stream->StopEvent = NULL;
    }
#else
    if(stream->Context >= 0) {
        close((int)stream->Context);
        stream->Context = -1;
    }
#endif

    IStream_deinit(&stream->Input);
    OStream_deinit(&stream->Output);
    logInfo("Serial stream closed");
    return 1;
}

// Error handler
static void SerialStream_errorHandle(SerialStream* stream) {
    IStream_resetIO(&stream->Input);
    OStream_resetIO(&stream->Output);
    IStream_receive(&stream->Input); // Restart reception
}

static Stream_Result SerialStream_transmit(StreamOut* stream, uint8_t* buff, Stream_LenType len) {
    SerialStream* serial = (SerialStream*) OStream_getDriverArgs(stream);
    if(!serial) return Stream_NoTransmit;

#if defined(_WIN32) || defined(_WIN64)
    if (serial->Context == (intptr_t)-1 || serial->Context == 0) return Stream_NoTransmit;
    HANDLE h = (HANDLE)serial->Context;

    OVERLAPPED ov;
    memset(&ov, 0, sizeof(ov));
    ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
    if (!ov.hEvent) return Stream_CustomError | GetLastError();

    DWORD written = 0;
    BOOL res = WriteFile(h, buff, (DWORD)len, &written, &ov);
    if (!res) {
        DWORD err = GetLastError();
        if (err == ERROR_IO_PENDING) {
            // Wait until done or timeout (small wait)
            DWORD wait = WaitForSingleObject(ov.hEvent, 200);
            if (wait == WAIT_OBJECT_0) {
                if (!GetOverlappedResult(h, &ov, &written, FALSE)) {
                    DWORD ge = GetLastError();
                    CloseHandle(ov.hEvent);
                    return Stream_CustomError | ge;
                }
            } else {
                // timed out or failed
                CancelIo(h);
                CloseHandle(ov.hEvent);
                return Stream_CustomError | ERROR_TIMEOUT;
            }
        } else {
            CloseHandle(ov.hEvent);
            return Stream_CustomError | err;
        }
    }
    CloseHandle(ov.hEvent);

    if ((int)written > 0) {
        stream->Buffer.InTransmit = 1;
        return OStream_handle(stream, (int)written);
    } else {
        return Stream_Ok;
    }

#else
    if(serial->Context < 0) {
        return Stream_NoTransmit;
    }

    int write_bytes = write((int)serial->Context, buff, len);
    if(write_bytes < 0) {
        if(!(errno == EAGAIN || errno == EWOULDBLOCK)) {
            return Stream_CustomError | errno; // Error writing
        }
    } 
    else if(write_bytes > 0) {
        stream->Buffer.InTransmit = 1;
        return OStream_handle(stream, write_bytes);
    }
    else {
        return Stream_Ok; // Successfully transmitted (zero?)
    }
#endif
}

// Thread / poll implementation (common function for both platforms)
static void* SerialStream_pollThread(void* arg) {
    SerialStream* stream = (SerialStream*)arg;
    if (!stream) return NULL;

#if defined(_WIN32) || defined(_WIN64)
    HANDLE h = (HANDLE)stream->Context;
    if (h == NULL || h == INVALID_HANDLE_VALUE) return NULL;

    // We'll use a simple ReadFile loop with timeouts and check StopEvent periodically
    const DWORD readChunk = 512;
    uint8_t localBuf[2048];

    while (1) {
        // check stop event
        if (stream->StopEvent && WaitForSingleObject(stream->StopEvent, 0) == WAIT_OBJECT_0) {
            break;
        }

        OVERLAPPED ov;
        memset(&ov, 0, sizeof(ov));
        ov.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
        if (!ov.hEvent) {
            // cannot create event - sleep a bit
            Sleep(10);
            continue;
        }

        DWORD read = 0;
        BOOL res = ReadFile(h, localBuf, (DWORD)sizeof(localBuf), &read, &ov);
        if (!res) {
            DWORD err = GetLastError();
            if (err == ERROR_IO_PENDING) {
                // wait for data or stop event
                HANDLE waits[2] = { ov.hEvent, stream->StopEvent ? stream->StopEvent : NULL };
                DWORD waitRes;
                if (stream->StopEvent) {
                    waitRes = WaitForMultipleObjects(2, waits, FALSE, 200);
                } else {
                    waitRes = WaitForSingleObject(ov.hEvent, 200);
                }

                if (stream->StopEvent && waitRes == WAIT_OBJECT_0 + 1) {
                    // stop requested
                    CancelIo(h);
                    CloseHandle(ov.hEvent);
                    break;
                } else if (waitRes == WAIT_OBJECT_0 || waitRes == WAIT_OBJECT_0 + 0) {
                    // data ready
                    if (!GetOverlappedResult(h, &ov, &read, FALSE)) {
                        // error
                        CloseHandle(ov.hEvent);
                        SerialStream_errorHandle(stream);
                        break;
                    }
                } else {
                    // timeout - cancel and continue
                    CancelIo(h);
                    CloseHandle(ov.hEvent);
                    continue;
                }
            } else {
                // fatal error
                CloseHandle(ov.hEvent);
                SerialStream_errorHandle(stream);
                break;
            }
        }

        if (read > 0) {
            __lockMutex(&stream->Input);
            uint8_t* buf = IStream_getDataPtr(&stream->Input);
            Stream_LenType len = IStream_directSpace(&stream->Input);

            // copy only what fits
            Stream_LenType toCopy = (read < len) ? (Stream_LenType)read : len;
            if (toCopy > 0) {
                memcpy(buf, localBuf, (size_t)toCopy);
                stream->Input.Buffer.InReceive = 1;
                stream->Input.Buffer.PendingBytes = (int)toCopy;
                IStream_handle(&stream->Input, (int)toCopy);
            }
            __unlockMutex(&stream->Input);
        }

        if (ov.hEvent) CloseHandle(ov.hEvent);
        // small sleep to avoid busy-looping
        Sleep(1);
    }

    return NULL;

#else
    while(stream->Context >= 0) {
        struct pollfd fds = {
            .fd = stream->Context,
            .events = POLLIN
        };

        int ret = poll(&fds, 1, -1);
        if(ret > 0) {
            if(fds.revents & POLLIN) {
                __lockMutex(&stream->Input);
                uint8_t* buf = IStream_getDataPtr(&stream->Input);
                Stream_LenType len = IStream_directSpace(&stream->Input);

                int read_bytes = read(stream->Context, buf, len);
                __unlockMutex(&stream->Input);

                if(read_bytes < 0) {
                    if(!(errno == EAGAIN || errno == EWOULDBLOCK)) {
                        SerialStream_errorHandle(stream);
                        break;
                    }
                }
                else if(read_bytes > 0) {
                    stream->Input.Buffer.InReceive = 1;
                    stream->Input.Buffer.PendingBytes = read_bytes;
                    IStream_handle(&stream->Input, read_bytes);
                }
                else {
                    // Stream closed
                    logInfo("Serial stream closed by remote");
                    SerialStream_errorHandle(stream);
                    return NULL;
                }
            }

            if(fds.revents & (POLLERR | POLLHUP | POLLNVAL)) {
                SerialStream_errorHandle(stream);
                break;
            }
        }
        else if(ret < 0) {
            if(errno != EINTR) {
                logError("Poll error: %s", strerror(errno));
                break;
            }
        }
    }
    return NULL;
#endif
}

#if STREAM_MUTEX
#include <errno.h>

static Stream_MutexResult SerialStream_mutexInit(StreamBuffer* stream, Stream_Mutex* mutex) {
    if (!stream) {
        return (Stream_MutexResult) EINVAL;
    }

#if defined(_WIN32) || defined(_WIN64)
    CRITICAL_SECTION* cs = (CRITICAL_SECTION*) malloc(sizeof(CRITICAL_SECTION));
    if (!cs) {
        return (Stream_MutexResult) ENOMEM;
    }
    InitializeCriticalSection(cs);
    stream->Mutex = (void*)cs;
#else
    pthread_mutex_t* new_mutex = malloc(sizeof(pthread_mutex_t));
    if (!new_mutex) {
        return (Stream_MutexResult) ENOMEM;
    }

    pthread_mutexattr_t attr;
    int ret = pthread_mutexattr_init(&attr);
    if (ret != 0) {
        free(new_mutex);
        return (Stream_MutexResult) ret;
    }

    ret = pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    if (ret != 0) {
        pthread_mutexattr_destroy(&attr);
        free(new_mutex);
        return (Stream_MutexResult) ret;
    }

    ret = pthread_mutex_init(new_mutex, &attr);
    pthread_mutexattr_destroy(&attr);
    if (ret != 0) {
        free(new_mutex);
        return (Stream_MutexResult) ret;
    }

    stream->Mutex = (void*)new_mutex;
#endif

    return Stream_Ok;
}

static Stream_MutexResult SerialStream_mutexLock(StreamBuffer* stream, Stream_Mutex* mutex) {
    if (!stream || !stream->Mutex) {
        return (Stream_MutexResult) EINVAL;
    }

#if defined(_WIN32) || defined(_WIN64)
    EnterCriticalSection((CRITICAL_SECTION*)stream->Mutex);
    return Stream_Ok;
#else
    return (Stream_MutexResult) pthread_mutex_lock((pthread_mutex_t*)stream->Mutex);
#endif
}

static Stream_MutexResult SerialStream_mutexUnlock(StreamBuffer* stream, Stream_Mutex* mutex) {
    if (!stream || !stream->Mutex) {
        return (Stream_MutexResult) EINVAL;
    }

#if defined(_WIN32) || defined(_WIN64)
    LeaveCriticalSection((CRITICAL_SECTION*)stream->Mutex);
    return Stream_Ok;
#else
    return (Stream_MutexResult) pthread_mutex_unlock((pthread_mutex_t*)stream->Mutex);
#endif
}

static Stream_MutexResult SerialStream_mutexDeInit(StreamBuffer* stream, Stream_Mutex* mutex) {
    if (!stream) {
        return (Stream_MutexResult) EINVAL;
    }

#if defined(_WIN32) || defined(_WIN64)
    if (stream->Mutex) {
        CRITICAL_SECTION* cs = (CRITICAL_SECTION*)stream->Mutex;
        DeleteCriticalSection(cs);
        free(cs);
        stream->Mutex = NULL;
    }
    return Stream_Ok;
#else
    pthread_mutex_t* mutex_ptr = (pthread_mutex_t*)stream->Mutex;
    int ret = pthread_mutex_destroy(mutex_ptr);
    free(mutex_ptr);
    stream->Mutex = NULL;
    if (ret != 0) {
        return (Stream_MutexResult) ret;
    }
    return Stream_Ok;
#endif
}
#endif // STREAM_MUTEX

static int32_t SerialStream_convertBaudrate(int32_t br) {
#if defined(_WIN32) || defined(_WIN64)
    // Windows uses raw integer baudrate
    return br;
#else
    switch (br) {
        case B57600:
        case 57600:
            return B57600;
        case B115200:
        case 115200:
            return B115200;
        case B230400:
        case 230400:
            return B230400;
        case B460800:
        case 460800:
            return B460800;
        case B500000:
        case 500000:
            return B500000;
        case B576000:
        case 576000:
            return B576000;
        case B921600:
        case 921600:
            return B921600;
        case B1000000:
        case 1000000:
            return B1000000;
        case B1152000:
        case 1152000:
            return B1152000;
        case B1500000:
        case 1500000:
            return B1500000;
        case B2000000:
        case 2000000:
            return B2000000;
        case B2500000:
        case 2500000:
            return B2500000;
        case B3000000:
        case 3000000:
            return B3000000;
        case B3500000:
        case 3500000:
            return B3500000;
        case B4000000:
        case 4000000:
            return B4000000;
        default:
            return B115200;
    }
#endif
}
