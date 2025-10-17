#include "SerialStream.h"
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <stdio.h>

#if SERIALSTREAM_LIB_LOG
    #include "Log.h"
#else
    #define  logInfo(...)
    #define  logError(...)
#endif


// Internal functions
// Handle errors (call from event loop)
static void SerialStream_errorHandle(SerialStream* stream);
// Poll for serial events (non-blocking)
static Stream_Result SerialStream_transmit(StreamOut* stream, uint8_t* buff, Stream_LenType len);

static void* SerialStream_pollThread(void* arg);

static int32_t SerialStream_convertBaudrate(int32_t br);

// Mutex for thread safety
#if STREAM_MUTEX
static Stream_MutexResult SerialStream_mutexInit(StreamBuffer* stream, Stream_Mutex* mutex);
static Stream_MutexResult SerialStream_mutexLock(StreamBuffer* stream, Stream_Mutex* mutex);
static Stream_MutexResult SerialStream_mutexUnlock(StreamBuffer* stream, Stream_Mutex* mutex);
static Stream_MutexResult SerialStream_mutexDeInit(StreamBuffer* stream, Stream_Mutex* mutex);

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

uint8_t SerialStream_init(
    SerialStream*   stream,
    const char*     port,
    int32_t         baudrate,
    uint8_t*        rxBuff,
    Stream_LenType  rxBuffSize,
    uint8_t*        txBuff,
    Stream_LenType  txBuffSize
) {
    // Open serial port
    stream->Context = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(stream->Context < 0) {
        perror("Error opening serial port");
        return 0;
    }

    // Configure serial interface
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

    // Initialize streams
    IStream_init(&stream->Input, NULL, rxBuff, rxBuffSize);
    IStream_setDriverArgs(&stream->Input, stream);

    OStream_init(&stream->Output, SerialStream_transmit, txBuff, txBuffSize);
    OStream_setDriverArgs(&stream->Output, stream);

    __initMutex(stream);
    // Start the polling thread
    if(pthread_create(&stream->PollThread, NULL, SerialStream_pollThread, stream) != 0) {
        perror("Error creating poll thread");
        close(stream->Context);
        stream->Context = -1;
        return 0;
    }
    // Detach the thread so it cleans up automatically
    pthread_detach(stream->PollThread);
    // Start receiving data
    logInfo("Serial stream initialized on %s with baudrate %d", port, baudrate);

    return 1;
}

uint8_t SerialStream_close(SerialStream* stream) {
    if(stream->Context >= 0) {
        close(stream->Context);
        stream->Context = -1;
    }

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
    if(serial->Context < 0) {
        return Stream_NoTransmit; // Serial port not open
    }

    int write_bytes = write(serial->Context, buff, len);
    if(write_bytes < 0) {
        if(!(errno == EAGAIN || errno == EWOULDBLOCK)) {
            return Stream_CustomError | errno; // Error reading
        }
    } 
    else if(write_bytes > 0) {
        stream->Buffer.InTransmit = 1;
        return OStream_handle(stream, write_bytes);
    }
    else {
        return Stream_Ok; // Successfully transmitted
    }
}

static void* SerialStream_pollThread(void* arg) {
    SerialStream* stream = (SerialStream*)arg;
    
    while(stream->Context >= 0) {
        // Only poll for input when needed (POLLIN only)
        struct pollfd fds = {
            .fd = stream->Context,
            .events = POLLIN  // Only wait for input
        };

        // Block indefinitely until data arrives or error occurs
        int ret = poll(&fds, 1, -1);  // -1 = no timeout
        
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
            if(errno != EINTR) {  // Ignore interrupted system calls
                logError("Poll error: %s", strerror(errno));
                break;
            }
        }
    }
    return NULL;
}

#if STREAM_MUTEX
#include <stdlib.h>
#include <errno.h>

// Platform detection
#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
    #define STREAM_MUTEX_TYPE CRITICAL_SECTION
#else
    #include <pthread.h>
    #define STREAM_MUTEX_TYPE pthread_mutex_t
#endif

static Stream_MutexResult SerialStream_mutexInit(StreamBuffer* stream, Stream_Mutex* mutex) {
    if (!stream) {
        return (Stream_MutexResult) EINVAL;
    }

#if defined(_WIN32) || defined(_WIN64)
    // Windows implementation
    CRITICAL_SECTION* cs = malloc(sizeof(CRITICAL_SECTION));
    if (!cs) {
        return (Stream_MutexResult) ENOMEM;
    }
    InitializeCriticalSection(cs);
    stream->Mutex = (void*)cs;
#else
    // Linux/POSIX implementation
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
    if (!stream || !stream->Mutex) {
        return (Stream_MutexResult) EINVAL;
    }

#if defined(_WIN32) || defined(_WIN64)
    CRITICAL_SECTION* cs = (CRITICAL_SECTION*)stream->Mutex;
    DeleteCriticalSection(cs);
    free(cs);
#else
    pthread_mutex_t* mutex_ptr = (pthread_mutex_t*)stream->Mutex;
    int ret = pthread_mutex_destroy(mutex_ptr);
    free(mutex_ptr);
    if (ret != 0) {
        return (Stream_MutexResult) ret;
    }
#endif

    stream->Mutex = NULL;
    return Stream_Ok;
}
#endif

static int32_t SerialStream_convertBaudrate(int32_t br) {
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
}
