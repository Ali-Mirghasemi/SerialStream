#ifndef _SERIAL_STREAM_H_
#define _SERIAL_STREAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "InputStream.h"
#include "OutputStream.h"

#include <stdint.h>

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
#else
    #include <termios.h>
    #include <pthread.h>
#endif

struct __SerialStream;
typedef struct __SerialStream SerialStream;

struct __SerialStream {
    void*                           Args;
    StreamIn                        Input;          // Input stream
    StreamOut                       Output;         // Output stream
    intptr_t                        Context;        // File descriptor (POSIX) or HANDLE cast to intptr_t (Win32)

#if defined(_WIN32) || defined(_WIN64)
    HANDLE                          PollThread;     // Thread handle (Win32)
    HANDLE                          StopEvent;      // Event to signal thread stop
#else
    pthread_t                       PollThread;     // Thread for polling data (POSIX)
#endif
};

// Initialize serial stream
uint8_t SerialStream_init(
    SerialStream*   stream,
    const char*     port,
    int32_t         baudrate,
    uint8_t*        rxBuff,
    Stream_LenType  rxBuffSize,
    uint8_t*        txBuff,
    Stream_LenType  txBuffSize
);

uint8_t SerialStream_close(SerialStream* stream);

#ifdef __cplusplus
};
#endif

#endif
