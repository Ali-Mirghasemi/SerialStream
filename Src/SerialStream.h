#ifndef _SERIAL_STREAM_H_
#define _SERIAL_STREAM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "InputStream.h"
#include "OutputStream.h"
#include <termios.h>
#include <pthread.h>
#include <stdint.h>

struct __SerialStream;
typedef struct __SerialStream SerialStream;

typedef void (*SerialStream_OnDataReceivedFn)(SerialStream* stream);

struct __SerialStream {
    void*                           Args;
    int                             Context;        // File descriptor for serial port
    StreamIn                        Input;          // Input stream
    StreamOut                       Output;         // Output stream
    SerialStream_OnDataReceivedFn   onDataReceived; // Callback for data received
    pthread_t                       PollThread;     // Thread for polling data
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