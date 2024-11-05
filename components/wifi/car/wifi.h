#ifndef WIFI_H
#define WIFI_H

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

void run_broadcast();
static void run_server();
void wifi_and_server_task();
void send_data_task();

// Function prototypes to initialize and access the message buffer
void InitMessageBuffer(void);
BaseType_t SendToMessageBuffer(const void *data, size_t size, TickType_t ticksToWait);
size_t ReceiveFromMessageBuffer(void *data, size_t maxSize, TickType_t ticksToWait);

#endif