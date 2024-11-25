/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// using UDP for the network connection
// chatgpt was used to modify this from the original TCP code

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "lwip/ip4_addr.h"

// #include "FreeRTOS.h"
// #include "message_buffer.h"
// #include "task.h"

#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include <lwip/sockets.h>
#include "lwipopts.h"
#include "wifi.h"
#include "components/motor_control/motor_control.h"
#include "components/ultrasonic_encoder/ultrasonic_encoder.h"

#ifndef PING_ADDR
#define PING_ADDR "192.168.137.1"
#endif
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define LWIP_SOCKET 1
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define mbaTASK_MESSAGE_BUFFER_SIZE (192) // message buffer size, increase as needed based on individual message size

#define BUF_SIZE 4

#define MESSAGE "Message from Pico!"
#define TARGET_IP "192.168.4.3" // Replace with the specific IP address of your computer (can be found using)
#define TARGET_PORT 8000          // Replace with the desired target port

static struct sockaddr_in target_addr_dashboard;
static struct sockaddr_in target_addr;

static int sock;
int conn_sock;
int num_stas;
uint8_t macs;
cyw43_t self;
MessageBufferHandle_t xSendMessageBuffer = NULL;

// Function to initialize the message buffer
void InitMessageBuffer(void)
{
    if (xSendMessageBuffer == NULL)
    { // Ensure it is only created once
        xSendMessageBuffer = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);
        configASSERT(xSendMessageBuffer); // Check creation success
    }
}

// Access functions for sending and receiving data
BaseType_t SendToMessageBuffer(const void *data, size_t size, TickType_t ticksToWait)
{
    return xMessageBufferSend(xSendMessageBuffer, data, size, ticksToWait);
}

size_t ReceiveFromMessageBuffer(void *data, size_t maxSize, TickType_t ticksToWait)
{
    return xMessageBufferReceive(xSendMessageBuffer, data, maxSize, ticksToWait);
}

void send_data_task(__unused void *params)
{
    static char sReceivedData[64] = {0};
    size_t xReceivedBytes;

    while (1)
    {
        ReceiveFromMessageBuffer(sReceivedData, sizeof(sReceivedData), portMAX_DELAY);

        // Send the message to the target address
        int result = sendto(sock, sReceivedData, strlen(sReceivedData), 0, (struct sockaddr *)&target_addr_dashboard, sizeof(target_addr_dashboard));
        if (result < 0)
        {
            printf("Failed to send message: error %d\n", errno);
        } else {
            // printf("Sent message\n");
        }
    }

    // Close the socket
    close(sock);
}

int create_socket()
{
    // Create a UDP socket for connecting to the dashboard pico
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        printf("Failed to create socket: error %d\n", errno);
        return 0;
    } else {
        printf("Created outgoing socket number: %d\n", sock);
    }

    // Set up the destination address
    memset(&target_addr_dashboard, 0, sizeof(target_addr_dashboard));
    target_addr_dashboard.sin_family = AF_INET;
    target_addr_dashboard.sin_port = htons(1234);
    target_addr_dashboard.sin_addr.s_addr = inet_addr(TARGET_IP);

    return 1;
}

static void run_server()
{
    // Step 1: Create a UDP socket
    conn_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (conn_sock < 0)
    {
        printf("Unable to create socket: error %d\n", errno);
        return;
    }
    printf("Conn sock num: %d\n", conn_sock);
    // Step 2: Bind the socket to the specified port and address
    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(TARGET_PORT),         // Port to listen on
        .sin_addr.s_addr = INADDR_ANY // Listen on any available IP address, on the pico the default is 192.168.4.1
    };

    if (bind(conn_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
    {
        printf("Unable to bind socket: error %d\n", errno);
        close(conn_sock);
        return;
    }

    printf("UDP server running on port %u\n", ntohs(listen_addr.sin_port));

    // Step 3: Set up buffer and remote address structure
    char buffer[BUF_SIZE];
    struct sockaddr_in remote_addr;
    socklen_t addr_len = sizeof(remote_addr);

    // Step 4: Loop to receive and process data
    while (true)
    {
        memset(buffer, 0, sizeof(buffer)); // Clear buffer

        // Step 5: Receive data from client
        int received_bytes = recvfrom(conn_sock, buffer, BUF_SIZE, 0,
                                      (struct sockaddr *)&remote_addr, &addr_len);
        if (received_bytes < 0)
        {
            printf("Error receiving data: %d\n", errno);
            continue;
        }

        // Step 6: Extract and print client IP address
        char ip_str[INET_ADDRSTRLEN]; // Buffer for client IP
        inet_ntop(AF_INET, &remote_addr.sin_addr, ip_str, sizeof(ip_str));
        process_command_with_speed(buffer[0]-'0'); 
    }

    // Step 8: Close the socket when done (though this is never reached here)
    close(conn_sock);
}

// wifi_and_server_task updated to use AP mode
void wifi_and_server_task(__unused void *params)
{
    printf("init wifi");
    if (cyw43_arch_init())
    // if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) // tell the wifi chip its location - maybe can have better performance?
    {
        printf("failed to initialise\n");
        return;
    }
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM); // set wifi power management to performance mode

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms("Matt", "whyyoustealingmydata", CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect.\n");
        exit(1);
    }
    else
    {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

        printf("Connected.\n");
    }

    const char *ap_name = "picow_p5a";
#if 1
    const char *password = "password";
#else
    const char *password = NULL;
#endif

    TaskHandle_t send_data;
    xTaskCreate(send_data_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, 2, &send_data);

    TaskHandle_t telemetry_task;
    xTaskCreate(telemetryTask, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, 2, &telemetry_task);

    create_socket();
    run_server();

    cyw43_arch_deinit(); // should never reach here unless error!!
}