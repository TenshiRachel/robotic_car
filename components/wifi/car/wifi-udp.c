/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// using UDP for the network connection
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include "lwip/ip4_addr.h"

#include "FreeRTOS.h"
#include "task.h"

#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include <lwip/sockets.h>
#include <semphr.h>

#ifndef PING_ADDR
#define PING_ADDR "192.168.137.1"
#endif
#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define LWIP_SOCKET 1
#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define BUF_SIZE 128

int conn_sock;

const char dashboard_ip[] = "192.168.1.3";
const char controller_ip[] = "192.168.1.2";

static void run_server()
{
    // Step 1: Create a UDP socket
    int conn_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (conn_sock < 0)
    {
        printf("Unable to create socket: error %d\n", errno);
        return;
    }
    printf("Conn sock num: %d\n", conn_sock);
    // Step 2: Bind the socket to the specified port and address
    struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(1234),      // Port to listen on (1234)
        .sin_addr.s_addr = INADDR_ANY // Listen on any available IP address
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
        // printf("Received data from %s:%d\n", ip_str, ntohs(remote_addr.sin_port));

        // Step 7: Process received data (e.g., print it)
        printf("Message: %s\n", buffer);

        // Initialize the start time for measuring execution time
        absolute_time_t start_time = get_absolute_time();

        // Process the command
        // if (!strcmp(buffer, "on"))
        // {
        //     cyw43_arch_gpio_put(0, true); // Turn on the LED
        // }
        // else if (!strcmp(buffer, "off"))
        // {
        //     cyw43_arch_gpio_put(0, false); // Turn off the LED
        // }
        // else
        // {
        // Generate and send a random message
        char randNum = 26 * (rand() / (RAND_MAX + 1.0)) + 97;
        char a[] = "Hello";
        char b[10] = {0};
        char c[4] = {randNum, '\r', '\n', '\0'};

        strcat(b, a);
        strcat(b, c);

        int sent_bytes = sendto(conn_sock, b, strlen(b), 0,
                                (struct sockaddr *)&remote_addr, addr_len);
        if (sent_bytes < 0)
        {
            printf("Error sending response: %d\n", errno);
        }
        // }

        // Measure end time for execution timing
        absolute_time_t end_time = get_absolute_time();
        uint32_t execution_time_before = absolute_time_diff_us(start_time, end_time);

        // Output execution time before optimization
        printf("Execution Time: %d microseconds\n", execution_time_before);
    }

    // Step 8: Close the socket when done (though this is never reached here)
    close(conn_sock);
}

void main_task(__unused void *params)
{
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }
    // cyw43_arch_enable_sta_mode();
    // printf("Connecting to Wi-Fi...\n");
    // if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    // {
    //     printf("failed to connect.\n");
    //     exit(1);
    // }
    // else
    // {
    //     printf("Connected.\n");
    // }
    const char *ap_name = "picow_test";
#if 1
    const char *password = "password";
#else
    const char *password = NULL;
#endif
    cyw43_arch_enable_ap_mode(ap_name, password, CYW43_AUTH_WPA2_AES_PSK);

    run_server();

    cyw43_arch_deinit();
}

void vLaunch(void)
{
    TaskHandle_t task;
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &task);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
    // takes care of it otherwise)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main(void)
{
    stdio_init_all();

    /* Configure the hardware ready to run the demo. */
    const char *rtos_name;
#if (configNUMBER_OF_CORES > 1)
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if (configNUMBER_OF_CORES == 2)
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif (RUN_FREERTOS_ON_CORE == 1)
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true)
        ;
#else
    sleep_ms(4000);
    printf("Starting %s on core 0:\n", rtos_name);
    s_ConnectionSemaphore = xSemaphoreCreateCounting(kConnectionThreadCount, kConnectionThreadCount);
    vLaunch();
#endif
    return 0;
}