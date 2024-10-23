/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// using TCP for the network connection
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

int conn_sock;
const int kConnectionThreadCount = 3;
static xSemaphoreHandle s_ConnectionSemaphore;

const char dashboard_ip[] = "192.168.1.3";
const char controller_ip[] = "192.168.1.2";

typedef struct conn_ip_mapping
{
    char client_ip[16];
    int conn_sock_num;
};

struct conn_ip_mapping conn_ip_map[2];

static void send_message(int socket, char *msg)
{
    // int len = strlen(msg);
    // int done = 0;
    // while (done < len)
    // {
    //     int done_now = send(socket, msg + done, len - done, 0);
    //     if (done_now <= 0)
    //         return;
    //     done += done_now;
    // }
    int len = strlen(msg);

    while (len > 0)
    {
        int done_now = send(socket, msg, len, 0); // Try to send 'len' bytes from 'msg'
        if (done_now <= 0)
            return; // Handle error or disconnection

        msg += done_now; // Move the pointer forward by 'done_now' bytes
        len -= done_now; // Decrement 'len' by 'done_now'
    }
}

// static int handle_single_command(int conn_sock)
// {
//     char buffer[128];
//     int done = 0;

//     send_message(conn_sock, "Enter command: ");

//     while (done < sizeof(buffer))
//     {
//         int done_now = recv(conn_sock, buffer + done, sizeof(buffer) - done, 0);
//         absolute_time_t start_time = get_absolute_time();

//         if (done_now <= 0)
//             return -1;
//         done += done_now;
//         char *end = strnstr(buffer, "\r", done);
//         if (!end)
//             continue;
//         *end = 0;
//         printf("%s", buffer);

//         if (!strcmp(buffer, "on"))
//         {
//             cyw43_arch_gpio_put(0, true);
//             send_message(conn_sock, "The LED is now on\r\n");
//         }
//         else if (!strcmp(buffer, "off"))
//         {
//             cyw43_arch_gpio_put(0, false);
//             send_message(conn_sock, "The LED is now off\r\n");
//         }
//         else
//         {
//             char randNum = 26 * (rand() / (RAND_MAX + 1.0));

//             randNum = randNum + 97;
//             char a[] = "Hello";
//             char b[10] = {0};

//             char c[4];

//             c[0] = randNum;
//             c[1] = '\r';
//             c[2] = '\n';
//             c[3] = '\0';

//             strcat(b, a);
//             strcat(b, c);

//             send_message(conn_sock, b);
//         }
//         absolute_time_t end_time = get_absolute_time();

//         uint32_t execution_time_before = absolute_time_diff_us(start_time, end_time);

//         // Output execution time before optimization
//         printf("Execution Time: %d microseconds\n", execution_time_before);

//         break;
//     }

//     return 0;
// }
static int handle_single_command(int conn_sock)
{
    char buffer[64];          // Command buffer
    int len = sizeof(buffer); // Remaining length to read

    send_message(conn_sock, "Enter cmd: ");

    while (len > 0)
    {
        // Receive data from the socket into buffer
        int done_now = recv(conn_sock, buffer, len, 0);

        // Initialize the start time for measuring execution time
        absolute_time_t start_time = get_absolute_time();

        if (done_now <= 0)
            return -1; // Error occurred

        len -= done_now; // Decrement the remaining length

        // Check if the carriage return is found in the newly received data
        char *end = strnstr(buffer, "\r", sizeof(buffer) - len);
        if (!end)
            continue; // If no '\r' found, continue receiving

        *end = 0; // Null-terminate the string at the location of '\r'
        
        printf("%s", buffer); // Print the command

        

        // Process the command
        // if (!strcmp(buffer, "on"))
        // {
        //     cyw43_arch_gpio_put(0, true); // Turn on the LED
        //     send_message(conn_sock, "The LED is now on\r\n");
        // }
        // else if (!strcmp(buffer, "off"))
        // {
        //     cyw43_arch_gpio_put(0, false); // Turn off the LED
        //     send_message(conn_sock, "The LED is now off\r\n");
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

        send_message(conn_sock, b);
        // }

        // Measure end time for execution timing
        absolute_time_t end_time = get_absolute_time();
        uint32_t execution_time_before = absolute_time_diff_us(start_time, end_time);

        // Output execution time before optimization
        printf("Execution Time: %d microseconds\n", execution_time_before);

        break; // Break the loop since we handled the command
    }

    return 0;
}

static void do_handle_connection(void *arg)
{
    int conn_sock = (int)arg;
    while (!handle_single_command(conn_sock))
    {
    }

    closesocket(conn_sock);
    printf("The socket connection with id %d was closed.\n", conn_sock);

    for (int i = 0; i < 2; i++)
    {
        if (conn_ip_map[i].conn_sock_num == conn_sock)
        {
            printf("IP address: %s, conn num: %d\n", conn_ip_map[i].client_ip, conn_ip_map[i].conn_sock_num);
            conn_ip_map[i].client_ip[0] = 0;
            conn_ip_map[i].conn_sock_num = -1;
            printf("Deleted connection at index %d\n", i);
            break;
        }
    }

    xSemaphoreGive(s_ConnectionSemaphore);
    vTaskDelete(NULL);
}

static void handle_connection(int conn_sock)
{
    TaskHandle_t task;
    xSemaphoreTake(s_ConnectionSemaphore, portMAX_DELAY);
    xTaskCreate(do_handle_connection, "Connection Thread", configMINIMAL_STACK_SIZE, (void *)conn_sock, TEST_TASK_PRIORITY, &task);
}

static void run_server()
{
    int server_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    struct sockaddr_in listen_addr =
        {
            .sin_len = sizeof(struct sockaddr_in),
            .sin_family = AF_INET,
            .sin_port = htons(1234),
            .sin_addr = 0,
        };

    if (server_sock < 0)
    {
        printf("Unable to create socket: error %d", errno);
        return;
    }

    if (bind(server_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0)
    {
        printf("Unable to bind socket: error %d\n", errno);
        return;
    }

    if (listen(server_sock, kConnectionThreadCount * 2) < 0)
    {
        printf("Unable to listen on socket: error %d\n", errno);
        return;
    }

    printf("Starting server at %s on port %u\n", ip4addr_ntoa(netif_ip4_addr(netif_list)), ntohs(listen_addr.sin_port));

    while (true)
    {
        struct sockaddr_storage remote_addr;
        socklen_t len = sizeof(remote_addr);
        conn_sock = accept(server_sock, (struct sockaddr *)&remote_addr, &len);

        char ip_str[16];

        struct sockaddr_in *s = (struct sockaddr_in *)&remote_addr;
        inet_ntop(AF_INET, &s->sin_addr, ip_str, sizeof(ip_str));

        if (conn_sock < 0)
        {
            printf("Unable to accept incoming connection: error %d\n", errno);
            return;
        }

        printf("Incoming connection from %s\n", ip_str);
        printf("Connection socket is %d\n", conn_sock);

        for (int i = 0; i < 2; i++)
        {
            if (conn_ip_map[i].client_ip[0] == '\0')
            {
                strncpy(conn_ip_map[i].client_ip, ip_str, 16);
                conn_ip_map[i].client_ip[15] = 0;
                conn_ip_map[i].conn_sock_num = conn_sock;
                printf("Inserted new connection at index %d\n", i);
                printf("IP address:  %s, conn num: %d\n", conn_ip_map[i].client_ip, conn_ip_map[i].conn_sock_num);

                break;
            }
        }

        handle_connection(conn_sock);
    }
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

    // ip_addr_t ping_addr;
    // ipaddr_aton(PING_ADDR, &ping_addr);
    // ping_init(&ping_addr);

    run_server();

    // while (true)
    // {
    //     // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
    //     vTaskDelay(100);
    // }

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