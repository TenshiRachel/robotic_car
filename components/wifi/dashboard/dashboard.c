#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "lwip/ip4_addr.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "lwip/apps/lwiperf.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include <lwip/sockets.h>

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define LWIP_SOCKET 1
#define BUF_SIZE 128

int conn_sock;

uint8_t led_blink_state = 1;

TaskHandle_t led_task;

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
        // char ip_str[INET_ADDRSTRLEN] = {0}; // Buffer for client IP
        // inet_ntop(AF_INET, &remote_addr.sin_addr, ip_str, sizeof(ip_str));
        printf("Message: %s\n", buffer);
    }
    close(conn_sock);
}

void blink_led_task(__unused void *params)
{
    while (1)
    {
        if (led_blink_state == 1)
        {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            sleep_ms(50);
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            sleep_ms(450);
        }
    }
}

void connect_wifi()
{
    while (1)
    {
        led_blink_state = 1;
        printf("Connecting wifi\n");
        if (cyw43_arch_wifi_connect_timeout_ms("Matt", "whyyoustealingmydata", CYW43_AUTH_WPA2_AES_PSK, 20000))
        {
            printf("failed to connect.\n");
        }
        else
        {
            printf("Connected.\n");
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
            break;
        }
    }
}

void poll_wifi_state_task(__unused void *params)
{
    /* definitions of wifi status codes for reference */

    // #define CYW43_LINK_DOWN (0)     ///< link is down
    // #define CYW43_LINK_JOIN (1)     ///< Connected to wifi
    // #define CYW43_LINK_NOIP (2)     ///< Connected to wifi, but no IP address
    // #define CYW43_LINK_UP (3)       ///< Connect to wifi with an IP address
    // #define CYW43_LINK_FAIL (-1)    ///< Connection failed
    // #define CYW43_LINK_NONET (-2)   ///< No matching SSID found (could be out of range, or down)
    // #define CYW43_LINK_BADAUTH (-3) ///< Authenticatation failure
    uint8_t macs[16] = {0};
    while (1)
    {
        int link_state = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if (link_state == CYW43_LINK_UP)
        {
            led_blink_state = 0;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        }
        else
        {
            printf("Wifi not connected\n");
            printf("Link state: %d\n", link_state);
            //connect_wifi(); // wifi reconnect after loss seems to be not working
        }
        
        // printf("Running from core %u\n", get_core_num());
        sleep_ms(1000);
    }
}

void main_task(__unused void *params)
{
    if (cyw43_arch_init())
    // if (cyw43_arch_init_with_country(CYW43_COUNTRY_SINGAPORE)) // tell the wifi chip its location - maybe can have better performance?
    {
        printf("failed to initialise\n");
        return;
    }
    cyw43_wifi_pm(&cyw43_state, CYW43_PERFORMANCE_PM);
    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");

    TaskHandle_t wifi_task;
    xTaskCreate(poll_wifi_state_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, 2, &wifi_task);

    TaskHandle_t led_task;
    xTaskCreate(blink_led_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, 2, &led_task);

    connect_wifi();
    // create_telemetry_socket();

    // TaskHandle_t handle_conn_task;
    // xTaskCreate(do_handle_connection, "Connection Thread", configMINIMAL_STACK_SIZE, (void *)sock, 2, &handle_conn_task);

    run_server();

    cyw43_arch_deinit();
}

void vLaunch(void)
{
    TaskHandle_t task;
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, 2, &task);

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
    sleep_ms(3000);
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}