// more info can be found in lwipopts_examples_common.h and the cmake
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
#include <lwip/sockets.h>

#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

#ifndef RUN_FREERTOS_ON_CORE
#define RUN_FREERTOS_ON_CORE 0
#endif

#define TEST_TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)
#define mbaTASK_MESSAGE_BUFFER_SIZE (80) // message buffer size, increase as needed based on individual message size

#define MESSAGE "Message from Pico!"


#define TARGET_IP "192.168.137.1" // Replace with the specific IP address of your computer (can be found using)
#define TARGET_PORT 8000          // Replace with the desired target port
#define BUF_SIZE 96

static struct sockaddr_in target_addr;
static int sock;
static MessageBufferHandle_t xSendMessageBuffer;

void send_data_task(__unused void *params)
{
    static char sReceivedData[40] = {0};
    size_t xReceivedBytes;

    while (1)
    {
        xReceivedBytes = xMessageBufferReceive(
            xSendMessageBuffer,    /* The message buffer to receive from. */
            (void *)&sReceivedData, /* Location to store received data. */
            sizeof(sReceivedData),  /* Maximum number of bytes to receive. */
            portMAX_DELAY);         /* Wait forever until something is received */

        // Send the message to the target address
        int result = sendto(sock, sReceivedData, strlen(sReceivedData), 0, (struct sockaddr *)&target_addr, sizeof(target_addr));
        if (result < 0)
        {
            printf("Failed to send message: error %d\n", errno);
        }
        else
        {
            printf("Message sent successfully\n");
        }
    }

    //Close the socket
    close(sock);
}

void blink_led_task(__unused void *params)
{
    while(1) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(50);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(450);
    }
}

void create_data()
{
    // simulate main loop where you get data and pass it to the wifi task to be sent out
    while(true) {
        // poll wifi state, if link goes down exit the program (todo error handling)
        if (cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP)
        {
            // create some random rubbish to send
            char randNum = 26 * (rand() / (RAND_MAX + 1.0)) + 97;
            static char a[] = "Hello ";
            char b[11] = {0};
            char c[4] = {randNum, '\r', '\n', '\0'};

            strcat(b, a);
            strcat(b, c);

            printf("Message sent: %s", b);
            xMessageBufferSend(/* The message buffer to write to. */
                            xSendMessageBuffer,
                            /* The source of the data to send. */
                            (void *)&b,
                            /* The length of the data to send. */
                            sizeof(b),
                            /* The block time; 0 = no block */
                            0);
            sleep_ms(1500);
        } else {
            printf("Wifi link failure\n");
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
            exit(1);
        }
    }
}

int create_socket() {
    // Create a UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        printf("Failed to create socket: error %d\n", errno);
        return 0;
    }

    // Set up the destination address
    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(8000);
    target_addr.sin_addr.s_addr = inet_addr(TARGET_IP);

    return 1;
}

void main_task(__unused void *params)
{
    if (cyw43_arch_init())
    {
        printf("failed to initialise\n");
        return;
    }
    sleep_ms(2000);

    // blink led while trying to connect the wifi
    TaskHandle_t blink_task;
    xTaskCreate(blink_led_task, "BlinkThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &blink_task);

    cyw43_arch_enable_sta_mode();
    printf("Connecting to Wi-Fi...\n");
    
    // make sure to set wifi ssid and password in the cmake environment
    // must hover mouse over these to make sure the values are correct!
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000))
    {
        printf("failed to connect.\n");
        exit(1);
    }
    else
    {
        // stop blinking led on connection success. keep it turned on
        vTaskDelete(blink_task);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        printf("Connected.\n");

        // successfully created a udp socket? start creating random data to send!
        if(create_socket() == 1) {
            create_data();
        } else {
            exit(1);
        }
    }

    cyw43_arch_deinit();
}

void vLaunch(void)
{
    TaskHandle_t task;
    xTaskCreate(main_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &task);
    TaskHandle_t send_task;
    xTaskCreate(send_data_task, "SendThread", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, &send_task);
    
    xSendMessageBuffer = xMessageBufferCreate(mbaTASK_MESSAGE_BUFFER_SIZE);

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