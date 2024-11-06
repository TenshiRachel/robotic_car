#include <stdio.h>
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "lwip/sockets.h"

#include "components/remote/remote.h"

// Configuration
#define TARGET_IP "192.168.4.1"
#define TARGET_PORT 8000
#define ACCEL_ADDRESS 0x19
#define MAG_ADDRESS 0x1E
#define BUF_SIZE 128
#define MESSAGE_BUFFER_SIZE 128  // Size of the message buffer

static int sock;
static struct sockaddr_in target_addr;
static MessageBufferHandle_t xMessageBuffer;

#define STATIONARY_Z 9

// WiFi Connection Task with LED Feedback
void wifi_connect_task(__unused void *params) {
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        vTaskDelete(NULL);
    }

    cyw43_arch_enable_sta_mode();

    // Blink LED to indicate connection attempt
    for (int i = 0; i < 20; i++) {  // Blink for 20 cycles
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(450));
    }

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("WiFi connection failed\n");
        cyw43_arch_deinit();
        vTaskDelete(NULL);
    }

    // Turn LED solid on connection success
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    printf("Connected to WiFi\n");

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("Failed to create socket\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        cyw43_arch_deinit();
        vTaskDelete(NULL);
    }

    memset(&target_addr, 0, sizeof(target_addr));
    target_addr.sin_family = AF_INET;
    target_addr.sin_port = htons(TARGET_PORT);
    target_addr.sin_addr.s_addr = inet_addr(TARGET_IP);

    // Keep the task alive with a delay to maintain WiFi and socket
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(100));  // Keeps the task active without consuming CPU time
    }
}

// Read Accelerometer and Magnetometer Data Task
void read_data_task(__unused void *params) {
    int16_t accel_x, accel_y, accel_z;
    int16_t mag_x, mag_y, mag_z;
    bool initialized = false;

    while (1) {
        read_accelerometer(&accel_x, &accel_y, &accel_z);
        read_magnetometer(&mag_x, &mag_y, &mag_z);
        update_mag_buffer((float)mag_x, (float)mag_y, (float)mag_z);
        float avg_mag_x, avg_mag_y, avg_mag_z;
        get_moving_average_mag(&avg_mag_x, &avg_mag_y, &avg_mag_z);

        apply_hybrid_filter(accel_x, accel_y, accel_z);

        filtered_mag[0] = complementary_filter(avg_mag_x, filtered_mag[0]);
        filtered_mag[1] = complementary_filter(avg_mag_y, filtered_mag[1]);
        filtered_mag[2] = complementary_filter(avg_mag_z, filtered_mag[2]);

        float current_heading = calculate_heading(filtered_mag[0], filtered_mag[1]);
        filtered_heading = complementary_filter(current_heading, filtered_heading);

        generate_command(filtered_accel[0], filtered_accel[1]);

        if (command_ready) {
            // Retrieve the movement command from remote.c
            const char *command = get_command_buffer();

            if (xMessageBufferSend(xMessageBuffer, command, strlen(command), pdMS_TO_TICKS(100)) != strlen(command)) {
                printf("Failed to send command over WiFi\n");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Send Movement Commands to Car Pico Task
void send_data_task(__unused void *params) {
    char sensor_data[BUF_SIZE];
    size_t received_bytes;

    if (sock < 0) {
        vTaskDelete(NULL);  // Exit if socket is invalid
    }

    while (1) {
        received_bytes = xMessageBufferReceive(xMessageBuffer, sensor_data, BUF_SIZE, portMAX_DELAY);
        if (received_bytes > 0) {
            if (sock < 0) {
                printf("Debug: Invalid socket before sendto, sock = %d\n", sock);
                continue;  // Skip sending if socket is invalid
            }

            int result = sendto(sock, sensor_data, received_bytes, 0, (struct sockaddr *)&target_addr, sizeof(target_addr));
            if (result < 0) {
                printf("Failed to send message: error %d, sock = %d\n", errno, sock);
            } else {
                printf("Accelerometer and magnetometer data sent successfully\n");
            }
        }
    }
}

void vLaunch() {
    xTaskCreate(wifi_connect_task, "WiFiTask", 2048, NULL, 1, NULL);
    xTaskCreate(read_data_task, "DataAcquisition", 2048, NULL, 1, NULL);
    xTaskCreate(send_data_task, "DataTransmission", 2048, NULL, 1, NULL);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUMBER_OF_CORES > 1
        // we must bind the main task to one core (well at least while the init is called)
        // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
        // takes care of it otherwise)
        vTaskCoreAffinitySet(task, 1);
#endif

    vTaskStartScheduler();
}

int main(void) {
    stdio_init_all();
    init_i2c();
    init_accelerometer();
    init_magnetometer();

    xMessageBuffer = xMessageBufferCreate(MESSAGE_BUFFER_SIZE);
    if (xMessageBuffer == NULL) {
        printf("Failed to create message buffer\n");
        return -1;
    }

    vLaunch();

    while (1) {
        tight_loop_contents();
    }
}
