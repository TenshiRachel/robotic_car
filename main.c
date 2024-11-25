#include <stdio.h>
#include "pico/stdlib.h"

// Components
#include "components/ultrasonic_encoder/ultrasonic_encoder.h"
#include "components/motor_control/motor_control.h"
#include "components/ir_sensor/ir_sensor.h"
#include "components/ir_sensor/ir_line_following.h"
#include "pico/cyw43_arch.h"

// Lib
// #include "FreeRTOS.h"
// #include "task.h"
// #include "message_buffer.h"
#include "components/wifi/car/wifi.h"
#define WHITE 0
#define BLACK 1
#define ON_LINE 2
#define mbaTASK_MESSAGE_BUFFER_SIZE (80) // message buffer size, increase as needed based on individual message size

static bool autonomous = false;
static bool barcodeTaskLaunched = false;
static int line_ir_poll_interval = 100;

void ultrasonicTask(__unused void *params){
    // Ultrasonic
    // Send pulse every 100ms
    struct repeating_timer timer;
    add_repeating_timer_ms(100, ultrasonic_timer_callback, NULL, &timer);
    while (1)
    {        
        vTaskDelay(portMAX_DELAY);
    }
    
}

void irBarcodeTask(__unused void *params){
    struct repeating_timer timer;
    add_repeating_timer_ms(1, process_barcode, NULL, &timer);
    while (1)
    {        
        vTaskDelay(portMAX_DELAY);
    }
}

void irTask(__unused void *params) {
    static bool readjustment = false;
    static int16_t white_counter = 0;
    while (1) {
        int line_state = read_line();  // Read sensor data
        if (!blocked) {
            // Control motors based on line state if needed
            if (line_state == WHITE && autonomous) {
                white_counter++;
                if (white_counter >= 20)
                {
                    readjustment = true;
                    turn_left(0.0f, 0.22f);
                } else
                {
                    // turn_left(0.0f,0.22f);
                    turn_right(0.22f,0.0f);
                }
            } else if (line_state == BLACK) {
                white_counter = 0;
                readjustment = false;
                if(!autonomous) {
                    autonomous = true;
                    line_ir_poll_interval = 500;
                } else {
                    // move_forward(0.48f,0.18f);
                    move_forward(0.38f,0.48f);
                }
            }
                // turn_right(0.5f,0.7f);
                // move_forward(0.55f,0.5f);
        }
        if (autonomous && !barcodeTaskLaunched) {
            TaskHandle_t infraBarCodeTask;
            xTaskCreate(irBarcodeTask, "barCodeThread", configMINIMAL_STACK_SIZE, NULL, 3, &infraBarCodeTask);
            barcodeTaskLaunched = true;
        }
    }
    vTaskDelay(pdMS_TO_TICKS(line_ir_poll_interval));  // Delay ?? ms between readings
}

// pid task
void pidTask(__unused void *params) {
    struct repeating_timer timer;
    add_repeating_timer_ms(10, pid_timer_callback, NULL, &timer);
    while (1)
    {
        vTaskDelay(portMAX_DELAY);
    }
}

void vLaunch( void){
    TaskHandle_t ultratask;
    xTaskCreate(ultrasonicTask, "ultrasonicThread", configMINIMAL_STACK_SIZE, NULL, 5, &ultratask);
    InitMessageBuffer();

    TaskHandle_t infraTask;
    xTaskCreate(irTask, "infraThread", configMINIMAL_STACK_SIZE, NULL, 3, &infraTask);

    // TaskHandle_t infraBarCodeTask;
    // xTaskCreate(irBarcodeTask, "barCodeThread", configMINIMAL_STACK_SIZE, NULL, 3, &infraBarCodeTask);
    TaskHandle_t task;
    xTaskCreate(wifi_and_server_task, "TestMainThread", configMINIMAL_STACK_SIZE, NULL, 3, &task);
    
    TaskHandle_t pidUpdateTask;
    xTaskCreate(pidTask, "pidThread", configMINIMAL_STACK_SIZE,NULL,3, &pidUpdateTask);

#if NO_SYS && configUSE_CORE_AFFINITY && configNUM_CORES > 1
    // we must bind the main task to one core (well at least while the init is called)
    // (note we only do this in NO_SYS mode, because cyw43_arch_freertos
    // takes care of it otherwise)
    vTaskCoreAffinitySet(task, 1);
#endif

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
}

int main(){
    stdio_init_all();
    sleep_ms(3000);

    // Init components
    // swapped wheel encoder and ultrasonic init!
    wheel_encoder_init();
    ultrasonic_init();
    
    motor_init();
    ir_init_barcode();
    ir_init_linefollow();

    const char *rtos_name;
    #if ( portSUPPORT_SMP == 1 )
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if ( configNUM_CORES == 2 )

    sleep_ms(3000);
    printf("Starting %s on both cores:\n", rtos_name);
    vLaunch();
#elif ( RUN_FREERTOS_ON_CORE == 1 )
    printf("Starting %s on core 1:\n", rtos_name);
    multicore_launch_core1(vLaunch);
    while (true);
#else
    printf("Starting %s on core 0:\n", rtos_name);
    vLaunch();
#endif
    return 0;
}
