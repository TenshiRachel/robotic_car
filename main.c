#include <stdio.h>
#include "pico/stdlib.h"

// Components
#include "components/ultrasonic/ultrasonic.h"
#include "components/wheel_encoder/wheel_encoder.h"
#include "components/motor_control/motor_control.h"
// #include "components/ir_sensor/ir_sensor.h"
#include "components/ir_sensor/ir_line_following.h"

// Lib
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"

#define WHITE 0
#define BLACK 1
#define ON_LINE 2


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

// void irTask(__unused void *params){
//     struct repeating_timer timer;
//     add_repeating_timer_ms(10, read_line, NULL, &timer);
//     while (1) {        
//         vTaskDelay(pdMS_TO_TICKS(10));  
//     }
// }

void irTask(__unused void *params) {
    while (1) {
        int line_state = read_line();  // Read sensor data every 10 ms

        // Control motors based on line state if needed
        if (line_state == WHITE) {
            printf("too white. turn left/\n");
        } else if (line_state == BLACK) {
            printf("too black. turn left/right\n");
        }

        vTaskDelay(pdMS_TO_TICKS(10));  // Delay 10 ms between readings
    }
}

void vLaunch( void){
    TaskHandle_t ultratask;
    xTaskCreate(ultrasonicTask, "ultrasonicThread", configMINIMAL_STACK_SIZE, NULL, 2, &ultratask);
    // TaskHandle_t infraTask;
    // xTaskCreate(irTask, "infraThread", configMINIMAL_STACK_SIZE, NULL, 3, &infraTask);

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

    // Init components
    ultrasonic_init();
    // wheel_encoder_init();
    motor_init();
    ir_init_linefollow();

    const char *rtos_name;
    #if ( portSUPPORT_SMP == 1 )
    rtos_name = "FreeRTOS SMP";
#else
    rtos_name = "FreeRTOS";
#endif

#if ( portSUPPORT_SMP == 1 ) && ( configNUM_CORES == 2 )
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
