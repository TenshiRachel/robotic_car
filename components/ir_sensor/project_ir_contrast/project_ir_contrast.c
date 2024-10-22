#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

int colour = 2;
#define THRESHOLD 0.6
#define WHITE 0
#define BLACK 1

void __not_in_flash_func(adc_capture)(uint16_t *buf, size_t count) {
    adc_fifo_setup(true, false, 0, false, false);
    adc_run(true);
    for (int i = 0; i < count; i = i + 1)
        buf[i] = adc_fifo_get_blocking();
    adc_run(false);
    adc_fifo_drain();
}

int main(void) {
    stdio_init_all();
    adc_init();
    adc_gpio_init(26); // set pin as the gpio adc input
    
    // Set pin 26 to input
    gpio_set_dir(26, false);
    gpio_set_function(26, GPIO_FUNC_SIO);
    adc_select_input(0);  // Select ADC channel 0 for GPIO 26

    while (1) {
        uint32_t result = adc_read();
        const float conversion_factor = 3.3f / (1 << 12);
        const float converted_result = result * conversion_factor; // TODO: Optimise without multiplying
        if (converted_result > THRESHOLD) {
            if (colour != BLACK) {
                printf("Black surface detected\n");
                colour = 1;
            }
        } else {
            if (colour != WHITE) {
                printf("White surface detected\n");
                colour = 0;
            }
        }
        sleep_ms(1);
    }
}


// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"
// #include "hardware/adc.h"
// #include "hardware/irq.h"
// #include "hardware/timer.h"

// volatile int colour = 2;
// volatile uint32_t result = 0;

// // repeating timer
// bool adc_timer_callback(struct repeating_timer *t) {
//     result = adc_read();
    
//     const float conversion_factor = 3.3f / (1 << 12);
//     const float converted_result = result * conversion_factor;

//     if (converted_result > 0.6) {
//         if (colour != 1) {
//             printf("Black surface detected\n");
//             colour = 1;
//         }
//     } else {
//         if (colour != 0) {
//             printf("White surface detected\n");
//             colour = 0;
//         }
//     }
//     return true;  // returning true = the timer will repeat
// }

// int main(void) {
//     stdio_init_all();
//     adc_init();
//     adc_gpio_init(26);
//     adc_select_input(0);

//     struct repeating_timer timer;

//     add_repeating_timer_us(-100000, adc_timer_callback, NULL, &timer);

//     // Main loop does nothing, as the work is handled by the interrupt-like timer
//     while (1) {
//         tight_loop_contents();  // Wait for the timer to trigger
//     }
// }
