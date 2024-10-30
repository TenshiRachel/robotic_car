#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <stdbool.h>
int state = 3;

#define WHITE 0
#define BLACK 1
#define ON_LINE 2
#define GPIO_PIN 26
#define ADC_CHANNEL 0

#include "ir_line_following.h"
int get_colour_line(uint32_t result);


// int main(void) {
//     stdio_init_all();
//     ir_init();
    
//     struct repeating_timer sampling_timer;
//     add_repeating_timer_ms(100, read_line, NULL, &sampling_timer);

//     while (1);
// }


void ir_init_linefollow(){
    adc_init();
    adc_gpio_init(GPIO_PIN); // set pin as the gpio adc input
    

    gpio_set_dir(GPIO_PIN, false);
    gpio_set_function(GPIO_PIN, GPIO_FUNC_SIO);
    adc_select_input(ADC_CHANNEL);
}


bool read_line()
{
    uint32_t result = adc_read();

    return get_colour_line(result);
}


int get_colour_line(uint32_t result) {
    // printf("ADC Value: %u", result);
    static uint32_t min_adc = 4095;
    static uint32_t max_adc = 0;
    static uint32_t left_threshold = 1365;  // (1/3 of 4095)
    static uint32_t right_threshold = 2730; // (2/3 of 4095)

    // Dynamically adjust min and max based on readings
    if (result < min_adc) {
        min_adc = result;
    } else if (result > max_adc) {
        max_adc = result;
    }

    // Recalculate thresholds based on the updated min and max values
    left_threshold = min_adc + (max_adc - min_adc) / 3;        // 1/3 of range
    right_threshold = min_adc + 2 * (max_adc - min_adc) / 3;   // 2/3 of range

    if (result <= left_threshold) {
        return WHITE;        // Off the line to the left
    } else if (result >= right_threshold) {
        return BLACK;        // Off the line to the right
    } else {
        return ON_LINE;
    }
}
