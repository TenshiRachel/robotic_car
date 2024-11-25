#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include <stdbool.h>
int state = 3;

#define WHITE 0
#define BLACK 1
#define ON_LINE 2
#define GPIO_PIN 27

#include "ir_line_following.h"
int get_colour_line(uint32_t result);


void ir_init_linefollow(){
    gpio_set_dir(GPIO_PIN, false);
    gpio_set_function(GPIO_PIN, GPIO_FUNC_SIO);
}


bool read_line()
{
    return gpio_get(GPIO_PIN);
}


int get_colour_line(uint32_t result) {
    static uint32_t min_adc = 4095;
    static uint32_t max_adc = 0;
    static uint32_t contrast_adc = 4095;

    if (result < min_adc)
    {
        min_adc = result;
        contrast_adc = (uint32_t)((min_adc + max_adc)/2);
    }
    else if (result > max_adc)
    {
        max_adc = result;
        contrast_adc = (uint32_t)((min_adc + max_adc)/2);
    }
    if (result >= contrast_adc)
    {
        return BLACK;
    }
    else
    {
        return WHITE;
    }    
}
