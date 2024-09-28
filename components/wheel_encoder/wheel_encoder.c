#include "pico/stdlib.h"
#include "hardware/gpio.h"

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
#define ENCODER_PIN 2
#define CIRCUMFERENCE_CM 21.0f

static uint32_t pulses = 0;
static absolute_time_t last_time;
static float speed = 0.0f;
static float total_distance = 0.0f;

void encoder_irq_callback(uint gpio, uint32_t events){
    if (gpio == ENCODER_PIN){
        pulses++;
        absolute_time_t current_time = get_absolute_time();
        int64_t time_diff = absolute_time_diff_us(last_time, current_time);
        last_time = current_time;

        if (time_diff > 0){
            // Convert to cm
            speed = (CIRCUMFERENCE_CM / time_diff) * 1000000;
        }

        total_distance += CIRCUMFERENCE_CM;
    }
}

void wheel_encoder_init(){
    gpio_init(ENCODER_PIN);
    gpio_set_dir(ENCODER_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_callback);
    last_time = get_absolute_time();
}

float get_speed(){
    return speed;
}

float get_travelled_distance(){
    return total_distance;
}