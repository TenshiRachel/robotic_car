#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdio.h>

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 8
#define NOTCHES_CM 1.05f // Circumference 21cm, 20 notches, therefore 1 notch approx 21/20cm

static uint32_t pulses_left = 0;
static uint32_t pulses_right = 0;
static float pulse_width_left;
static float pulse_width_right;
static absolute_time_t last_time_left;
static absolute_time_t last_time_right;
static volatile float speed;
static volatile float total_distance = 0.0f;

void set_speed_distance(){
    // Get average 
    total_distance += ((pulses_left + pulses_right) / 2) * NOTCHES_CM;

    float left_speed = (pulse_width_left > 0) ? NOTCHES_CM/pulse_width_left : 0;
    float right_speed = (pulse_width_right > 0) ? NOTCHES_CM/pulse_width_right : 0;
    // printf("Speed left: %.2fcm/s\n", left_speed);
    // printf("Speed right: %.2fcm/s\n", right_speed);

    speed = (left_speed + right_speed) /2;
}

void encoder_irq_callback(uint gpio, uint32_t events){
    absolute_time_t current_time = get_absolute_time();
    if (gpio == LEFT_ENCODER_PIN || gpio == RIGHT_ENCODER_PIN){
        if (gpio == LEFT_ENCODER_PIN){
            pulses_left++;
            int64_t time_diff = absolute_time_diff_us(last_time_left, current_time);
            pulse_width_left = (float) (time_diff/1000000.0f); // Convert to seconds
            last_time_left = current_time;
        }
        if (gpio == RIGHT_ENCODER_PIN){
            pulses_right++;
            int64_t time_diff = absolute_time_diff_us(last_time_right, current_time);
            pulse_width_right = (float) (time_diff/1000000.0f);
            last_time_right = current_time;
        }
        set_speed_distance();
        // printf("Speed: %.2f cm/s\nDistance travelled: %.2fcm\n", speed, total_distance);
    }

}

void wheel_encoder_init(){
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_callback);
    last_time_left = get_absolute_time();

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);
    gpio_set_irq_enabled_with_callback(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true, &encoder_irq_callback);
    last_time_right = get_absolute_time();
}
