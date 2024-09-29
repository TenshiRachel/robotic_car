#include "pico/stdlib.h"
#include "hardware/gpio.h"

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
#define LEFT_ENCODER_PIN 2
#define RIGHT_ENCODER_PIN 3
#define NOTCHES_CM 1.05f // Circumference 21cm, 20 notches, therefore 1 notch approx 21/20cm

static uint32_t pulses_left = 0;
static uint32_t pulses_right = 0;
static float pulse_width_left;
static float pulse_width_right;
static absolute_time_t last_time_left;
static absolute_time_t last_time_right;
static float speed;
static float total_distance = 0.0f;

void encoder_irq_callback(uint gpio, uint32_t events){
    absolute_time_t current_time = get_absolute_time();
    if (gpio == LEFT_ENCODER_PIN){
        pulses_left++;
        int64_t time_diff = absolute_time_diff_us(last_time_left, current_time);
        pulse_width_left = (float) (time_diff/1000000.0f);
        last_time_left = current_time;
    }
    if (gpio == RIGHT_ENCODER_PIN){
        pulses_right++;
        int64_t time_diff = absolute_time_diff_us(last_time_right, current_time);
        pulse_width_right = (float) (time_diff/1000000.0f);
        last_time_right = current_time;
    }
}

void set_speed_distance(){
    speed = ((pulses_left + pulses_right) / 2) * NOTCHES_CM;
    total_distance += ((pulses_left + pulses_right) / 2) * NOTCHES_CM;

    float left_speed = 0;
    float right_speed = 0;

    left_speed = (pulse_width_left > 0) ? NOTCHES_CM/pulse_width_left : 0;
    right_speed = (pulse_width_right > 0) ? NOTCHES_CM/pulse_width_right : 0;
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

float get_speed(){
    return speed;
}

float get_travelled_distance(){
    return total_distance;
}