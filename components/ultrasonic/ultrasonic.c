#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
#define TRIG_PIN 1
#define ECHO_PIN 0
#define SAFETY_THRESHOLD 10.0f
#define MAX_RANGE 400.0f
#define SOUND_SPEED 0.0343f

void ultrasonic_init(){
    // Setup
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    // Make sure TRIG PIN is at 0
    gpio_put(TRIG_PIN, 0);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
}

float get_obstacle_distance(){
    gpio_put(TRIG_PIN, 0);
    sleep_us(2);
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);

    uint64_t width = 0;

    absolute_time_t start = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 0){
        // Timeout after 0.05s
        if (absolute_time_diff_us(start, get_absolute_time()) > 50000){
            return MAX_RANGE;
        }
    }

    // Measure width of pulse
    absolute_time_t pulse_start = get_absolute_time();
    while (gpio_get(ECHO_PIN) == 1);
    absolute_time_t pulse_end = get_absolute_time();

    // Calculate distance based on speed of sound
    int64_t duration = absolute_time_diff_us(pulse_start, pulse_end);
    float distance = (duration * SOUND_SPEED) / 2;

    return distance > MAX_RANGE ? MAX_RANGE : distance;
}