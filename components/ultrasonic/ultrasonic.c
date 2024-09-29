#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
#define TRIG_PIN 1
#define ECHO_PIN 0
#define MAX_RANGE 400.0f
#define SOUND_SPEED 0.0343f
#define TIMEOUT_US 50000

absolute_time_t start;
absolute_time_t end;

static float obstacle_distance = 0.0f;

void echo_callback(uint gpio, uint32_t events);

void ultrasonic_init(){
    // Setup
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    // Make sure TRIG PIN is at 0
    gpio_put(TRIG_PIN, 0);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    // Interrupt when there is rise or fall
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &echo_callback);
}

void sendPulse(){
    gpio_put(TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(TRIG_PIN, 0);
}

bool ultrasonic_timer_callback(struct repeating_timer *t){
    sendPulse(TRIG_PIN);
    return true;
}

float calculate_obstacle_distance(uint64_t pulse){
    return pulse * SOUND_SPEED / 2;
}

void echo_callback(uint gpio, uint32_t events){
    if (events == GPIO_IRQ_EDGE_RISE && gpio == ECHO_PIN){
        start = get_absolute_time();
    }

    if (events == GPIO_IRQ_EDGE_FALL && gpio == ECHO_PIN){
        end = get_absolute_time();
        if (absolute_time_diff_us(start, end) > TIMEOUT_US){
            start = end;
        }
        obstacle_distance = calculate_obstacle_distance(absolute_time_diff_us(start, end));
    }
}

float get_obstacle_distance(){
    return obstacle_distance;
}
