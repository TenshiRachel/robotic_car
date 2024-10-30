#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include <stdio.h>

#include "components/motor_control/motor_control.h"

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
// #define TRIG_PIN 1
// #define ECHO_PIN 0
#define TRIG_PIN 7
#define ECHO_PIN 6
#define MAX_RANGE 400.0f
#define SOUND_SPEED 0.0343f
#define TIMEOUT_US 26100
#define SAFETY_THRESHOLD 20

absolute_time_t start, end;

static float obstacle_distance = 0.0f;

// john
volatile bool blocked = false;  // Flag to enable/disable callback

// Kalman filter variables
static float estimate = 0.0f;
static float uncertainty = 1.0f;
const float Q = 0.01f;
const float R = 0.5f;

void sendPulse(){
    gpio_put(TRIG_PIN, 1);
    busy_wait_us(10);
    gpio_put(TRIG_PIN, 0);
}

float kalman_update(float distance){
    uncertainty += Q;

    float K = uncertainty / (uncertainty + R);

    estimate = estimate + K * (distance - estimate);

    uncertainty = (1 - K) * uncertainty;
    
    return estimate;
}

void echo_callback(uint gpio, uint32_t events){

    if ((events & GPIO_IRQ_EDGE_RISE) && gpio == ECHO_PIN){
        start = get_absolute_time();
    }

    if ((events & GPIO_IRQ_EDGE_FALL) && gpio == ECHO_PIN){
        end = get_absolute_time();
        if (absolute_time_diff_us(start, end) > TIMEOUT_US){
            start = end;
        }
        else{
            uint64_t pulse = absolute_time_diff_us(start, end);
            float raw_distance = pulse * SOUND_SPEED / 2;
            obstacle_distance = kalman_update(raw_distance);
            // printf("Distance to obstacle: %.2f\n", obstacle_distance);
            if (obstacle_distance <= SAFETY_THRESHOLD && !blocked){
                stop_motors();
                // sleep_ms(1000);
                // turn_right(0.6f,0.4f);
                blocked = true;
            }

            else if (obstacle_distance > SAFETY_THRESHOLD && blocked){
                move_forward(0.6f,0.6f);
                blocked = false;
            }
        }
    }
}

bool ultrasonic_timer_callback(struct repeating_timer *t){
    sendPulse();
    return true;
}


void ultrasonic_init(){
    // Setup
    gpio_init(TRIG_PIN);
    gpio_init(ECHO_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    // Make sure TRIG PIN is at 0
    gpio_put(TRIG_PIN, 0);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_up(ECHO_PIN);

    // Interrupt when there is rise or fall
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &echo_callback);
}

