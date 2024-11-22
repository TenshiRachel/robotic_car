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
#define SAFETY_THRESHOLD 18
// #define SAFETY_THRESHOLD 15
absolute_time_t start, end;

static float obstacle_distance = 0.0f;

volatile bool blocked = false;  // Flag to enable/disable callback
volatile bool turning = false; // track turning

// Kalman filter variables
static float estimate = 0.0f;
static float uncertainty = 1.0f;
const float Q = 0.01f;
const float R = 0.1f;


/* defines/static variables from wheel encoder are below */

// IMPORTANT!!! CHANGE BASED ON WHERE ITS PLUGGED FOR ACTUAL
// #define LEFT_ENCODER_PIN 2
#define LEFT_ENCODER_PIN 8
// #define RIGHT_ENCODER_PIN 26
#define RIGHT_ENCODER_PIN 2
#define TIMEOUT_MS 1000   // Timeout after 1 second to reset pulse widths
#define NOTCHES_CM 1.025f // Circumference 21cm, 20 notches, therefore 1 notch approx 20.5/20cm

volatile uint32_t pulses_left = 0;
volatile uint32_t pulses_right = 0;
volatile float left_speed = 0;
volatile float right_speed = 0;
volatile uint32_t pulse_required = 0; // Global to store the target pulse count for turning


static float pulse_width_left, pulse_width_right;
static absolute_time_t last_time_left, last_time_right;
static volatile float speed;
static volatile float total_distance = 0.0f;
float end_distance = 0.0f; // Station 1: to indicate end of 90cm mark


void sendPulse(){
    gpio_put(TRIG_PIN, 1);
    busy_wait_us(10);
    gpio_put(TRIG_PIN, 0);
}

// from wheel encoder
void set_speed_distance()
{
    // Get average
    float dist_left = pulses_left * NOTCHES_CM;
    float dist_right = pulses_right * NOTCHES_CM;
    total_distance = ((dist_left + dist_right) / 2);
    // printf("distance: %f, pulse left: %d, pulse right: %d \n", total_distance, pulses_left, pulses_right);

    left_speed = (pulse_width_left > 0) ? (NOTCHES_CM / pulse_width_left) * 1000 : 0;
    right_speed = (pulse_width_right > 0) ? (NOTCHES_CM / pulse_width_right) * 1000 : 0;
    // printf("Speed left: %.2fcm/s\n", left_speed);
    // printf("Speed right: %.2fcm/s\n", right_speed);

    speed = (left_speed + right_speed) / 2;
}

void shared_callback(uint gpio, uint32_t events){
    if (gpio == LEFT_ENCODER_PIN || gpio == RIGHT_ENCODER_PIN)
    {
        volatile absolute_time_t current_time = get_absolute_time();
        if (gpio == LEFT_ENCODER_PIN)
        {
            pulses_left++;
            volatile int64_t time_diff_left = absolute_time_diff_us(last_time_left, current_time);

            if (time_diff_left > TIMEOUT_MS * 1000)
            {
                pulse_width_left = 0.0f;
            }
            else
            {
                pulse_width_left = (float)(time_diff_left / 1000.0f); // Convert to ms
                // printf("Pulse width left: %f\n", pulse_width_left);
            }
            // printf("Pulses Left: %u\n", pulses_left);

            last_time_left = current_time;
        }
        if (gpio == RIGHT_ENCODER_PIN)
        {
            pulses_right++;
            volatile int64_t time_diff_right = absolute_time_diff_us(last_time_right, current_time);

            if (time_diff_right > TIMEOUT_MS * 1000)
            {
                pulse_width_right = 0.0f;
            }
            else
            {
                pulse_width_right = (float)(time_diff_right / 1000.0f);
                // printf("Pulse width right: %f\n", pulse_width_right);
            }

            last_time_right = current_time;
        }
        set_speed_distance();

        // printf("Speed: %.2f cm/s\nDistance travelled: %.2fcm\n", speed, total_distance);
    } 

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
            obstacle_distance = raw_distance;
            // obstacle_distance = kalman_update(raw_distance);
            // printf("Distance to obstacle: %.2f\n", obstacle_distance);
            if (obstacle_distance <= SAFETY_THRESHOLD && !blocked){
                stop_motors();
                blocked = true;
            }

            else if (obstacle_distance > SAFETY_THRESHOLD && blocked){
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
    gpio_set_irq_enabled_with_callback(ECHO_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &shared_callback);
}

void wheel_encoder_init()
{
    gpio_init(LEFT_ENCODER_PIN);
    gpio_set_dir(LEFT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(LEFT_ENCODER_PIN);
    // Set the callback only for the first pin
    gpio_set_irq_enabled(LEFT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true);
    last_time_left = get_absolute_time();

    gpio_init(RIGHT_ENCODER_PIN);
    gpio_set_dir(RIGHT_ENCODER_PIN, GPIO_IN);
    gpio_pull_up(RIGHT_ENCODER_PIN);
    // Use gpio_set_irq_enabled for additional pins
    gpio_set_irq_enabled(RIGHT_ENCODER_PIN, GPIO_IRQ_EDGE_RISE, true);
    last_time_right = get_absolute_time();
}
