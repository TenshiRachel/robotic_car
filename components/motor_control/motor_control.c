#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#include "components/wheel_encoder/wheel_encoder.h"
// Define GPIO pins for Motor A (LEFT)
#define PWM_PIN_A 14     // GP2 for Motor A PWM
#define DIR_PIN_A1 10    // GP0 for Motor A direction
#define DIR_PIN_A2 11    // GP1 for Motor A direction

// Define GPIO pins for Motor B (RIGHT)
#define PWM_PIN_B 15    // GP3 for Motor B PWM
#define DIR_PIN_B1 12    // GP4 for Motor B direction
#define DIR_PIN_B2 13   // GP5 for Motor B direction

volatile bool is_moving = false; // track if car is moving or not 

// Function to set up the PWM
void setup_pwm(uint gpio, float freq, float duty_cycle) {
    // Set the GPIO function to PWM
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(gpio);

    // Calculate the PWM frequency and set the PWM wrap value
    float clock_freq = 125000000.0f;  // Default Pico clock frequency in Hz
    uint32_t divider = clock_freq / (freq * 65536);  // Compute divider for given frequency
    pwm_set_clkdiv(slice_num, divider);

    // Set the PWM wrap value (maximum count value) -- for 65536 cycles
    pwm_set_wrap(slice_num, 65535);  // 16-bit counter (0 - 65535)

    // Set the duty cycle
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));

    // Enable the PWM
    pwm_set_enabled(slice_num, true);
}

// Function to move Motor A
void move_motor_A(float duty_cycle, bool forward){
    if (forward)
    {
    gpio_put(DIR_PIN_A1, 1);
    gpio_put(DIR_PIN_A2, 0);
    }
    else
    { // move backwards
    gpio_put(DIR_PIN_A1, 0);
    gpio_put(DIR_PIN_A2, 1);
    }

    setup_pwm(PWM_PIN_A, 100.0f, duty_cycle);    
}

// Function to move Motor B
void move_motor_B(float duty_cycle, bool forward){
    if (forward)
    {
    gpio_put(DIR_PIN_B1, 1);
    gpio_put(DIR_PIN_B2, 0);
    }

    else
    { // move backwards
    gpio_put(DIR_PIN_B1, 0);
    gpio_put(DIR_PIN_B2, 1);
    }

    setup_pwm(PWM_PIN_B, 100.0f, duty_cycle);    
}


// both motors forward (both directions forward)
void move_forward(float duty_cycle_A, float duty_cycle_B) {
    move_motor_A(duty_cycle_A, true); // move A forward
    move_motor_B(duty_cycle_B, true); // move B forward
}

// both motors backward (both directions backward)
void move_backward(float duty_cycle_A, float duty_cycle_B) {
    move_motor_A(duty_cycle_A, false); // move A backward
    move_motor_B(duty_cycle_B, false); // move B backward

}

// Function to stop both motors
void stop_motors() {
    gpio_put(DIR_PIN_A1, 0);
    gpio_put(DIR_PIN_A2, 0);
    gpio_put(DIR_PIN_B1, 0);
    gpio_put(DIR_PIN_B2, 0);
    is_moving = false;
}


// turn left (Motor A forward, Motor B backward)
void turn_left(float duty_cycle_A, float duty_cycle_B) {
    move_motor_A(duty_cycle_A,false);   // motor A backward
    move_motor_B(duty_cycle_B, true);   // Motor B forward
}

// turn right (Motor A backward, Motor B forward)
void turn_right(float duty_cycle_A, float duty_cycle_B){
    move_motor_A(duty_cycle_A, true);
    move_motor_B(duty_cycle_B,false);
}


// PID Constants for Motor A
float KpLeft = 0.21f; // Proportional: used to correct how far current is from the setpoint
float KiLeft = 0.03f; // Integral: accumulated error over time, if Proportional gain does not match setpoint, integral will make the adjustment
float KdLeft = 0.01f; // Derivative: predicts based on change/trend over time, prevent overshooting of value

// PID Constants for Motor B
float KpRight= 0.295f; // Proportional: used to correct how far current is from the setpoint
float KiRight = 0.03f; // Integral: accumulated error over time, if Proportional gain does not match setpoint, integral will make the adjustment
float KdRight = 0.01f; // Derivative: predicts based on change/trend over time, prevent overshooting of value

// track prev error and integral of respective motors
float previous_error_motorA = 0.0f;
float integral_motorA = 0.0f;

float previous_error_motorB = 0.0f;
float integral_motorB = 0.0f;

// setpoint for motors
float target_speed_motorA = 0.0f;
float target_speed_motorB = 0.0f;

// Function to compute the control signal
float compute_pid(float setpoint, float current_value, float *integral, float *prev_error, float Kp, float Ki, float Kd) {

    float error = setpoint - current_value;
    
    *integral += error;

    float derivative = error - *prev_error;

    float control_signal = Kp * error + Ki * (*integral) + Kd * derivative; 

    *prev_error = error;

    // set boundary so that calculated signal will be within PWM range
    if (control_signal >= 1.0f)
    {
        control_signal = 0.99f;
        *integral = 0;
    }
    else if (control_signal < 0.0f)
    {
        control_signal = 0.0f;
        *integral = 0;
    }

    return control_signal;
}


// both motors forward with PID
void move_up(){
    target_speed_motorA = 35.0f;
    target_speed_motorB = 35.0f;
    // get duty cycle based on PID
    float duty_cycle_A = compute_pid(target_speed_motorA, left_speed, &integral_motorA, &previous_error_motorA, KpLeft, KiLeft, KdLeft);
    float duty_cycle_B = compute_pid(target_speed_motorB, right_speed, &integral_motorB,&previous_error_motorB, KpRight, KiRight, KdRight);
    move_motor_A(duty_cycle_A, true); // move A forward with duty cycle from PID
    move_motor_B(duty_cycle_B, true); // move B forward with duty cycle from PID
    is_moving = true;
}

struct repeating_timer pid_timer;

bool pid_timer_callback() {
    if (is_moving) {
        // Compute PID output for Motor A
        float duty_cycle_A = compute_pid(target_speed_motorA, left_speed, 
                                         &integral_motorA, &previous_error_motorA, 
                                         KpLeft, KiLeft, KdLeft);
        
        // Compute PID output for Motor B
        float duty_cycle_B = compute_pid(target_speed_motorB, right_speed, 
                                         &integral_motorB, &previous_error_motorB, 
                                         KpRight, KiRight, KdRight);

        // Update motor speeds with the computed duty cycles
        move_motor_A(duty_cycle_A, true);
        move_motor_B(duty_cycle_B, true);
    }
    return true;
}

void motor_init(){
    
    // Initialize GPIO pins for Motor A and Motor B direction control
    gpio_init(DIR_PIN_A1);
    gpio_init(DIR_PIN_A2);
    gpio_set_dir(DIR_PIN_A1, GPIO_OUT); 
    gpio_set_dir(DIR_PIN_A2, GPIO_OUT);

    gpio_init(DIR_PIN_B1);
    gpio_init(DIR_PIN_B2);
    gpio_set_dir(DIR_PIN_B1, GPIO_OUT);
    gpio_set_dir(DIR_PIN_B2, GPIO_OUT);

    move_up(); // move forward with PID
}
