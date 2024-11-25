#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"
#include "motor_control.h"

#include "components/ultrasonic_encoder/ultrasonic_encoder.h"
// Define GPIO pins for Motor A (LEFT)
#define LEFT_MOTOR_ENA 14     // GP2 for Motor A PWM
#define LEFT_MOTOR_IN1 10    // GP0 for Motor A direction
#define LEFT_MOTOR_IN2 11    // GP1 for Motor A direction

// Define GPIO pins for Motor B (RIGHT)
#define RIGHT_MOTOR_ENB 15    // GP3 for Motor B PWM
#define RIGHT_MOTOR_IN3 12    // GP4 for Motor B direction
#define RIGHT_MOTOR_IN4 13   // GP5 for Motor B direction

volatile bool is_moving = false; // track if car is moving or not 

volatile bool is_reversing = false; // track if car moving back

volatile bool autonomous = false;

// Function to set up the PWM
// void setup_pwm(uint gpio, float freq, float duty_cycle) {
void setup_pwm(uint gpio, float freq) {
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

    // // Set the duty cycle
    // pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));

    // Enable the PWM
    pwm_set_enabled(slice_num, true);
}

void set_duty_cycle(uint gpio, float duty_cycle)
{
    pwm_set_gpio_level(gpio, (uint16_t)(duty_cycle * 65536));

}

// Function to move LEFT motor
void move_left_motor(float duty_cycle, bool forward){
    if (forward)
    {
    gpio_put(LEFT_MOTOR_IN1, 1);
    gpio_put(LEFT_MOTOR_IN2, 0);
    }
    else
    { // move backwards
    gpio_put(LEFT_MOTOR_IN1, 0);
    gpio_put(LEFT_MOTOR_IN2, 1);
    }
    set_duty_cycle(LEFT_MOTOR_ENA,duty_cycle);
 
}

// Function to move RIGHT motor
void move_right_motor(float duty_cycle, bool forward){
    if (forward)
    {
    gpio_put(RIGHT_MOTOR_IN3, 1);
    gpio_put(RIGHT_MOTOR_IN4, 0);
    }

    else
    { // move backwards
    gpio_put(RIGHT_MOTOR_IN3, 0);
    gpio_put(RIGHT_MOTOR_IN4, 1);
    }

    set_duty_cycle(RIGHT_MOTOR_ENB, duty_cycle);   
}


// both motors forward (both directions forward)
void move_forward(float duty_cycle_A, float duty_cycle_B) {
    move_left_motor(duty_cycle_A, true); // move A forward
    move_right_motor(duty_cycle_B, true); // move B forward
}

// both motors backward (both directions backward)
void move_backward(float duty_cycle_A, float duty_cycle_B) {
    move_left_motor(duty_cycle_A, false); // move A backward
    move_right_motor(duty_cycle_B, false); // move B backward

}

// Function to stop both motors
void stop_motors() {
    gpio_put(LEFT_MOTOR_IN1, 0);
    gpio_put(LEFT_MOTOR_IN2, 0);
    gpio_put(RIGHT_MOTOR_IN3, 0);
    gpio_put(RIGHT_MOTOR_IN4, 0);
    is_moving = false;
    is_reversing = false;
}


void turn_left(float duty_cycle_A, float duty_cycle_B) {
    move_left_motor(duty_cycle_A,false);   // LEFT motor backward
    move_right_motor(duty_cycle_B, true);   // RIGHT motor forward
}

// turn right (Motor A backward, Motor B forward)
void turn_right(float duty_cycle_A, float duty_cycle_B){
    move_left_motor(duty_cycle_A, true); // LEFT motor forward
    move_right_motor(duty_cycle_B,false); // RIGHT motor backward
}


// PID Constants for Motor A
float KpLeft = 0.17f; // Proportional: used to correct how far current is from the setpoint
float KiLeft = 0.00f; // Integral: accumulated error over time, if Proportional gain does not match setpoint, integral will make the adjustment
// float KiLeft =0.005f;
// float KiLeft = 0.01f; // Integral: accumulated error over time, if Proportional gain does not match setpoint, integral will make the adjustment
float KdLeft = 0.00f; // Derivative: predicts based on change/trend over time, prevent overshooting of value

// PID Constants for Motor B
float KpRight= 0.16f; // Proportional: used to correct how far current is from the setpoint
float KiRight = 0.01f; // Integral: accumulated error over time, if Proportional gain does not match setpoint, integral will make the adjustment
float KdRight = 0.00f; // Derivative: predicts based on change/trend over time, prevent overshooting of value

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
    
    float max_integral = 10.0f; // Adjust this limit as needed
    if (*integral > max_integral) *integral = max_integral;
    else if (*integral < -max_integral) *integral = -max_integral;


    float motor_response = control_signal * 0.1;  // Motor response model
    //printf("Control Signal: %f , Motor Response: %f, current value: %f\n", control_signal,motor_response,current_value);    
    return motor_response;
}


// both motors forward with PID
void move_up(){
    target_speed_motorA = 40.0f;
    target_speed_motorB = 40.0f;
    // printf("Left spd: %f, Right spd: %f\n", left_speed, right_speed);
    // get duty cycle based on PID
    float duty_cycle_A = compute_pid(target_speed_motorA, left_speed, &integral_motorA, &previous_error_motorA, KpLeft, KiLeft, KdLeft);
    float duty_cycle_B = compute_pid(target_speed_motorB, right_speed, &integral_motorB,&previous_error_motorB, KpRight, KiRight, KdRight);

    move_left_motor(duty_cycle_A, true); // move A forward with duty cycle from PID
    move_right_motor(duty_cycle_B, true); // move B forward with duty cycle from PID
    
    is_moving = true;
}

void move_back()
{
    target_speed_motorA = 40.0f;
    target_speed_motorB = 40.0f;
    float duty_cycle_A = compute_pid(target_speed_motorA,left_speed,&integral_motorA, &previous_error_motorA, KpLeft, KiLeft, KdLeft);
    float duty_cycle_B = compute_pid(target_speed_motorB,right_speed, &integral_motorB, &previous_error_motorB, KpRight, KiRight, KdRight);

    move_left_motor(duty_cycle_A, false);
    move_right_motor(duty_cycle_B, false);

    is_reversing = true;


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
        move_left_motor(duty_cycle_A, true);
        move_right_motor(duty_cycle_B, true);
    }
    else if (is_reversing)
    {
        // Compute PID output for Motor A
        float duty_cycle_A = compute_pid(target_speed_motorA, left_speed, 
                                         &integral_motorA, &previous_error_motorA, 
                                         KpLeft, KiLeft, KdLeft);
        
        // Compute PID output for Motor B
        float duty_cycle_B = compute_pid(target_speed_motorB, right_speed, 
                                         &integral_motorB, &previous_error_motorB, 
                                         KpRight, KiRight, KdRight);

        // Update motor speeds with the computed duty cycles
        move_left_motor(duty_cycle_A, false);
        move_right_motor(duty_cycle_B, false);        
    }
    return true;
}

void motor_init(){
    
    // Initialize GPIO pins for Motor A and Motor B direction control
    gpio_init(LEFT_MOTOR_IN1);
    gpio_init(LEFT_MOTOR_IN2);
    gpio_set_dir(LEFT_MOTOR_IN1, GPIO_OUT); 
    gpio_set_dir(LEFT_MOTOR_IN2, GPIO_OUT);

    gpio_init(RIGHT_MOTOR_IN3);
    gpio_init(RIGHT_MOTOR_IN4);
    gpio_set_dir(RIGHT_MOTOR_IN3, GPIO_OUT);
    gpio_set_dir(RIGHT_MOTOR_IN4, GPIO_OUT);
    
    setup_pwm(LEFT_MOTOR_ENA, 100.0f);
    setup_pwm(RIGHT_MOTOR_ENB, 100.0f);    
}

void process_command_with_speed(const int command) {
    // printf("Autonomous: %d, Command: %d\n", autonomous, command);

    if (autonomous) {
        return;
    }

    // printf("Command: %d\n", command);
        switch (command) {
            case 0:  // Forward
                // printf("Forward\n");
                stop_motors();
                move_up(); // use PID to move
                break;

            case 1:  // Backward
                // printf("Backward\n");
                stop_motors();
                move_back(); // use PID to move backwards
                break;

            case 2:  // Turn Right
                // printf("Right\n");
                stop_motors();
                turn_right(0.3f, 0.2f);  
                break;

            case 3:  // Turn Left
                // printf("Left\n");
                stop_motors();
                turn_left(0.38f, 0.28f);  

                break;

            case 4: // Stop car
                // printf("Stop\n");
                stop_motors();
                break;

            default:
                printf("Unknown action: %d\n", command); 
                break;
        }
}
