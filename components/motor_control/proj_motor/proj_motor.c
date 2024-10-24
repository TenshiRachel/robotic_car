#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Define GPIO pins for Motor A
#define PWM_PIN_A 2     // GP2 for Motor A PWM
#define DIR_PIN_A1 0    // GP0 for Motor A direction
#define DIR_PIN_A2 1    // GP1 for Motor A direction

// Define GPIO pins for Motor B
#define PWM_PIN_B 3     // GP3 for Motor B PWM
#define DIR_PIN_B1 4    // GP4 for Motor B direction
#define DIR_PIN_B2 5    // GP5 for Motor B direction

// Define GPIO pin for interrupt (for Week 8 Demo)
#define BTN_PIN 21
#define DEBOUNCE_TIME 200  // milliseconds
static absolute_time_t last_press_time;
static int motor_action_index = 0; // State variable to track the current action

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
    printf("moving forward, duty cycle A: %.2f, duty cycle B: %.2f\n", duty_cycle_A, duty_cycle_B);

}

// both motors backward (both directions backward)
void move_backward(float duty_cycle_A, float duty_cycle_B) {
    move_motor_A(duty_cycle_A, false); // move A backward
    move_motor_B(duty_cycle_B, false); // move B backward
    printf("moving backwards, duty cycle A: %.2f, duty cycle B: %.2f\n", duty_cycle_A, duty_cycle_B);

}

// Function to stop both motors
void stop_motors() {
    gpio_put(DIR_PIN_A1, 0);
    gpio_put(DIR_PIN_A2, 0);
    gpio_put(DIR_PIN_B1, 0);
    gpio_put(DIR_PIN_B2, 0);
    printf("stopping\n");
}

// // turn left (Motor A forward, Motor B backward)
// void turn_left(float duty_cycle_A, float duty_cycle_B) {
//     move_motor_A(duty_cycle_A, true);   // Motor A forward
//     move_motor_B(duty_cycle_B, false);  // Motor B backward
//     printf("turning left, duty cycle A: %.2f, duty cycle B: %.2f\n", duty_cycle_A, duty_cycle_B);
// }

// // turn right (Motor A backward, Motor B forward)
// void turn_right(float duty_cycle_A, float duty_cycle_B) {
//     move_motor_A(duty_cycle_A, false);  // Motor A backward
//     move_motor_B(duty_cycle_B, true);   // Motor B forward
//     printf("turning right, duty cycle A: %.2f, duty cycle B: %.2f\n", duty_cycle_A, duty_cycle_B);
// }


// GPIO interrupt callback function (Week 8 Demo)
void gpio_callback(uint gpio, uint32_t events) {
    absolute_time_t current_time = get_absolute_time();

    if (events & GPIO_IRQ_EDGE_FALL) {  // Button pressed
        if (absolute_time_diff_us(last_press_time, current_time) < DEBOUNCE_TIME * 1000) {
            return;  // Ignore if too soon
        }
        last_press_time = current_time;  // Update last pressed time

        // Call the function based on the current index
        switch (motor_action_index) {
            case 0:
                move_forward(0.5f, 0.5f);  // Move forward
                break;
            case 1:
                move_forward(0.8f,0.8f); // move forward faster
                break;
            case 2:
                move_forward(0.4f,0.4f); // move forward slower
                break;
            case 3:
                move_backward(0.5f, 0.5f); // Move backward
                break;
            case 4:
                move_backward(0.8f,0.8f); // move backward faster
                break;
            case 5:
                move_backward(0.4f,0.4f); // move backward slower
                break;                
            case 6:
                stop_motors();             // Stop motors
                break;
            // case 7:
            //     turn_left(0.5f, 0.4f);     // Turn left
            //     break;
            // case 8:
            //     turn_right(0.4f, 0.5f);    // Turn right
            //     break;
        }

        // Update the action index, wrapping around if necessary
        motor_action_index = (motor_action_index + 1) % 7;
    }
}

int main() {
    
    stdio_init_all();

    // Initialize GPIO pins for Motor A and Motor B direction control
    gpio_init(DIR_PIN_A1);
    gpio_init(DIR_PIN_A2);
    gpio_set_dir(DIR_PIN_A1, GPIO_OUT);
    gpio_set_dir(DIR_PIN_A2, GPIO_OUT);

    gpio_init(DIR_PIN_B1);
    gpio_init(DIR_PIN_B2);
    gpio_set_dir(DIR_PIN_B1, GPIO_OUT);
    gpio_set_dir(DIR_PIN_B2, GPIO_OUT);

    // Configure GP21 as input with pull-up resistor
    gpio_set_dir(BTN_PIN, GPIO_IN);
    gpio_set_pulls(BTN_PIN, true, false);

    // Enable GPIO interrupts for rising and falling edges on the GP21 button press/release
    gpio_set_irq_enabled_with_callback(BTN_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    
    move_forward(0.38f, 0.38f);  // Start motors , lowest speed should be 0.4f 

    while (true) {
        tight_loop_contents(); // wait for interrupt
    }

    return 0;
}
