#include <stdio.h>
#include "pico/stdlib.h"

const uint IR_SENSOR_PIN = 26;
uint64_t timeout = 26100;
int state = 2;
#define WHITE 0
#define BLACK 1


void setupPins(uint pin) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
}

int main(void) {
    stdio_init_all();
    sleep_ms(5000);
    printf("Measuring pulse:\n");

    setupPins(IR_SENSOR_PIN);
    absolute_time_t startTime = get_absolute_time();

    while (1) {
        // Check the current state of the pin
        int currentState = gpio_get(IR_SENSOR_PIN);
        
        if (currentState != state) {
            // If there was a state change (HIGH to LOW or LOW to HIGH), calculate the pulse duration
            absolute_time_t endTime = get_absolute_time();
            uint64_t pulseDuration = absolute_time_diff_us(startTime, endTime);

            if (currentState == WHITE) {
                printf("BLACK Pulse Duration: %llu ms\n", pulseDuration/1000);
            } else {
                printf("WHITE Pulse Duration: %llu ms\n", pulseDuration/1000);
            }

            // Update the state and reset start time for the next pulse
            state = currentState;
            startTime = get_absolute_time(); // Reset the start time
        }

        sleep_ms(1);
    }
}


// INTERRUPT VERSION

// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/gpio.h"

// const uint IR_SENSOR_PIN = 26;
// volatile absolute_time_t startTime;
// volatile uint64_t pulseDuration = 0;
// volatile bool pulseMeasured = false;
// uint64_t debounceTime = 100;

// // Interrupt handler for the IR sensor pin
// void gpio_callback(uint gpio, uint32_t events) {
//     absolute_time_t currentTime = get_absolute_time();
    
//     if (events & GPIO_IRQ_EDGE_RISE) {
//         startTime = currentTime;  // Capture start time on rising edge
//     }
    
//     if (events & GPIO_IRQ_EDGE_FALL) {
//         uint64_t duration = absolute_time_diff_us(startTime, currentTime);

//         // debounce to ignore short pulses
//         if (duration > debounceTime) {
//             pulseDuration = duration;
//             pulseMeasured = true; // Flag
//         }
//     }
// }

// void setupPins(uint pin) {
//     gpio_init(pin);
//     gpio_set_dir(pin, GPIO_IN);
//     gpio_pull_down(pin); 
//     gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
// }

// int main(void) {
//     stdio_init_all();
//     sleep_ms(5000); 
//     printf("Measuring pulse with interrupts:\n");

//     setupPins(IR_SENSOR_PIN);

//     while (1) {
//         if (pulseMeasured) {
//             // Only print if a valid pulse has been measured
//             printf("Pulse Duration: %llu microseconds\n", pulseDuration);
//             pulseMeasured = false; // Reset the flag
//         }
//         sleep_ms(50);
//     }
// }
