#include <stdio.h>
#include "pico/stdlib.h"

// Components
#include "components/ultrasonic/ultrasonic.h"
#include "components/wheel_encoder/wheel_encoder.h"

#define SAFETY_THRESHOLD 20.0f

struct repeating_timer timer;

int main(){
    stdio_init_all();

    // Init components
    ultrasonic_init();
    wheel_encoder_init();

    // Ultrasonic
    // Send pulse every 100ms
    add_repeating_timer_ms(100, ultrasonic_timer_callback, NULL, &timer);

    while (1){
        printf("Distance to object: %.2fcm\n", get_obstacle_distance());
        sleep_ms(200);
    }
}
