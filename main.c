#include <stdio.h>
#include "pico/stdlib.h"

// Components
#include "components/ultrasonic/ultrasonic.h"
#include "components/wheel_encoder/wheel_encoder.h"

int main(){
    stdio_init_all();

    // Init components
    ultrasonic_init();
    wheel_encoder_init();

    while (1){
        printf("Distance: %f\n", get_obstacle_distance());
    }
}
