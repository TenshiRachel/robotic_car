#ifndef ULTRASONIC_H
#define ULTRASONIC_H

void ultrasonic_init();
bool ultrasonic_timer_callback(struct repeating_timer *t);
float get_obstacle_distance();

#endif