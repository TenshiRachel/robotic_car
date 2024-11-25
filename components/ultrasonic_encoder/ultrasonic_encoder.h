#ifndef ULTRASONIC_ENCODER_H
#define ULTRASONIC_ENCODER_H

void wheel_encoder_init();
void ultrasonic_init();
bool ultrasonic_timer_callback(struct repeating_timer *t);
void telemetryTask(__unused void *params);

extern volatile bool blocked;
extern volatile float obstacle_distance;
extern volatile float left_speed;
extern volatile float right_speed;
#endif