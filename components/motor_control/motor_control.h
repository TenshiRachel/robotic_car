#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

void motor_init();
void move_forward(float duty_cycle_A, float duty_cycle_B);
void move_backward(float duty_cycle_A, float duty_cycle_B);
void stop_motors();
void move_up();
void turn_right(float duty_cycle_A, float duty_cycle_B);
void turn_left(float duty_cycle_A, float duty_cycle_B);
bool pid_timer_callback();
extern volatile bool is_moving;



#endif