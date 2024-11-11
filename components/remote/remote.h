// remote.h
#ifndef REMOTE_H
#define REMOTE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float R;  // Measurement noise covariance
    float Q;  // Process noise covariance
    float P;  // Estimate error covariance
    float K;  // Kalman gain
    float X;  // Filtered value (state estimate)
} KalmanFilter;

// Function declarations
void kalman_update(KalmanFilter *filter, float measurement);
float calculate_moving_average(float buffer[], int size);
void update_mag_buffer(float x, float y, float z);
void get_moving_average_mag(float* avg_x, float* avg_y, float* avg_z);
int calculate_speed_level(int16_t accel_y);
int calculate_turn_level(int16_t accel_x);
void generate_command(int16_t accel_x, int16_t accel_y);
void initialize_filtered_accel(float x, float y, float z);
float complementary_filter(float new_value, float prev_filtered_value);
float calculate_heading(float mag_x, float mag_y);
void init_i2c();
void init_accelerometer();
void init_magnetometer();
uint8_t read_register(uint8_t addr, uint8_t reg);
void read_accelerometer(int16_t* x, int16_t* y, int16_t* z);
void read_magnetometer(int16_t* x, int16_t* y, int16_t* z);
void apply_hybrid_filter(float raw_x, float raw_y, float raw_z);

// Variable declarations
extern float filtered_mag[3];
extern float filtered_heading;
extern float alpha;
extern float filtered_accel[3];
extern float mag_buffer[3];
extern int mag_index;
extern float last_accel_x;
extern float last_accel_y;
extern float last_accel_z;
extern bool instant_override;

extern KalmanFilter kf_x;
extern KalmanFilter kf_y;
extern KalmanFilter kf_z;

// For sending data via wifi
#define COMMAND_BUFFER_SIZE 100
extern char command_buffer[COMMAND_BUFFER_SIZE];
extern bool command_ready; // Flag to indicate new command availability

const char* get_command_buffer(); // Function to retrieve the command buffer

#endif
