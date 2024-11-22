#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "stdlib.h"
#include "string.h"

// I2C addresses for the accelerometer and magnetometer
#define ACCEL_ADDRESS 0x19
#define MAG_ADDRESS 0x1E

// Registers for accelerometer data
#define OUT_X_L_A 0x28
#define OUT_X_H_A 0x29
#define OUT_Y_L_A 0x2A
#define OUT_Y_H_A 0x2B
#define OUT_Z_L_A 0x2C
#define OUT_Z_H_A 0x2D

// Control register for accelerometer
#define CTRL_REG1_A 0x20
#define CTRL_REG4_A 0x23

// Registers for magnetometer data
#define OUT_X_L_M 0x04 
#define OUT_X_H_M 0x03 
#define OUT_Y_L_M 0x08 
#define OUT_Y_H_M 0x07 
#define OUT_Z_L_M 0x06 
#define OUT_Z_H_M 0x05

// Control register for magnetometer
#define MR_REG_M 0x02

// Z-axis reading when stationary
#define STATIONARY_Z 9 

// Number of samples for moving average calculation
#define MOVING_AVG_SIZE 5 

// Define a threshold for directional shift detection
#define DIRECTION_CHANGE_THRESHOLD 5.0 

// Minimum Y-axis tilt to trigger forward movement
#define FORWARD_THRESHOLD 4
// Minimum Y-axis tilt to trigger backward movement         
#define BACKWARD_THRESHOLD -4     
// Minimum X-axis tilt to trigger left/right turn  
#define TURN_THRESHOLD 3
// Minimum value to trigger stop       
#define STOP_THRESHOLD 10          

// Buffer to prevent fluctuations
#define HYSTERESIS 5  

// Maximum tilt angle
#define MAX_TILT_ANGLE 170

#define FORWARD_STATE 0
#define BACKWARD_STATE 1
#define LEFT_STATE 3
#define RIGHT_STATE 2
#define STOP_STATE 4

// Initialize complementary filter parameters
float alpha = 0.95;
float filtered_mag[3] = {0, 0, 0};
float filtered_heading = 0;

// Define filtered accelerometer data storage
float filtered_accel[3] = {0, 0, 0};

// Variables to store the last state
int last_speed = 0;
int last_turn_speed = 0;
int last_direction = 0;
int last_turn = 0;
int last_turn_direction = 0;

float mag_buffer[3][MOVING_AVG_SIZE] = {0};
int mag_index = 0;

// Variables to track the previous acceleration direction
float last_accel_x = 0.0;
float last_accel_y = 0.0;
float last_accel_z = STATIONARY_Z;

// Flag to determine if instantaneous value shd be used
bool instant_override = false;

typedef struct {
    float R;  // Measurement noise covariance
    float Q;  // Process noise covariance
    float P;  // Estimate error covariance
    float K;  // Kalman gain
    float X;  // Filtered value (state estimate)
} KalmanFilter;

// Kalman filter instance for each axis
KalmanFilter kf_x = {.R = 0.1, .Q = 0.05, .P = 1.0, .X = 0};
KalmanFilter kf_y = {.R = 0.1, .Q = 0.05, .P = 1.0, .X = 0};
KalmanFilter kf_z = {.R = 0.1, .Q = 0.05, .P = 1.0, .X = STATIONARY_Z};

#define COMMAND_BUFFER_SIZE 100
char command_buffer[COMMAND_BUFFER_SIZE];
bool command_ready = false;

void kalman_update(KalmanFilter *filter, float measurement) {
    filter->P += filter->Q;
    
    filter->K = filter->P / (filter->P + filter->R);  // Compute Kalman gain
    filter->X += filter->K * (measurement - filter->X);  // Update estimate with measurement
    filter->P *= (1 - filter->K);  // Update error covariance
}

float calculate_moving_average(float buffer[], int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

void update_mag_buffer(float x, float y, float z) {
    mag_buffer[0][mag_index] = x;
    mag_buffer[1][mag_index] = y;
    mag_buffer[2][mag_index] = z;
    mag_index = (mag_index + 1) % MOVING_AVG_SIZE;
}

void get_moving_average_mag(float* avg_x, float* avg_y, float* avg_z) {
    *avg_x = calculate_moving_average(mag_buffer[0], MOVING_AVG_SIZE);
    *avg_y = calculate_moving_average(mag_buffer[1], MOVING_AVG_SIZE);
    *avg_z = calculate_moving_average(mag_buffer[2], MOVING_AVG_SIZE);
}

// Function to determine speed level based on Y-axis tilt
// int calculate_speed_level(int16_t accel_y) {
//     static int last_speed = 0;  // Static variable to store the previous speed between calls
//     float max_forward_accel = 5;

//     float tilt_angle = (fabs(accel_y) / max_forward_accel) * 100;

//     if (tilt_angle > MAX_TILT_ANGLE) {
//         tilt_angle = MAX_TILT_ANGLE;
//     }

//     int speed = (int)((tilt_angle / MAX_TILT_ANGLE) * 100);

//     if (abs(speed - last_speed) > HYSTERESIS) {
//         last_speed = speed;
//     }

//     return last_speed;
// }

// // Function to determine turn level based on X-axis tilt
// int calculate_turn_level(int16_t accel_x) {
//     static int last_turn_speed = 0;  // Static variable to store the previous turn speed between calls
//     float max_turn_accel = 8;

//     float tilt_angle = (fabs(accel_x) / max_turn_accel) * 100;

//     if (tilt_angle > 100) {
//         tilt_angle = 100;
//     }

//     int turn_speed = (int)tilt_angle;

//     if (abs(turn_speed - last_turn_speed) > HYSTERESIS) {
//         last_turn_speed = turn_speed;
//     }

//     return last_turn_speed;
// }

// Function to generate commands based on the calculated speed and turn levels
void generate_command(int16_t accel_x, int16_t accel_y) {
    int direction = STOP_STATE;  // Default to stop

    if (accel_y > FORWARD_THRESHOLD) {
        direction = FORWARD_STATE;  
    } else if (accel_y < BACKWARD_THRESHOLD) {
        direction = BACKWARD_STATE; 
    } else if (accel_x > TURN_THRESHOLD) {
        direction = RIGHT_STATE;  
    } else if (accel_x < -TURN_THRESHOLD) {
        direction = LEFT_STATE; 
    }

    // Check if direction has changed
    if (direction != last_direction) {
        snprintf(command_buffer, COMMAND_BUFFER_SIZE, "%d", direction);
        printf("Command: %d\n", direction);

        last_direction = direction;  // Update the last known direction
        command_ready = true;  // Indicate that a new command is ready
    }
}

// Getter function for remote_wifi.c to access the command
const char* get_command_buffer() {
    // Reset the flag after reading
    command_ready = false;
    return command_buffer;
}

void initialize_filtered_accel(float x, float y, float z) {
    filtered_accel[0] = x;
    filtered_accel[1] = y;
    filtered_accel[2] = z;
}

// Complementary filter for each axis
float complementary_filter(float new_value, float prev_filtered_value) {
    return alpha * new_value + (1 - alpha) * prev_filtered_value;
}

// Calculate heading from filtered magnetometer data
float calculate_heading(float mag_x, float mag_y) {
    float heading = atan2(mag_y, mag_x) * 180.0 / M_PI;
    if (heading < 0) heading += 360;
    return heading;
}

// Function to initialize I2C
void init_i2c() {
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

// Function to initialize the accelerometer
void init_accelerometer() {
    uint8_t config[] = {CTRL_REG1_A, 0x57};
    i2c_write_blocking(i2c_default, ACCEL_ADDRESS, config, 2, false);
    uint8_t config_reg4[] = {CTRL_REG4_A, 0x30};
    i2c_write_blocking(i2c_default, ACCEL_ADDRESS, config_reg4, 2, false);
}

// Function to initialize the magnetometer
void init_magnetometer() {
    uint8_t config[] = {MR_REG_M, 0x00};
    i2c_write_blocking(i2c_default, MAG_ADDRESS, config, 2, false);
}

// Function to read value from a specific register
uint8_t read_register(uint8_t addr, uint8_t reg) {
    uint8_t val;
    i2c_write_blocking(i2c_default, addr, &reg, 1, true); // Write register to read from
    i2c_read_blocking(i2c_default, addr, &val, 1, false); // Read the value from register
    return val;
}

// Function to read accelerometer data
void read_accelerometer(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t x_l = read_register(ACCEL_ADDRESS, OUT_X_L_A);
    uint8_t x_h = read_register(ACCEL_ADDRESS, OUT_X_H_A);
    uint8_t y_l = read_register(ACCEL_ADDRESS, OUT_Y_L_A);
    uint8_t y_h = read_register(ACCEL_ADDRESS, OUT_Y_H_A);
    uint8_t z_l = read_register(ACCEL_ADDRESS, OUT_Z_L_A);
    uint8_t z_h = read_register(ACCEL_ADDRESS, OUT_Z_H_A);

    // Combine high and low bytes to get full 16-bit data
    *x = (int16_t)(x_h << 8 | x_l);
    *y = (int16_t)(y_h << 8 | y_l);
    *z = (int16_t)(z_h << 8 | z_l);

    // Apply scaling factor to convert raw data
    *x = *x / 130;
    *y = *y / 130;
    *z = *z / 130;
}

// Function to read magnetometer data
void read_magnetometer(int16_t* x, int16_t* y, int16_t* z) {
    uint8_t x_l = read_register(MAG_ADDRESS, OUT_X_L_M);
    uint8_t x_h = read_register(MAG_ADDRESS, OUT_X_H_M);
    uint8_t y_l = read_register(MAG_ADDRESS, OUT_Y_L_M);
    uint8_t y_h = read_register(MAG_ADDRESS, OUT_Y_H_M);
    uint8_t z_l = read_register(MAG_ADDRESS, OUT_Z_L_M);
    uint8_t z_h = read_register(MAG_ADDRESS, OUT_Z_H_M);

    // Combine high and low bytes to get full 16-bit data
    *x = (int16_t)(x_h << 8 | x_l);
    *y = (int16_t)(y_h << 8 | y_l);
    *z = (int16_t)(z_h << 8 | z_l);
}

void apply_hybrid_filter(float raw_x, float raw_y, float raw_z) {
    // Check for a significant change in direction
    if (fabs(raw_x - last_accel_x) > DIRECTION_CHANGE_THRESHOLD ||
        fabs(raw_y - last_accel_y) > DIRECTION_CHANGE_THRESHOLD ||
        fabs(raw_z - last_accel_z) > DIRECTION_CHANGE_THRESHOLD) {
        
        instant_override = true;
    } else {
        instant_override = false;
    }

    if (instant_override) {
        // Override the values
        initialize_filtered_accel(raw_x, raw_y, raw_z);
    } else {
        // Use Kalman filter for each axis
        kalman_update(&kf_x, raw_x);
        kalman_update(&kf_y, raw_y);
        kalman_update(&kf_z, raw_z);

        // Store filtered values from the Kalman filter
        filtered_accel[0] = kf_x.X;
        filtered_accel[1] = kf_y.X;
        filtered_accel[2] = kf_z.X;
    }

    // Store the last known raw values for the next comparison
    last_accel_x = raw_x;
    last_accel_y = raw_y;
    last_accel_z = raw_z;
}

// For testing (DO NOT DELETE)
// int main() {
//     stdio_init_all();
//     sleep_ms(2000);

//     // Initialize I2C, accelerometer and magnetometer
//     init_i2c();
//     init_accelerometer();
//     init_magnetometer();

//     int16_t accel_x, accel_y, accel_z;
//     int16_t mag_x, mag_y, mag_z;
//     bool initialized = false;

//     while (true) {
//         read_accelerometer(&accel_x, &accel_y, &accel_z);
//         // printf("Raw Accelerometer: X=%d, Y=%d, Z=%d\n", accel_x, accel_y, accel_z);
//         read_magnetometer(&mag_x, &mag_y, &mag_z);
//         update_mag_buffer((float)mag_x, (float)mag_y, (float)mag_z);
//         float avg_mag_x, avg_mag_y, avg_mag_z;
//         get_moving_average_mag(&avg_mag_x, &avg_mag_y, &avg_mag_z);

//         apply_hybrid_filter(accel_x, accel_y, accel_z);

//         // Apply complementary filter to magnetometer data
//         filtered_mag[0] = complementary_filter(avg_mag_x, filtered_mag[0]);
//         filtered_mag[1] = complementary_filter(avg_mag_y, filtered_mag[1]);
//         filtered_mag[2] = complementary_filter(avg_mag_z, filtered_mag[2]);

//         // Calculate heading using filtered magnetometer data
//         float current_heading = calculate_heading(filtered_mag[0], filtered_mag[1]);
//         filtered_heading = complementary_filter(current_heading, filtered_heading);

//         // Print filtered data
//         // printf("Filtered Magnetometer: X=%.2f, Y=%.2f, Z=%.2f, Heading=%.2f deg\n", filtered_mag[0], filtered_mag[1], filtered_mag[2], filtered_heading);

//         // Use the filtered accelerometer data to generate commands
//         generate_command(filtered_accel[0], filtered_accel[1]);

//         sleep_ms(30);
//     }

//     return 0;
// }