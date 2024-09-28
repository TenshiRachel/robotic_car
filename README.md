## INF2004 Robotic Car

Deliverables:

A robotic car with the followin features:
1. Motor Control System with PID Controller
2. IR Sensors for Line following and Barcode detecting
3. MPU9250 for Gesture Remote Control
4. WiFi Communication
5. Ultrasonic Sensor for Real-time Distance Measurement
6. Wheel Encoder for Speed and Distance Measurement

## Project Setup

### Prerequisites

- Raspberry Pi
- CMake, CMake Tools extensions
- C/C++ Extension
- Make sure you are working in 'Pico - Visual Studio Code'


### Steps

1. Clone the project
2. Open the robotic_car project folder with VS Code
3. If prompted to configure project, select 'Yes'
4. Look for 'Pico ARM GCC - Pico SDK Toolchain with GCC arm-none-eabi'
5. If not found, select unspecified
6. Build the main.c code in the CMake tab
7. Copy the uf2 file from the build folder to your Pico drive