## INF2004 Robotic Car

### Deliverables

A robotic car with the following features:
1. Motor Control System with PID Controller
2. IR Sensors for Line following and Barcode detecting
3. MPU9250 for Gesture Remote Control
4. WiFi Communication
5. Ultrasonic Sensor for Real-time Distance Measurement
6. Wheel Encoder for Speed and Distance Measurement

### Block Diagram
![Block Diagram](https://github.com/TenshiRachel/robotic_car/blob/master/block_diagram.png)

## Project Setup

### Prerequisites

- Raspberry Pi
- CMake, CMake Tools extensions
- C/C++ Extension
- Make sure you are working in 'Pico - Visual Studio Code'
- Make sure the folder is called 'robotic_car'
- 3 Picos (For Car, Remote, and Dashboard)
- Change the parameters in 'wifi.c' line 190 to your Wifi or Hotspot SSID and password

```
cyw43_arch_wifi_connect_timeout_ms(YOUR_HOTSPOT_SSID, YOUR_HOTSPOT_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)
```

- Change the WIFI_SSID and WIFI_PASSWORD in 'CMakeLists.txt' line 65-66 and 101-102 to your Wifi or Hotspot SSID and password

```
target_compile_definitions(robotic_car_wifi PRIVATE
        WIFI_SSID=\"Matt\"
        WIFI_PASSWORD=\"whyyoustealingmydata\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        PING_USE_SOCKETS=1
        )

target_compile_definitions(remote_wifi PRIVATE
        STATIC_IP=1    # uncomment this to disable dhcp client and use static ip addr. must be used in conjunction with PICO_ROLE
        PICO_ROLE=1    # uncomment this to set the appropriate static ip. refer to lwipopts_examples_common.h for more info
        WIFI_SSID=\"Matt\"
        WIFI_PASSWORD=\"whyyoustealingmydata\"
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )
```


### Steps

1. Clone the project
2. Open the robotic_car project folder with VS Code
3. If prompted to configure project, select 'Yes'
4. Look for 'Pico ARM GCC - Pico SDK Toolchain with GCC arm-none-eabi'
5. If not found, select unspecified
6. Build the robotic_car (Executable) in the CMake tab
7. Copy the uf2 file from the build folder into your Robotic car Pico's drive
8. Build the remote_wifi (Executable) in the CMake tab
9. Copy the uf2 file from the build folder into your Remote controller Pico's drive
10. Build the dashboard (Executable) in the CMake tab
11. Copy the uf2 file from the build folder into your Dashboard Pico's drive

### Establishing connection between Picos
Connection state is indicated on LED of Remote control and Dashboard Picos

#### LED states:
- Blinking (Looking for connection)
- Off (Connecting)
- On (Connected)

### Remote controls
- Move forward (Tilt down)
- Move backwards (Tilt up)
- Stop (Flat position aligned with flat surface)
- Turn left (Tilt left)
- Turn right (Tilt right)

## Functions
1. Remote controller to control car over Wifi (Hotspot) and Magnetometer
2. Dashboard to monitor speed, distance covered and barcode scanning output
through Serial Monitor
3. Switching to auto mode upon detecting a line
4. Line following and barcode reading through IR Sensor
5. Speed and distance tracking through Wheel Encoders
6. Emergency braking upon detecting obstacle within 10cm
7. Dynamic motor system using pid
