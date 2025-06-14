# ME-507 Drone Project

![Image](https://github.com/user-attachments/assets/0bce00c2-0306-40a0-9c8e-9f046839d04f)

## Overview
This is the documentation for a quadcopter design by Aaron Lubisnky and Dane Carroll. This project was taken on and guided by Cal Poly's ME507 course (Graduate Mechanical Control Systems Design) and instructed by lecturer, Charlie Refvem.

The drone has a custom designed and 3D printed frame. It has been designed to operate using a custom PCB including an onboard STM2F411CEU6 MCU and BNO055 IMU. Though, currently uses a Blackpill development board and Bosch BN055 development board due to in house PCB manufacturing failures. The arming sequence is tailored towards ReadyTosky ESCs and the bluetooth module (HC05) is intended to interact with a PC running a custom python script for drone control and plotting blackbox data.

## Hardware
The drone consists of a custom electronics system powered by an STM32F411 development board. It uses a 3D-printed frame to house four brushless motors, ESCs, a power distribution board, and a LiPo battery. A 9-axis IMU is used for orientation sensing, and an HC-05 Bluetooth module allows wireless communication. 

**Hardware Components:**
- STM32F4 Development Board
- BNO055 IMU Module
- HC05 Bluetooth Module
- 4x Electronic Speed Controllers (ESCs)
- 4x 3 Phase Brushless Motors
- 4-channel Power Distribution Board
- 3s/4s Battery
- Status LEDs

## Software Implementation
The flight control system is built using a 3-axis PID loop, with separate PID controllers for roll, pitch, and yaw. Commands are received wirelessly from an Xbox controller via Bluetooth, with a Python script parsing joystick input and mapping it to setpoints for angle and throttle. The STM32’s main loop is structured as a finite state machine with stages for initialization, calibration, ESC arming, PID updates, and sending blackbox data. Orientation measurements are handled using the BNO055 9 axis IMU in fusion mode. Motor outputs are updated every control cycle, with PWM signals generated using STM32 timers. 

## Modeling and Tuning
![Image](github.com/aaronlubinsky/ME507_Drone/blob/main/DroneBlockDiagram.png)

A dynamic model of the drone was used to derive tuning parameters for the PID loops. Control effort and responsiveness were balanced using empirical tuning and log analysis. Pitch and roll dynamics were first tuned independently using step-response testing. Sensor data such as pitch, roll, and yaw angles were logged and plotted to visualize control performance. This allowed iterative tuning to reduce overshoot and steady-state error.

![Image](https://github.com/user-attachments/assets/787cf0c6-adaf-4d6b-9fd1-31b2936815f2)

## Documentation
The complete codebase is documented using Doxygen. Visit the hosted documentation at:

[aaronlubinsky.github.io/ME507_Drone](https://aaronlubinsky.github.io/ME507_Drone/html/index.html)

## Future Improvements
To advance this project, several key improvements are planned to enhance its functionality, compactness, and performance. A primary goal is transitioning from the current Blackpill development board and Bosch BNO055 development board to a custom-designed PCB integrating the STM32F411CEU6 microcontroller and BNO055 IMU. This will streamline the electronics, reduce weight, and improve reliability by eliminating the dependency on external development boards, addressing the challenges faced with in-house PCB manufacturing failures. Additionally, the Python-based ground station software will be enhanced to include a more robust user interface, featuring real-time plotting of critical flight parameters such as orientation, throttle, and control effort, alongside interactive tuning capabilities for PID gains. Another potential upgrade would be integrating a more robust wireless communication protocol, such as Wi-Fi or a dedicated RF module, to replace the HC-05 Bluetooth module, which would increase range and data throughput.

## Acknowledgments
We thank Lecturer Charlie Refvem for his guidance and Cal Poly's ME-507 course for providing the framework to develop this project.
