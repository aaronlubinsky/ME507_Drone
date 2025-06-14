# ME-507 Drone Project

![Image](https://github.com/user-attachments/assets/0bce00c2-0306-40a0-9c8e-9f046839d04f)

## Overview
This is the documentation for a quadcopter design by Aaron Lubisnky and Dane Carroll. This project was taken on and guided by Cal Poly's ME507 course (Graduate Mechanical Control Systems Design) and instructed by lecturer, Charlie Refvem.

The drone has a custom designed and 3D printed frame. It has been designed to operate using a custom PCB including an onboard STM2F411CEU6 MCU and BNO055 IMU. Though, currently uses a Blackpill development board and Bosch BN055 development board due to in house PCB manufacturing failures. The arming sequence is tailored towards ReadyTosky ESCs and the bluetooth module (HC05) is intended to interact with a PC running a custom python script for drone control and plotting blackbox data.

## Hardware
The drone’s frame, fabricated using 3D printing technology, is lightweight yet robust, providing a stable platform for four brushless motors, each controlled by an individual ReadyToSky electronic speed controller. Power is supplied by a 3s or 4s LiPo battery, distributed through a 4-channel power distribution board to ensure consistent voltage delivery to all components. At the core of the electronics system is an STM32F411 development board (Blackpill), which serves as the primary microcontroller for processing sensor data and executing control algorithms. Orientation sensing is achieved using an Adafruit BNO055 IMU module, which provides 9-axis data (accelerometer, gyroscope, and magnetometer) in fusion mode for accurate roll, pitch, and yaw measurements. Wireless communication is enabled by an HC-05 Bluetooth module, allowing the drone to receive commands from a ground station and transmit blackbox data for real-time monitoring. Status LEDs are integrated to provide visual feedback on the drone’s operational state, such as arming status and error conditions. While the current setup relies on development boards due to PCB manufacturing issues, the design is optimized for a future transition to a custom PCB integrating the STM32F411CEU6 and BNO055, which will reduce size, weight, and potential points of failure.

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
The software for the ME-507 Drone Project is a sophisticated flight control system that integrates real-time sensor processing, control algorithms, and wireless communication. The core of the system runs on the STM32F411 development board, utilizing a finite state machine to manage the drone’s operational states, including initialization, sensor calibration, ESC arming, PID control updates, and blackbox data transmission. The flight control is implemented using a 3-axis PID control loop, with independent controllers for roll, pitch, and yaw to maintain stable flight. The BNO055 IMU operates in fusion mode, combining accelerometer, gyroscope, and magnetometer data to provide accurate orientation measurements. Motor control is achieved by generating PWM signals using the STM32’s hardware timers, ensuring precise and responsive adjustments to the brushless motors. Commands are received wirelessly from an Xbox controller via the HC-05 Bluetooth module, with a custom Python script running on a PC ground station parsing joystick inputs and mapping them to setpoints for angle (roll, pitch, yaw) and throttle. This Python script also handles blackbox data logging, enabling post-flight analysis of flight dynamics. The software is designed for modularity, allowing for easy integration of advanced control algorithms or additional sensors in future iterations. 

## Modeling and Tuning
![Image](https://github.com/user-attachments/assets/658c8ed7-c9fe-4352-9392-acb05f88b5b2)

The control system design for the ME-507 Drone Project relied on a dynamic model of the quadcopter to inform the tuning of its PID controllers. The PID tuning process involved a combination of analytical and empirical methods to balance responsiveness and stability. Initial tuning parameters were derived from the dynamic model, focusing on the roll and pitch dynamics, which were tested independently using step-response experiments. These tests involved applying controlled inputs to the drone and measuring its response via the BNO055 IMU, with data logged for pitch, roll, and yaw angles. The logged data was visualized using the Python ground station software, allowing the team to analyze overshoot, settling time, and steady-state error. Iterative tuning was performed by adjusting the proportional, integral, and derivative gains to minimize oscillations and improve tracking performance. 

![Image](https://github.com/user-attachments/assets/787cf0c6-adaf-4d6b-9fd1-31b2936815f2)

## Documentation
The complete codebase is documented using Doxygen. Visit the hosted documentation at:

[aaronlubinsky.github.io/ME507_Drone](https://aaronlubinsky.github.io/ME507_Drone/html/index.html)

## Future Improvements
To advance this project, several key improvements are planned to enhance its functionality, compactness, and performance. A primary goal is transitioning from the current Blackpill development board and Bosch BNO055 development board to a custom-designed PCB integrating the STM32F411CEU6 microcontroller and BNO055 IMU. This will streamline the electronics, reduce weight, and improve reliability by eliminating the dependency on external development boards, addressing the challenges faced with in-house PCB manufacturing failures. Additionally, the Python-based ground station software will be enhanced to include a more robust user interface, featuring real-time plotting of critical flight parameters such as orientation, throttle, and control effort, alongside interactive tuning capabilities for PID gains. Another potential upgrade would be integrating a more robust wireless communication protocol, such as Wi-Fi or a dedicated RF module, to replace the HC-05 Bluetooth module, which would increase range and data throughput.

## Acknowledgments
We thank Lecturer Charlie Refvem for his guidance and Cal Poly's ME-507 course for providing the framework to develop this project.
