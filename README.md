# DC Motor Controller for Row Seeder

## Project Description
This project focuses on the design and development of a modular DC motor controller system for a row seeder.
The system ensures precise control over seed dispensing and supports communication with external devices via CAN protocol.

The project consists of a main controller board an an extension board connected via 2.54mm pin sockets:
- **Main Controller Board**: Handles motor control, sensor readings and CAN communication protocol.
- **Extension Board**: Adds high-current switches and extra digital inputs for expanded functionality.

This modular approach allows future modifications for broader compatibility and functionality.

---

## Features
### Main Controller Board:
- **Microcontroller**: STM32L476RGT6 (low power, high-performance Cortex-M4).
- **DC Motor Control**: H-bridge driver with speed and direction regulation using PWM.
- **CAN Communication**: Communication interface enabled by an SN65HVD230 transceiver.
- **Sensor Inputs**:
  - Analog inputs (12V) for angle sensors.
  - Encoder inputs (5V and 12V) for speed control.

### Extension Board:
- **Digital Input Expansion**: Additional pins for external sensors or signals.
- **High-current Switches**: Reliable control of external loads through robust switches.
- **Modular Connectivity**: Connects seamlessly to the main board via pin sockets.

---

## PCB Renderings
- **Main Controller Board**:
  ![Main Controller PCB](Images/BOTTOM.png)

- **Extension Board**:
  ![Extension PCB (made by my friend :))](Images/UP.png)

## Results
- **PWM Motor Control**: Verified precise speed and direction control for the DC motor.
- **Sensor Readings**: Successfully processed analog inputs and encoders.
- **PID Algorithm**: Achieved stable motor speed with minimal overshoot and fast response.
  ![](Images/PID.png)
  *Example motor angular velocity step response (fs = 10Hz)*
- **CAN Communication**: Reliable message transmission with external systems.
  ![](Images/CAN.PNG)
  *Sending analog reading of distance sensor via CAN bus*


## Potential Improvements
- Support for a broader range of motors and sensors.
- Add more communication protocols and better diagnostic.
- Motor measurement for better motor control.
- Enhanced firmware for fault detection and recovery.

## Author
- **Name**: Krzysztof Tomicki
- **Contact**: tomicki.krzysztof.pv@gmail.com
  
