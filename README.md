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
- **High-current Switches**: Reliable cotrol of external loards through robust switches.
- **Modular Connectivity**: Connects seamlessly to the main board via pin sockets.

---

### Visualization of PCB
- **Main Controller Board**:
  ![Main Controller PCB](Images/BOTTOM.png)
