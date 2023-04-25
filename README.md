# Field Oriented Motor Controller

This project is a field-oriented control (FOC) implementation for brushless DC (BLDC) motors using an Arduino microcontroller. The FOC algorithm is a modern and efficient approach to controlling BLDC motors, offering precise control of torque, speed, and position. The project uses the SimpleFOC library and integrates with the Stealth Controller and DRV8316 motor driver for optimized motor control performance.

## Features

- Field-oriented control for precise and efficient BLDC motor control
- Stealth Controller integration for enhanced control capabilities
- DRV8316 motor driver support for driving 6-PWM BLDC motors
- MA702 magnetic sensor for accurate rotor position sensing
- CAN and Serial communication for commanding and monitoring the motor
- Customizable control parameters for tuning motor performance

## Getting Started

1. Clone or download the repository to your local machine.
2. Install the necessary Arduino libraries:
   - [SimpleFOC](https://github.com/askuric/Arduino-FOC)
   - [SimpleFOCCAN](https://github.com/simplefoc/Arduino-SimpleFOC-CAN) (Optional, for CAN communication support)
3. Open the main sketch in the Arduino IDE.
4. Configure the motor, driver, and sensor parameters according to your specific hardware.
5. Upload the sketch to your Arduino board.
6. Use the provided Serial or CAN commands to control and monitor the motor.

## Contributing

Contributions to the project are welcome. Please open an issue or submit a pull request if you would like to contribute to the development or suggest any improvements.

## License

This project is released under the MIT License. See the `LICENSE` file for more details.