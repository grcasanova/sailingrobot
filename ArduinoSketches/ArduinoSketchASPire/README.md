# Arduino ASPire Sketch #
This sketch contains code to run in the Arduino board located in the Actuator Unit (big box).
Its main responsibilities are interfacing the actuators with the CanBus messages coming from the Navigation Unit (the Raspberry Pi) and sending self-status and current sensors data over CanBus.
The code is organised so that messages from the bus are polled as frequently as possible to avoid internal code delays.

## External Libraries ##
This sketch uses the following third-party libraries

* [MaestroController](https://github.com/pololu/maestro-arduino) to interface with Polou servo motors and actuators control boards
* [SoftwareSerial](https://www.arduino.cc/en/Reference/SoftwareSerial) to allow several connection types (UART, SPI, I2C, ...) using IO pins
* CanBus and MsgParsing are two libraries created ad hoc for this project
