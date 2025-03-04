# Arduino Motor Controller

This code turns an Arduino into a motor controller!

It provides a serial interface to communicate with ros2_control framework and generates the appropriate PWM signals for two L298N motor drivers to drive four motors of the mecanum drive; although the code was [originally](https://github.com/hbrobotics/ros_arduino_bridge/tree/a960c8a88a6255d0104c92838045c06257c509d0/ros_arduino_firmware/src/libraries/ROSArduinoBridge) developed for a diff drive.

This code should be compiled in Arduino IDE and uploaded to Arduino. Before compilation [PinChangeInterrupt.h](https://github.com/NicoHood/PinChangeInterrupt) and [ShiftRegister74HC595.h](https://blog.timodenk.com/shift-register-arduino-library/) libraries should be installed in Arduino IDE.

To test the motor controller install python3-serial:
```
$ sudo apt install python3-serial
```
Then run:
```
$ pyserial-miniterm -e /dev/arduino 115200
```
Where /dev/arduino is the udev alias set in /etc/udev/rules.d on Ubuntu Noble 24.04 and associated with the Arduino device, usually /dev/typeUSB0 if no other USB devices are connected, and 115200 is the baud rate set in ROS2Arduino.ino file. For Arduino UNO the maximun baud rate is 115200. To send commands listed in commands.h file to Arduino type them in the terminal to receive a response from the device, for example:

```
$ pyserial-miniterm -e /dev/arduino 115200
--- Miniterm on /dev/arduino  115200,8,N,1 ---
--- Quit: Ctrl+] | Menu: Ctrl+T | Help: Ctrl+T followed by Ctrl+H ---
e
0 0 0 0
o 100 -100 100 -100
OK
v
Kp:20;Kd:12;Ki:0;Ko:50

--- exit ---
```
This motor controller was developed specifically for [robo_cart](https://github.com/vamichrom/robo_cart) project.