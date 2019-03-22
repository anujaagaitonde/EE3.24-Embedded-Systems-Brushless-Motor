## Description

In this coursework, a control algorithm is implemented to control the rotation of a synchronous DC brushless motor, via serial input commands from the user. The allowed controls are:

- spinning velocity
- number of revolutions
- spinning orientation
 
 Additionnally, this reatime operating system (RTOS) is simultaneously running a bitcoin mining algorithm in the background of the motor control operations.
 
 ## Usage (Mac)
 
 - Connect the board via USB
 - Open terminal
 - Enter `ls /dev/tty.usb*`
 - Identify the right port
 - Enter `screen /dev/tty.usb...` with the corresponding port
