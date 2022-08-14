# Readme 

### Objective
Maintain a variable (for this project, the distance between a ball and a proximity sensor on a tilt) about a predefined setpoint value via a PID (Proportional-Integral-Derivative) Controller ran on an Arduino Uno. 

#### Variables 
Input: distance from proximity sensor to ball on tilt
Output: degree angle of micro servo motor attached to tilt 
Setpoint: the desired distance between proximity sensor and ball on tilt 
Error == Setpoint - Input [=] cm

### Methods

To run this code, upload the contents of `tilt_pid_demo.ino` to your Arduino after creating the physical setup shown in this video: https://youtu.be/T3xJn-kEdWY. 

I used and tested this code on an Arduino Uno. 

Note that you will need to adjust the Kp, Ki, and Kd terms in the source code (`tilt_pid_demo.ino`) as needed to optimize performance (how well the ball is kept near setpoint distance without major fluctuations or offset). 

### Physical Dimensions of Demo 
Length of tilt: 26 cm
Width of tilt: 7 cm
Half side of tilt: 3.5 cm
Height of tilt off floor: 5 cm (the height of a wine cork cylinder)
Height of the micro servo support: 1 cm 
Distance from center pivot of tilt to pivot for servo connection: 2 cm (on same side of the micro servo 9g motor)
Length of stop to prevent ball from hitting proximity sensor: 3 cm (both sides)
