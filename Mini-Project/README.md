# CV Subsystem
### EENG 350 - Mini Project Pi Code.
### Group 7 - Pyrite
### Pi code created Bruce Bearden and Drew Barner
### Arduino code created by Parker Anderson and Caleb Brown

'''
This code detects the region where an aruco marker is detected, and
sends a command to the arduino to where the wheels should turn. Based on
which quadrant of the camera the marker is detected on the wheel will turn as
follows:

    Qudrant  -   Wheel Location [left wheel, right wheel]
    ----------------------------------------------------
      NE     -   [0,0]
      NW     -   [0,1]
      SE     -   [1,0]
      SW     -   [1,1]

To increase/decrease the lag adjustment of the frame rate is possible
The program runs optimally displaying every thrid frame, since the processing
power of the pi is not anything special.


The I2C communication protocol is used to send a string to the arduino to display the
desired location of the left wheel and right wheel, and to send a byte to the
arduino to commincate the desired location.

To run this code, ensure that the aruco marker used for testing is defined in the
imported aruco dictionary, and verify the following connections from the lcd pi shield
and the arduino:

    *NOTE: The LCD pi shield needs to properly attached where the left side of the shield
           is lined up with the SD card side of the PI. View the EENG 350 Assingment 1
           computer vision and communication tutorial for more details.

    Connections:
    pi        -        arduino
    --------------------------
    GND       -        GND
    SCL       -        A5
    SDA       -        A4

    
'''

# Controls Subsystem
Arduino-Based Two-Wheel Position Control System  

Purpose:
This program controls the position of a two-wheeled system using encoder feedback
and integral control. The system detects wheel positions based on OpenCV marker detection,
updates motor commands accordingly, and displays the target position on an LCD.

Hardware Setup:
- Two DC motors with rotary encoders for position feedback.
- Encoder connections:
    - Left Wheel Encoder: 
        - Channel A -> Pin 2 (interrupt pin)
        - Channel B -> Pin 5
    - Right Wheel Encoder:
        - Channel A -> Pin 3 (interrupt pin)
        - Channel B -> Pin 6
- Motor Control Pins:
    - Left Motor Direction -> Pin 7
    - Right Motor Direction -> Pin 8
    - Left Motor Pwm -> Pin 9
    - Right Motor Pwm -> Pin 10
- Communication:
    - Uses I2C (Wire.h) to receive position data from OpenCV.
    - Sends real-time position updates to a display (LCD).

How to Run:
1. It may be helpful to have the robot's wheels elavated from the ground and starting with the '0' tape markers facing up.
2. Upload this code to the Arduino board on the robot.
3. Ensure the OpenCV system is running and sending position data over I2C.
4. Observe the real-time wheel position, and target position on the LCD.
5. Adjust control parameters (kp, ki, etc.) in the code as needed for stability.

Control Strategy:
- Uses integral control to correct wheel position based on detected markers.
- Goal positions are continuously updated via OpenCV input.
- Encoder feedback ensures precise motor adjustments.

Authors: Parker Anderson, Caleb Brown
Date: 2/18/25
