# CV Subsystem
Python-based system for detecting Aruco markers, estimating angles, and displaying real-time updates on an LCD.

### Purpose
This program uses OpenCV to detect Aruco markers, compute their angles relative to the camera, and send the data to an LCD display. It runs continuously, updating the display only when marker positions change.

### Hardware Setup
Camera: Used for capturing frames and detecting markers.  
LCD Display (Optional): Displays real-time marker angles.  
I2C Communication: Used for LCD updates.  

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

### Software Setup
Dependencies:  
- OpenCV (cv2)  
- NumPy  
- Adafruit Character LCD library  

Files:  
- cameraMatrix.pkl – Camera calibration data  
- dist.pkl – Distortion coefficients  

### How to Run
- Ensure the camera is connected and properly calibrated.
- Run the script:
    bash
    Copy
    Edit
    python detect_aruco.py
- Observe detected markers and angles on the LCD (or console if LCD is disabled).
- Press 'q' to exit.


# Controls Subsystem
Arduino-based two-wheel position control system using encoder feedback and PID control.

### Purpose
This program controls a two-wheeled robot, aligning it to a target position using encoders and a PID-based feedback loop. The robot receives navigation data via I2C from a Raspberry Pi and follows a finite state machine (FSM) for movement.

### Hardware Setup
- Motors & Encoders:
    - Left Encoder:
        - Channel A → Pin 2
        - Channel B → Pin 5
    - Right Encoder:
        - Channel A → Pin 3
        - Channel B → Pin 6
    - Left Motor:
        - Direction → Pin 7
        - PWM → Pin 9
    - Right Motor:
        - Direction → Pin 8
        - PWM → Pin 10

### How to Run
- Input desired phi and rho values
- Ensure the robot is positioned with wheels elevated for initial motor movements.
- Upload robot_control.ino to the Arduino.
