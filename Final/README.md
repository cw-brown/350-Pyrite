# Final Project - Autonomous Robot Navigation

## Overview
This project enables a two-wheeled robot to autonomously detect an Aruco marker beacon, approach it within 1.5 feet, and execute a directional turn based on a detected arrow color, ultimately completing a set course/maze. The robot uses a Raspberry Pi for computer vision and an Arduino for motor and encoder management.

Performance was evaluated based on the robot’s **accuracy**, **speed**, and **reliability** across multiple runs.

---

## System Components

### Raspberry Pi (Python)
- Detects the nearest Aruco marker and measures its distance and angle relative to the robot.
- Identifies the direction of an arrow indicator based on color (green for left, red for right).
- Sends target navigation commands to the Arduino via I2C communication.

### Arduino (C++)
- Receives navigation commands (distance and angle) from the Raspberry Pi.
- Controls motors using a finite state machine (FSM) and PID controllers.
- Uses encoder feedback to track wheel positions and update motor outputs.
- Communicates with the Raspberry Pi over I2C to receive navigation goals.

---

## Communication
- **Protocol:** I2C
- **Data Sent:**
  - Floating-point distance (rho) and angle (phi) information (as bytes)
  - phi = +/- 90 for left or right turn when at marker
  - phi = -50 for hard stop
  - rho = 99.9, phi = 99.9 for no marker detected

---

## Running the System
1. Verify that the Aruco marker used is included in the selected dictionary in the Python script.
2. Upload the Arduino code to the microcontroller.
3. Run the Python script on the Raspberry Pi.
4. Place the course markers and power on the robot from an initial position/rotation where no visible markers other than the first marker will be closer than 5 ft while seeking.

---

## Notes
- Only the nearest visible marker is used for navigation decisions.
- For detailed hardware wiring, refer to internal project documentation or hardware schematics.

---

## Authors
- Parker Anderson
- Bruce Bearden
- Drew Barner
- Caleb Brown

## Date
- 4/28/25
