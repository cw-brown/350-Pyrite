# Final Project - Autonomous Robot Navigation

## Overview
This project enables a two-wheeled robot to autonomously detect an Aruco marker beacon, approach it within 1.5 feet, and optionally execute a directional turn based on a detected arrow color. The robot uses a Raspberry Pi for computer vision and high-level control, and an Arduino for low-level motor and encoder management.

Performance was evaluated based on the robot’s **accuracy**, **speed**, and **reliability** across multiple runs.

---

## System Components

### Raspberry Pi (Python)
- Detects the nearest Aruco marker and measures its distance and angle relative to the robot.
- Identifies the direction of an arrow indicator based on color (red for left, blue for right).
- Sends target navigation commands to the Arduino via I2C communication.
- Supports two operational modes:
  1. **Approach Only** — Drive to within 1.5 feet of the beacon and stop.
  2. **Approach and Turn** — Drive to the beacon, then execute a 90° turn based on arrow direction.

> **Note:** Operational mode is controlled by editing the `DO_TURN` variable in the Arduino code.

### Arduino (C++)
- Receives navigation commands (distance and angle) from the Raspberry Pi.
- Controls motors using a finite state machine (FSM) and PID controllers.
- Uses encoder feedback to track wheel positions and update motor outputs.
- Communicates with the Raspberry Pi over I2C to receive navigation goals.

---

## Communication
- **Protocol:** I2C
- **Data Sent:**
  - Floating-point distance and angle information (as bytes)
  - Command flag indicating if a turn should occur after reaching the marker

---

## Running the System
1. Ensure the Raspberry Pi’s LCD shield is correctly aligned (left side matching the Pi’s SD card side).
2. Verify that the Aruco marker used is included in the selected dictionary in the Python script.
3. Upload the Arduino code to the microcontroller.
4. Run the Python script on the Raspberry Pi.
5. Place the beacon marker and start the robot from an initial position facing roughly toward the marker.

---

## Notes
- Only the nearest visible marker is used for navigation decisions.
- System requires manual setup of operational mode (`DO_TURN`) before deployment.
- For detailed hardware wiring, refer to internal project documentation or hardware schematics.

---

## Authors
- [Your Name]

## Date
- April 2025
