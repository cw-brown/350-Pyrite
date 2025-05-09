# SEED Lab Group 7
Parker Anderson Caleb Brown (Controls)  
Drew Barner and Bruce Bearden (Computer Vision)  

Throughout this semester, our task is to design a PID controlled robot. To do this, we are utilizing a Raspberry Pi for camera detection of markers, and an Arduino for motor control and localization. The Arduino will interpret data from the camera, and drive to and from Aruco markers to complete a set course.

This goal was realized by completing the following checkpoints. For a more in-depth explanation, please view the README files listed in the individual folders.

# Mini Project
The overall goals achieved by this project:
* Use OpenCV to detect aruco markers and return their position as seen by the camera.
* Use control schemes to accurately move the wheels of the robot to a desired position.
* Communicate the goal position on an LCD screen connected to the rasberry pi.
* Implement threading to minimize any delay with the programs.
  
The Mini Project aims to integrate a two-wheel system with a computer vision interface to create a user-controlled positioning system. The primary objective is to use a camera to detect an Aruco marker’s position within an image and translate this into specific set-points for two wheels, each marked with tape labeled "0" and "1" at 180-degree intervals. The system operates on a Raspberry Pi for vision processing and display, with an Arduino controlling the motors. The project emphasizes modular task design, effective debugging, and robust control to ensure accurate wheel positioning despite external disturbances.

The computer vision subsystem, implemented on the Raspberry Pi, detects an aruco marker in the camera’s field of view and determines its quadrant (Northeast, Northwest, Southwest, or Southeast). Based on the quadrant, it assigns goal positions for the left and right wheels according to a predefined logic: NE (Left=0, Right=0), NW (Left=0, Right=1), SW (Left=1, Right=1), and SE (Left=1, Right=0). This subsystem must process images in real-time, display the camera feed with the marker’s position, and communicate the goal positions to the motor control subsystem. It leverages Python scripts, using OpenCV for marker detection, and employs threading to manage concurrent tasks like image processing and LCD updates without lag.

The motor control subsystem, implemented on Arduino, manages the two wheels to achieve the desired positions as determined by the computer vision subsystem. Each wheel’s position is controlled using a PI controller wrapped around an existing velocity controller, ensuring the wheels reach and maintain their goal positions ("0" or "1" facing up). The subsystem incorporates integral control to reject disturbances, such as manual wheel perturbations, by returning the wheels to their set-points. It calculates position errors, integrates them over time, and adjusts motor voltages to achieve the desired wheel angles, with all control logic implemented in Arduino sketches for precise, real-time execution.

The display and communication subsystem ties the project together by providing user feedback and ensuring seamless interaction between the Pi and Arduino. An LCD connected to the Pi displays the goal positions (e.g., "Goal Position: 01") based on the marker’s quadrant, updating without noticeable lag using threading to offload LCD updates to a separate thread. The subsystem also facilitates communication, with the Pi sending goal positions to the Arduino, which processes them to control the motors. Supplementary debugging scripts, Simulink models, and MATLAB analyses verify subsystem performance, such as step responses, ensuring the integrated system meets the project’s requirements for accuracy and robustness.
# Demo 1
The overall goals achieved by this project:
* Use OpenCV to detect aruco markers and return their angle from the Z axis of the camera.
* Use camera calibration to ensure accurate angle detection.
* Communicate the angle on an LCD screen connected to the rasberry pi while using threading to minimize any delay with the programs.
* Implement a PID controller to ensure the robot can move in a straight line and desired distance.
* Implement a PID controller to ensure the robot can accuratly turn a desired distance.
* Optimize program speed and accuracy
* Ensure accurate individual subsystem development
  
The Demo 1 project focuses on developing and demonstrating foundational capabilities for a robot that integrates computer vision and precise movement. This demo requires the robot to perform three critical tasks: detecting an ArUco marker and reporting its angle relative to the camera axis, moving forward a specified distance in a straight line, and rotating by a specified angle before moving forward a set distance. The project emphasizes teamwork, modular design, and documentation. 

The computer vision subsystem is tasked with detecting a 5 cm by 5 cm ArUco marker (marker 0 from the 6x6 dictionary) and calculating the signed angle between the camera’s z-axis and the marker’s position, with positive angles indicating the marker is to the left. During testing, the robot remains stationary while the marker is placed at five different locations, 1 to 5 feet away. The subsystem must detect the marker within 10 seconds, display the detected angle on an LCD screen, while also minimizing delay. Implemented in Python using OpenCV on a Raspberry Pi, this subsystem requires robust image processing to handle varying marker positions and ensure accurate angle reporting.

The straight-line movement test requires the robot to move forward a specified distance, ranging from 1 to 10 feet, with high precision. During three test runs, the robot must start within 10 seconds of code download, complete the movement within 60 seconds, and stop accurately at the desired distance. The subsystem, likely controlled by an Arduino managing motor outputs, measures the error between the desired and actual positions, aiming for an average error of 0.75 inches or less. This requires a well-tuned velocity control system and accurate distance tracking, possibly using wheel encoders or other sensors, to ensure the robot travels in a straight line without deviation.

The rotation and movement tests combine angular and linear motion, requiring the robot to rotate by a specified angle (possibly zero) and then move forward a specified distance in feet. Tested over three runs with the same timing constraints as the straight-line task, this subsystem must achieve precise rotation and translation, with an average position error of 0.75 inches or less. Controlled via Arduino, it builds on the straight-line movement system by adding a rotation mechanism, using differential wheel speeds or a dedicated steering system. The subsystem’s performance is supported by Simulink models for simulation and debugging, ensuring accurate execution of combined maneuvers.

# Demo 2
The overall goals achieved by this project:
* Use OpenCV to detect the color of the arrow next to the marker.
* Build a finite state machine, to control the state of the robot.
  
The Demo 2 project builds on prior work to advance a robot’s autonomous navigation capabilities. The demo requires the robot to locate a 2-inch ArUco marker (ID 0, 6x6 dictionary) placed 4 to 6 feet away, navigate to within 1.5 feet of it, and, in some trials, rotate 90 degrees in the direction indicated by an arrow next to the marker. The robot operates untethered, with performance judged on time to complete tasks and the number of failed trials.

The computer vision subsystem enables the robot to identify the ArUco marker and determine its position relative to the robot’s initial random orientation. The robot must rotate to locate the marker, which is placed approximately 5 feet away, and then calculate the necessary distance and angle. This subsystem processes camera input to detect the marker and estimate its distance and angle, providing data to guide navigation. 

The control subsystem is responsible for moving the robot to within 1.5 feet of the beacon’s center, defined as the robot’s center of rotation (mid-point between wheels), without colliding with the marker. In the first trial type, the robot rotates to face the beacon, moves forward, and stops precisely within the 1.5-foot radius. This requires accurate distance control and straight-line motion, building on prior demo capabilities. The subsystem must execute the maneuver within time constraints, with failures recorded if the robot does not start, stop correctly, or overshoots the target. Success is measured by the average time across two successful runs and the number of failed attempts.

In the second trial type for the control subsystem, the robot is required to reach the beacon and then turn 90 degrees in the direction indicated by the beacon’s arrow. After approaching within 1.5 feet, the robot must interpret the arrow’s direction and execute the correct rotation, avoiding incorrect turns or collisions with the beacon. This subsystem integrates precise angular control, ensuring the robot aligns accurately within the trial’s time limits. Performance is evaluated based on the average time for two successful runs and the number of failures.
# Final
The overall goals achieved by this project:
* Complete subsystem integration.
* Add the functionality of the stop marker.
* Reliably and quickly move the robot through the course.
  
The Final Demo represents the culmination of the robot development process, requiring full implementation of an autonomous navigation system. The robot must navigate a path of 5 to 8 ArUco markers (2-inch, ID 0, 6x6 dictionary) in a 7x10-foot area, starting 2 to 5 feet from the first marker with a random initial orientation. Each marker’s arrow indicates a 90-degree turn to locate the next marker, which is always within 5 feet, while other markers are beyond 5 feet to avoid confusion. The robot must reach within 1.5 feet of each marker’s center (measured at the robot’s center of rotation) without skipping any, with performance scored on path completion percentage, time per marker, number of markers approached, and restarts needed.

The computer vision subsystem is responsible for detecting and identifying the ArUco markers to guide the robot along the path. Starting with a counter-clockwise rotation to locate the first marker, the subsystem processes camera input to detect each 2-inch marker, determine its position and distance (ensuring the next marker is within 5 feet), and interpret the arrow’s direction for the subsequent 90-degree turn. This subsystem must reliably distinguish the correct target marker from others in the field of view, providing accurate positional data to the control subsystem for navigation. It operates in real-time to support continuous path following, ensuring the robot progresses without missing or misidentifying markers.

The control subsystem manages the robot’s movement and orientation to execute the navigation path. It rotates the robot to face each marker, moves it to within 1.5 feet of the marker’s center, and performs a 90-degree turn in the direction indicated by the arrow to align with the next marker. This subsystem integrates precise linear and angular control, using feedback from the computer vision subsystem to adjust trajectories and maintain accuracy. It must operate untethered, starting each maneuver promptly and completing the path efficiently, with performance metrics focused on minimizing time per marker and avoiding restarts. The subsystem is supported by well-documented code and Simulink models for simulation and debugging.

Both subsystems work in tandem to achieve seamless navigation, with the computer vision subsystem providing critical positional and directional data, and the control subsystem executing precise movements.
