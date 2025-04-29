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
# Final
