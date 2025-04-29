# SEED Lab Group 7
Parker Anderson Caleb Brown (Controls)  
Drew Barner and Bruce Bearden (Computer Vision)  

Throughout this semester, our task is to design a PID controlled robot. To do this, we are utilizing a Raspberry Pi for camera detection of markers, and an Arduino for motor control and localization. The Arduino will interpret data from the camera, and drive to and from Aruco markers to complete a set course.

This goal was realized by completing the following checkpoints. For a more in-depth explanation, please view the README files listed in the individual folders.

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
# Demo 2
# Final
