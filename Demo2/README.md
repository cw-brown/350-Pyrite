# EENG 350 Team 7 Pyrite
## Demo 2
The goal of demo 2 was to show a full, albeit limited, integration between computer vision and control. In order to send data between the two subsystems we use **I2C**, a board to board communication protocol. CV will send the distance and the angle. 
### State
CV currently sends 99.9 as an arbitrary number if they don't detect anything and -90/90 for a left/right turn respectively. The arduino code is set up to interpret these and change the state of certain flags. The results of this intepretation determine the state diagram transistions of the internal *finite state machine* for controls.
### Control Finite State Machine
At startup, the robot enters a seek mode, moving at a constant rotational speed until CV detects a marker. To make the code very easy to debug, the Pi will send infeasible data during the seek mode. When a marker is detected, the state will change, indicating a transition in the FSM. Then, the robot will enter its rotate mode, to get directly in line with the marker and then the move forward mode. Once the robot is within 1 feet of the target, it will enter either the stop state and then possibly the rotate state(depending on the hardcoded doTurn flag).
