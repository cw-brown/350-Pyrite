# EENG 350 Team 7 Pyrite
## Demo 2
The goal of demo 2 was to show a full integration between computer vision and control. In order to send data between the two subsystems using **I2C**, a board to board communication protocol. CV will send three buckets of data: the state, the distance, and the angle. 
### State
CV will send a variable related to the state that they detect. This determines the state diagram transistions of the internal *finite state machine* of controls. State encodes things like turn right, move forward, or seek.
### Control Finite State Machine
At startup, the robot enters a seek mode, moving at a constant rotational speed until CV detects a marker. To make the code very easy to debug, the Pi will send infeasible data during the seek mode. When a marker is detected, the state will change, indicating a transition in the FSM. Then, the robot will enter its rotate and move forward mode. Once the robot is within 1.5 feet of the target, it will enter either the rotate state (depending on current configuration), or the stop state.

