# POE-lab3

Code and results for Lab 3 of POE at Olin College, Fall 2016

[report.pdf](Report)

# Current Todos :

Jamie :

- [ ] Redesign IR Mounts
- [x] Simulator with Higher Fidelity HW Interface
- [x] Characterize Motors, Velocity-Voltage
- [ ] Autocalibration
- [ ] On-Off Switch (might as well be E-STOP)
- [ ] Voltage Monitor (differentiate USB vs. Outlet power?)

Eric :

- [x] Build data structures for storing path data
- [x] Integrate motor characterization into kinematics model
- [x] Record line-followed path
- [x] Play back recorded path losslessly
- [x] Hybridize path following and line following
- [x] Properly handle left-biased and right-biased following
- [ ] Make framework for segmented paths (using turns as odometry resets)
- [x] Build state machine in code for handling training/replaying


# IR Reflectance Sensor Wiring Diagram

![IRWiring](images/ir_reader_wiring.jpg)

# Connecting to Arduino

![IRReader](images/ir_reader.png)

NOTE: motor wires should be connected such that, when viewed from the front, the wires cross and red is left
