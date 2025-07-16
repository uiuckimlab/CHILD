# TODO
### Checklist for cleaning up the code
- [ ] Change name 
  - [ ] env, follower -> follower
  - [ ] device, controller -> leader
  - [ ] teleoperator -> teleoperation
  - [ ] arm -> limb
### Features
  - [ ] IsaacSim or IsacLab
  - [ ] Better visualization of force feedback on leader side
### Robot Setup
- [x] Add G1
  - [x] Change all the arm-related words to `limb`
- [x] Open Manipulator Y
- [ ] Height from world - ground
- [ ] Merge Spot
  - [ ] Collision checking with body - considering non-teleop related body joints

### Device Setup
- [ ] VisionPro
- [ ] Joycon - make it more reliable
- [ ] RGBD Cam - make it more reliable
- [x] Open Manipulator Y

### Env Setup
- [x] ROS2
- [ ] Collision Filter in IsaacGym

### System-wise
- [x] Update Config file structure regarding ctrl_joint_names, arm_dof
  - [ ] documentation
  - [x] debug
- [ ] Constrained IK
- [ ] Add base class for everything - to provide some template

### Misc
- [ ] Make interactive demo for setting up configuration file
  - [ ] Print joint information
  - [ ] Setup initial pose / or read from srdf
- [ ] Just remove 'task' part
