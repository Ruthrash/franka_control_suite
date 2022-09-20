# franka_control_suite
**This repo is a work in progress **

Contains lowlevel torque controllers for Franka Emika panda and an interprocess communication interface using ZeroMQ. This code base's intended functionality is similar to [franka-interface](https://github.com/iamlab-cmu/franka-interface) but without any dependancy in ROS.


# Dependencies 

- libfranka
- ZeroMQ
- Eigen




To do:
- [ ] add python example test scripts 
- [ ] add interpolators to all implementations

Notes: 
- IK controller has been tested in the real world 
- other controllers are being tested currently
