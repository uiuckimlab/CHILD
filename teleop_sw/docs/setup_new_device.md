# How to setup a new device
---
## Gello 
- write a config file
  - file name convention: `gello_<robot_name>.yaml`
  - it is good to add ik_cfg in the config file, in case we use it for non-direct joint mapping.
#### Direct Mapping
- Direct joint mapping is available only when the gello device has same structure as the robot.
  - e.g. `device/gello_papras_7dof_2arm_table` <-> `robot/papras_7dof_2arm_table`
  - You can set which robot is possible for direct mapping in the config file.
    ``
    direct_mapping_available_robots: ['papras_7dof_2arm_table']
    ``
- Otherwise, we need to use inverse kinematics to map the gello device to the robot.
  - e.g. `device/gello_ur5_2arm_table` <-> `robot/papras_7dof_2arm_table`  
- We have two methods to do this: (you can set this option 'joint_mapping_method' in the config file)
  1. `ik_leader`: Calculate delta eef_pose, and use inverse kinematics of `gello_<target_robot>`, then forward the joint angles to the robot.
  2. `ik_follower`: Calculate delta eef_pose, and use inverse kinematics of `<target_robot>`, then forward the joint angles to the robot.
- For the `ik_leader` method, we need to set the `ik_cfg` in the `gello_<target_robot>.yaml` file, not in the `gello_<current_device>.yaml` file. The code will automatically find the `ik_cfg` in the `gello_<target_robot>.yaml` file.

#### Reset Pose
- To properly use the gello device, we need to set the reset pose of the device - this pose is used when you want to end the teleoperation session, and reset the robot pose
- You may use `scripts/gello_figure_out_pose.py` to figure out the pose of the device.
  - this code assumes the device runs with ROS, so you need to run the ROS node for the device.
  - first write down the configuration file, and run the code with the configuration file.
  ```
    python -m scripts.gello_figure_out_pose -c configs/device/gello_papras_7dof_2arm_table.yaml
  ```
  - copy and paste the output to the configuration file.
    ```
    reset_pose: [[0.15, 0.3, 0.2], [0.15, -0.3, 0.2]] # eef pose regarding world frame
    reset_cube: [[0.2, 0.1, 0.05], [0.2, 0.1, 0.05]] # half cube size of reset region
    ```
---
## New Type
- Write a new class in `src/devices/<device_name>.py`. You can check the template in `src/devices/base.py`.
- add the new class to DEVICES_DICT in `src/devices/__init__.py`
