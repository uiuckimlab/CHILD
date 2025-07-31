## Installation
### Inside CHILD
```
mkdir -p ws_child/src
cd ws_child/src
git clone https://github.com/uiuckimlab/CHILD.git
git clone https://github.com/uiuckimlab/PAPRAS-V0-Public.git -b ros2_humble
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
cd ../
colcon build --symlink-install
```

### Inside G1
- Setup ROS env
```
mkdir -p ws_child/src
cd ws_child/src
git clone https://github.com/uiuckimlab/CHILD.git
rm -r CHILD/hw_interface/teleop_leaders # for simple build
cd ../
colcon build --symlink-install
```
- Setup python virtualenv
```
cd ws_child/src/CHILD
virtualenv venv_child -p python3.8
source venv_child/bin/activate
cd teleop_sw
pip install -r requirements.txt
```
If pip install shows "ERROR: could not find a version that satisfies the requirement libxml2" run:
```
pip install lxml --only-binary :all:
pip install -r requirements.txt
```
Install unitree_sdk2_python from source
```
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip3 install -e .
```
Update the IP addresses on lines 11 and 12 of the cycloneds.xml file to match G1 and CHILD
```
<Peer address="xx.x.xx.xxx"/>
<Peer address="xx.x.xx.xxx"/>
```



# Robot Setup

![img](https://oss-global-cdn.unitree.com/static/98431a05f8e747709722e901d32d8ce3_11798x7046.jpg)


Also this is an official doc for booting up and making G1 to stand:

https://support.unitree.com/home/en/G1_developer/quick_start

### Standing & Walking Mode

0. After booting up the robot, the robot will be in `Zero Torque` mode.
1. Press `L2+B` then the robot will be in `Damping` mode in `Normal` mode. At this step, if you raise the robot arm, you can feel the torque from the shoulder joint.
2. Press `L2+Up`, the robot will be in `Get Ready`, and arms will go to specific pose. Now you can put g1 down to the ground.
3. Press `R1+Y`  it goes to `Main Operation Control` mode, it will starts to try balancing
4. If you press `R2+A` instead of `R1+Y`, it will goes to `Runninng Mode`. The robot will walk a lot faster. You can increase/decrease the velocity using `R2+Down` or `R2+Up`.

- **E-STOP**:  Hold `L2+B` for 5 seconds

### Debug Mode

0. After booting up the robot, the robot will be in `Zero Torque` mode.
1. Press `L2+R2` then the robot will be in `Damping` mode in `Debug` mode. At this step, if you raise the robot arm, you can feel the torque from the shoulder joint.
2. Press `L2+A`, the robot will be in `Diagnostic Action`, and arms will go to specific pose. 
3. G1 is ready for teleoperation!





## Teleop Setup

- If you want both walking&teleop, make G1 to stand on the ground before running teleop codes.

### Inside CHILD

```
export CHILD_WIFI_ADDRESS=XX.X.XX.XXX
ssh papras@$CHILD_WIFI_ADDRESS
```
- launch ros2 docker 

    - `docker ps -a` : To see the list of existing container

    - If there is a container with name `ros_humble`

        - ```
            docker start ros_humble && docker exec -it ros_humble bash
            ```

    - If there is not, 

      ```shell
        docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb --env=DISPLAY --volume="/etc/group:/etc/group:ro" --volume="/etc/passwd:/etc/passwd:ro" --volume="/etc/shadow:/etc/shadow:ro" --volume="/etc/sudoers.d:/etc/sudoers.d:ro" --net host -v /home:/home -v ~/Docker/hosts:/etc/hosts -v ~/shared:/home/usr/ papras-ros-humble
      ```

- Setup ROS env

  ```shell
  cd /home/papras/shared/ws_child/ && source install/setup.bash
  export ROS_DOMAIN_ID=55
  ros2 launch teleop_leaders leader_hw_g1_all_limbs.launch.py 
  ```


### Inside G1

- Make sure G1 is connected to the same wifi as CHILD.

- - If g1 is not connected to wifi, wire-connect to g1, and 

    ```
    ssh unitree@192.168.123.164
    sudo nmcli connection up <wifi-name>
    ```
```
export G1_WIFI_ADDRESS=10.1.10.189 # Please replace this IP address with your G1's IP address
ssh -X unitree@$G1_WIFI_ADDRESS
```
- for the teleop demo with baby, enter followings (If using a different ROS domain ID, must modify line 2 of cyclonedds.xml before exporting)
```
export CYCLONEDDS_URI=file:///home/unitree/ws_child/src/CHILD/teleop_sw/cyclonedds.xml
export ROS_DOMAIN_ID=55
cd ws_child
source install/setup.bash
cd src/CHILD
source venv_child/bin/activate
cd teleop_sw
python -m run_g1_upper_body # for upper body control - need to be in standing mode
python -m run_g1_full_body_teleop # for whole body control - need to be in debug mode
```

- It will have multiple stages to start teleoperation
    0. G1 will go to the zero pose
    1. Pressing both gripper 3 secs, it will go to the initial pose from the baby
    2. Then teleoperation starts.
    3. If the operator press one of the gripper 2 secs, the leg on the same side will be activated.
       - The leg can be used as joystick for walking
       - If both of the legs are activated, the speed and orientation are the sum of the values from each leg.
       - if the operator press the gripper 2secs while activated, the leg will go back to deactivated mode -> zero position.
    4. If the operator press both of the grippers 3 secs, the teleoperation will be ended.
