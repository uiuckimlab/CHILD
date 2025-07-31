import os
import numpy as np
import time
import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber#, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from configs.default import BaseConfig
robot_config, device_config = BaseConfig().parse()
from tqdm import tqdm
from omegaconf import OmegaConf
from src.devices import DEVICES_DICT

kPi = 3.141592654
kPi_2 = 1.57079632
class Mode:
    PR = 0  # Series Control for Pitch/Roll Joints
    AB = 1  # Parallel Control for A/B Joints
USE_WAIST = False
class G1JointIndex:
    # Left leg
    LeftHipPitch = 0
    LeftHipRoll = 1
    LeftHipYaw = 2
    LeftKnee = 3
    LeftAnklePitch = 4
    LeftAnkleB = 4
    LeftAnkleRoll = 5
    LeftAnkleA = 5

    # Right leg
    RightHipPitch = 6
    RightHipRoll = 7
    RightHipYaw = 8
    RightKnee = 9
    RightAnklePitch = 10
    RightAnkleB = 10
    RightAnkleRoll = 11
    RightAnkleA = 11

    WaistYaw = 12
    WaistRoll = 13        # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistA = 13           # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistPitch = 14       # NOTE: INVALID for g1 23dof/29dof with waist locked
    WaistB = 14           # NOTE: INVALID for g1 23dof/29dof with waist locked

    # Left arm
    LeftShoulderPitch = 15
    LeftShoulderRoll = 16
    LeftShoulderYaw = 17
    LeftElbow = 18
    LeftWristRoll = 19
    LeftWristPitch = 20   # NOTE: INVALID for g1 23dof
    LeftWristYaw = 21     # NOTE: INVALID for g1 23dof

    # Right arm
    RightShoulderPitch = 22
    RightShoulderRoll = 23
    RightShoulderYaw = 24
    RightElbow = 25
    RightWristRoll = 26
    RightWristPitch = 27  # NOTE: INVALID for g1 23dof
    RightWristYaw = 28    # NOTE: INVALID for g1 23dof

    kNotUsedJoint = 29 # NOTE: Weight

child = DEVICES_DICT[device_config.type](robot_config, device_config)
class Runner():
    def __init__(self, device_config):
        self.child = child
        self.child.launch_init(np.zeros(14))
        self.shutdown = False

        self.time_ = 0.0
        self.control_dt_ = 0.01
        self.duration_ = 6.0
        self.counter_ = 0
        self.weight = 0.
        self.weight_rate = 0.2
        self.kp = 60.
        self.kd = 1.0
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False
        self.prev_target_qpos = None
        self.mode_machine_ = 0
        self.joints = [
          G1JointIndex.LeftHipPitch,G1JointIndex.LeftHipRoll,
          G1JointIndex.LeftHipYaw,G1JointIndex.LeftKnee,
          G1JointIndex.RightHipPitch,G1JointIndex.RightHipRoll,
          G1JointIndex.RightHipYaw,G1JointIndex.RightKnee,
          G1JointIndex.LeftShoulderPitch,  G1JointIndex.LeftShoulderRoll,
          G1JointIndex.LeftShoulderYaw,    G1JointIndex.LeftElbow,
          G1JointIndex.LeftWristRoll,      G1JointIndex.LeftWristPitch,
          G1JointIndex.LeftWristYaw,
          G1JointIndex.RightShoulderPitch, G1JointIndex.RightShoulderRoll,
          G1JointIndex.RightShoulderYaw,   G1JointIndex.RightElbow,
          G1JointIndex.RightWristRoll,     G1JointIndex.RightWristPitch,
          G1JointIndex.RightWristYaw,
          G1JointIndex.WaistYaw,
          G1JointIndex.WaistRoll,
          G1JointIndex.WaistPitch
        ]

        self.idx_mapping_dict = {
            'left_hip_pitch_joint': G1JointIndex.LeftHipPitch,
            'left_hip_roll_joint': G1JointIndex.LeftHipRoll,
            'left_hip_yaw_joint': G1JointIndex.LeftHipYaw,
            'left_knee_joint': G1JointIndex.LeftKnee,
            'left_ankle_pitch_joint': G1JointIndex.LeftAnklePitch,
            'left_ankle_roll_joint': G1JointIndex.LeftAnkleRoll,
            'right_hip_pitch_joint': G1JointIndex.RightHipPitch,
            'right_hip_roll_joint': G1JointIndex.RightHipRoll,
            'right_hip_yaw_joint': G1JointIndex.RightHipYaw,
            'right_knee_joint': G1JointIndex.RightKnee,
            'right_ankle_pitch_joint': G1JointIndex.RightAnklePitch,
            'right_ankle_roll_joint': G1JointIndex.RightAnkleRoll,
            'left_shoulder_pitch_joint':  G1JointIndex.LeftShoulderPitch,
            'left_shoulder_roll_joint': G1JointIndex.LeftShoulderRoll,
            'left_shoulder_yaw_joint': G1JointIndex.LeftShoulderYaw,
            'left_elbow_joint': G1JointIndex.LeftElbow,
            'left_wrist_roll_joint': G1JointIndex.LeftWristRoll,
            'left_wrist_pitch_joint': G1JointIndex.LeftWristPitch,
            'left_wrist_yaw_joint': G1JointIndex.LeftWristYaw,
            'right_shoulder_pitch_joint': G1JointIndex.RightShoulderPitch,
            'right_shoulder_roll_joint': G1JointIndex.RightShoulderRoll,
            'right_shoulder_yaw_joint': G1JointIndex.RightShoulderYaw,
            'right_elbow_joint': G1JointIndex.RightElbow,
            'right_wrist_roll_joint': G1JointIndex.RightWristRoll,
            'right_wrist_pitch_joint': G1JointIndex.RightWristPitch,
            'right_wrist_yaw_joint': G1JointIndex.RightWristYaw
            #waist_yaw_joint':G1JointIndex.WaistYaw
            # add for waist
        }

        self.waist_ind_array = [
            G1JointIndex.WaistPitch,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistYaw,
        ]

        self.child2g1_idx_mapping_array, self.g12child_idx_mapping_dict = [], {}
        id = 0
        for limb_name, limb_joint_names in device_config.limb_joint_names.items():
            for joint_name in limb_joint_names:
                joint_idx = self.idx_mapping_dict[joint_name]
                self.child2g1_idx_mapping_array.append(joint_idx)
                self.g12child_idx_mapping_dict[joint_idx] = id
                id += 1

        self.kp_dict = {joint_idx: self.kp for joint_idx in self.child2g1_idx_mapping_array}
        for waist_joint in self.waist_ind_array:
            self.kp_dict[waist_joint] = 200

        self.kd_dict = {joint_idx: self.kd for joint_idx in self.child2g1_idx_mapping_array}
        for waist_joint in self.waist_ind_array:
            self.kd_dict[waist_joint] = 5

        self.kp_dict[G1JointIndex.LeftWristPitch] = 10.0
        self.kp_dict[G1JointIndex.RightWristPitch] = 10.0
        self.kp_dict[G1JointIndex.LeftWristRoll] = 10.0
        self.kp_dict[G1JointIndex.RightWristRoll] = 10.0
        self.kp_dict[G1JointIndex.LeftWristYaw] = 5.0
        self.kp_dict[G1JointIndex.RightWristYaw] = 5.0

        self.kd_dict[G1JointIndex.LeftWristPitch] = 0.5
        self.kd_dict[G1JointIndex.RightWristPitch] = 0.5
        self.kd_dict[G1JointIndex.LeftWristRoll] = 0.5
        self.kd_dict[G1JointIndex.RightWristRoll] = 0.5
        self.kd_dict[G1JointIndex.LeftWristYaw] = 0.5
        self.kd_dict[G1JointIndex.RightWristYaw] = 0.5




        self.update_mode_machine_ = False


    def Init(self):

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)
        # create publisher #
        self.arm_sdk_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber #
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        while self.first_update_low_state == False:
            print("trying to get low state")
            time.sleep(1)
        print("Got low_state")


    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg
        self.first_update_low_state = True
        if self.update_mode_machine_ == False:
            self.mode_machine_ = self.low_state.mode_machine
            self.update_mode_machine_ = True
        
        self.counter_ +=1
        if (self.counter_ % 500 == 0) :
            self.counter_ = 0


    def run(self):

        ## Phase1: Initialize the limbs to zero pose
        zero_duration = 3.0
        time_ = 0
        pbar = tqdm(total=int(zero_duration/self.control_dt_), desc='1. Going to zero Pose')
        while time_ < zero_duration:
            start_time = time.time()
            time_ += self.control_dt_
            # self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0  # 1:Enable arm_sdk, 0:Disable arm_sdk
            ratio = np.clip(time_ / zero_duration, 0.0, 1.0)
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.mode_pr = Mode.PR

            for i, joint in enumerate(self.child2g1_idx_mapping_array):
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]

            for waist_id, joint in enumerate(self.waist_ind_array):
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q 
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            loop_time = time.time() - start_time
            left_time = max(self.control_dt_ - loop_time, 0.0)
            time.sleep(left_time)
            pbar.update(1)

        while not self.child.is_ready:
            print("Waiting for the child...", end='\r')
            if self.shutdown: return
            time.sleep(0.01)

        ## Phase2: Initialize the arm to initial pose
        self.command = initial_command = self.child.get_status()[0]
        # interpolate to initial command
        init_duration = 3.0
        target_qpos = np.concatenate([initial_command['left_leg'],initial_command['right_leg'],initial_command['left_arm'],initial_command['right_arm']])
        time_ = 0
        pbar = tqdm(total=int(init_duration/self.control_dt_), desc='2. Going to init Pose')
        while time_ < init_duration:
            start_time = time.time()
            time_ += self.control_dt_
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.mode_pr = Mode.PR
            # self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0  # 1:Enable arm_sdk, 0:Disable arm_sdk
            ratio = np.clip(time_ / init_duration, 0.0, 1.0)
            for i, joint in enumerate(self.child2g1_idx_mapping_array):
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * target_qpos[i]
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]

            for waist_id, joint in enumerate(self.waist_ind_array):
                v = self.command['waist'][waist_id] if USE_WAIST else 0.0
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q + ratio * v
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            loop_time = time.time() - start_time
            left_time = max(self.control_dt_ - loop_time, 0.0)
            time.sleep(left_time)
            pbar.update(1)

        print("Init pose done, starting teleoperation...")

        iteration = 0
        status_str = "Running... "
        bar_list = ["|", "|", "|", "|", "/", "/", "/", "/", "-", "-", "-", "-", "\\", "\\", "\\", "\\"]
        while not self.child.require_end:
            command = self.child.get_status()
            self.command = command[0]
            target_qpos = np.concatenate([command[0]['left_leg'], command[0]['right_leg'], command[0]['left_arm'], command[0]['right_arm']])
            if self.prev_target_qpos is not None:
                diff = target_qpos - self.prev_target_qpos
                if (abs(diff) > 0.3).any(): break

            start_time = time.time()
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0  # 1:Enable arm_sdk, 0:Disable arm_sdk
            self.low_cmd.mode_pr = Mode.PR
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.motor_cmd[i].mode =  1 # 1:Enable, 0:Disable
            for i, joint in enumerate(self.child2g1_idx_mapping_array):
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = target_qpos[i]
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]
            for waist_id, joint in  enumerate(self.waist_ind_array):
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = 0.0 if not USE_WAIST else self.command['waist'][waist_id]
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            loop_time = time.time() - start_time
            left_time = max(self.control_dt_ - loop_time, 0.0)
            time.sleep(left_time)
            iteration += 1
            print(status_str + bar_list[iteration%16], end="\r")
            self.prev_target_qpos = target_qpos

        print("End signal detected.. go back to zero pose")
        zero_duration = 3.0
        time_ = 0
        pbar = tqdm(total=int(zero_duration/self.control_dt_), desc='3. Going back to zero Pose')
        while time_ < zero_duration:
            start_time = time.time()
            time_ += self.control_dt_
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 0  # 1:Enable arm_sdk, 0:Disable arm_sdk
            self.low_cmd.mode_machine = self.mode_machine_
            self.low_cmd.mode_pr = Mode.PR
            ratio = np.clip(time_ / zero_duration, 0.0, 1.0)
            for i, joint in enumerate(self.child2g1_idx_mapping_array):
                self.low_cmd.motor_cmd[joint].mode = 1
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp
                self.low_cmd.motor_cmd[joint].kd = self.kd
            for waist_id, joint in  enumerate(self.waist_ind_array):
                self.low_cmd.motor_cmd[joint].tau = 0.
                self.low_cmd.motor_cmd[joint].q = (1.0 - ratio) * self.low_state.motor_state[joint].q
                self.low_cmd.motor_cmd[joint].dq = 0.
                self.low_cmd.motor_cmd[joint].kp = self.kp_dict[joint]
                self.low_cmd.motor_cmd[joint].kd = self.kd_dict[joint]

            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.arm_sdk_publisher.Write(self.low_cmd)
            loop_time = time.time() - start_time
            left_time = max(self.control_dt_ - loop_time, 0.0)
            time.sleep(left_time)
            pbar.update(1)

        sys.exit(-1)
        return

if __name__ == '__main__':
    ChannelFactoryInitialize(0, 'eth0')
    runner = Runner(device_config)
    runner.Init()
    runner.run()
