import os
import numpy as np
import time
import time
import sys

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
# from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
from configs.default import BaseConfig
robot_config, device_config = BaseConfig().parse()
from tqdm import tqdm
from omegaconf import OmegaConf
from src.devices import DEVICES_DICT
from src.devices.child import INDEX_LEFT_LEG, INDEX_RIGHT_LEG, MODE_LEG_ACTIVATED, MODE_LEG_DEACTIVATED
from threading import Lock, Thread

kPi = 3.141592654
kPi_2 = 1.57079632

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
USE_WAIST = True
TOTAL_JOINTS = 17 if USE_WAIST else 14
class Runner():
    def __init__(self, device_config):
        self.child = child
        self.child.launch_init(np.zeros(14)) # only arms
        self.shutdown = False

        self.time_ = 0.0
        self.control_dt_ = 0.02
        self.duration_ = 3.0
        self.counter_ = 0
        self.weight = 0.
        self.weight_rate = 0.2
        self.kp = 60.
        self.kd = 1.5
        self.dq = 0.
        self.tau_ff = 0.
        self.mode_machine_ = 0
        self.low_cmd = unitree_hg_msg_dds__LowCmd_()
        self.low_state = None
        self.first_update_low_state = False
        self.crc = CRC()
        self.done = False
        self.prev_target_qpos = None

        self.command_lock = Lock()
        self.command = {"right_leg":[0.0, 0.0, 0.0, 0.0], # pitch, roll, yaw, knee
                         'left_leg':[0.0, 0.0, 0.0, 0.0], 'waist':[0.0, 0.0, 0.0], # pitch, roll, yaw
        }
        self.mode_lock = Lock()
        self.right_leg_activated = False
        self.left_leg_activated = False
        self.transition_filter = np.ones([len(self.low_cmd.motor_cmd)])

        #### Setup loco client ####
        self.teleoperating = False
        self.teleop_ended = False
        self.sport_client = LocoClient()  
        self.sport_client.SetTimeout(0.04)
        self.sport_client.Init()
        self.sport_client.ZeroTorque()

        ### Setup Audio Client (Mainly for LED control) ###
        self.audio_client = AudioClient()
        self.audio_client.SetTimeout(0.01)
        self.audio_client.Init()
        self.audio_client.TtsMaker("Hello Child",1)
        self.audio_client.LedControl(0,133,0)

        self.vx = 0.0
        self.vy = 0.0
        self.w = 0.0
        self.vel_cmd_duration = 0.4 ##not sure about this
        self.vel_lim = 0.8
        self.omega_lim = 0.8

        self.idx_mapping_dict = {
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
            'right_wrist_yaw_joint': G1JointIndex.RightWristYaw,
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
        }
        self.leg_idx_mapping_dict = {}

        self.left_arm_names = ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
                               'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint']
        self.left_arm_ind_array = [self.idx_mapping_dict[joint_name] for joint_name in self.left_arm_names]
        self.right_arm_names = ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
                                'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']
        self.right_arm_ind_array = [self.idx_mapping_dict[joint_name] for joint_name in self.right_arm_names]

        self.arm_ind_array = self.left_arm_ind_array + self.right_arm_ind_array
        self.waist_ind_array = [
            G1JointIndex.WaistPitch,
            G1JointIndex.WaistRoll,
            G1JointIndex.WaistYaw,

        ]
        if TOTAL_JOINTS == 14:
            self.total_control_joint_array = self.arm_ind_array 
        elif TOTAL_JOINTS == 17:
            self.total_control_joint_array = self.arm_ind_array + self.waist_ind_array
        else:
            raise ValueError(f"Invalid Options, please check again the setting, TOTAL_JOINTS = {TOTAL_JOINTS}, USE_WAIST = {USE_WAIST}")

        print(f"Controlling total {len(self.total_control_joint_array)} joints : {self.total_control_joint_array}")


        self.child2g1_idx_mapping_array, self.g12child_idx_mapping_dict = [], {}
        id = 0
        for limb_name, limb_joint_names in device_config.limb_joint_names.items():
            for joint_name in limb_joint_names:
                joint_idx = self.idx_mapping_dict[joint_name]
                if joint_idx not in self.total_control_joint_array: 
                    continue # skip the joints that are not controlled - like legs
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

        self.led_mode = 'preparing'
        self.setledThread = Thread(target=self.set_led_thread)
        self.setledThread.start()

    def Init(self):
        # create publisher #
        self.arm_sdk_publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        self.arm_sdk_publisher.Init()

        # create subscriber #
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateHandler, 10)

        while self.first_update_low_state == False:
            time.sleep(1)
        print("Got low_state")

        self.velCmdWriteThread = Thread(target=self.velCmdWrite)
        self.velCmdWriteThread.start()

        self.armCmdWriteThread = Thread(target=self.armCmdWrite)
        self.armCmdWriteThread.start()

    def set_led_thread(self):

        mode_dict = {
            'only_arm': (0,0,255), # blue
            'left_leg': (255,0,0), # red
            'right_leg': (0,255,0), # green
            'both_leg': (255,34,255), # purple
            'preparing': (251,133,0) # yellow orangish
        }

        while not self.teleop_ended:
            if not self.teleoperating or self.teleop_ended:
                self.led_mode = 'preparing'
            elif self.left_leg_activated and self.right_leg_activated:
                self.led_mode = 'both_leg'
            elif self.left_leg_activated:
                self.led_mode = 'left_leg'
            elif self.right_leg_activated:
                self.led_mode = 'right_leg'
            else:
                self.led_mode = 'only_arm'

            r,g,b = mode_dict[self.led_mode]
            self.audio_client.LedControl(r,g,b)
            time.sleep(0.1)
        #print("LED CONTROL", r,g,b, "mode:", self.led_mode)
        return

    def velCmdWrite(self):
        while not self.teleop_ended:
            loop_start = time.time()
            if self.teleoperating and not self.child.require_end:

                if not self.left_leg_activated and self.child.leg_mode[INDEX_LEFT_LEG] == MODE_LEG_ACTIVATED:
                    with self.mode_lock:
                        self.left_leg_activated = True
                        self.last_left_arm_qpos = self.command['left_arm']
                    self.child.send_request(self.left_arm_names, self.last_left_arm_qpos, joint_kps=0.07)

                
                elif self.left_leg_activated and self.child.leg_mode[INDEX_LEFT_LEG] == MODE_LEG_DEACTIVATED:
                    with self.mode_lock:
                        self.left_leg_activated = False
                        self.transition_filter[self.left_arm_ind_array] = 0
                    zero_popse = np.zeros([len(self.left_arm_names)])
                    zero_popse[self.left_arm_names.index("left_elbow_joint")] = 0.8
                    self.child.send_request(self.left_arm_names, zero_popse, joint_kps=[0.02,0.02,0.02,0.02,0.00,0.00,0.00])
                    
                if not self.right_leg_activated and self.child.leg_mode[INDEX_RIGHT_LEG] == MODE_LEG_ACTIVATED:
                    with self.mode_lock:
                        self.right_leg_activated = True
                        self.last_right_arm_qpos = self.command['right_arm']
                    self.child.send_request(self.right_arm_names, self.last_right_arm_qpos, joint_kps=0.07)

                elif self.right_leg_activated and self.child.leg_mode[INDEX_RIGHT_LEG] == MODE_LEG_DEACTIVATED:
                    with self.mode_lock:
                        self.right_leg_activated = False
                        self.transition_filter[self.right_arm_ind_array] = 0
                    zero_popse = np.zeros([len(self.right_arm_names)])
                    zero_popse[self.right_arm_names.index("right_elbow_joint")] = 0.8
                    self.child.send_request(self.right_arm_names, zero_popse, joint_kps=[0.02,0.02,0.02,0.02,0.00,0.00,0.00])
                        
                leg_qpos = np.zeros(3)
                if self.left_leg_activated:
                    leg_qpos += self.command['left_leg'][:3]
                if self.right_leg_activated:
                    leg_qpos += self.command['right_leg'][:3]

                self.vx = float(np.clip(-leg_qpos[0]/1.0, -self.vel_lim, self.vel_lim))
                self.vy = float(np.clip(leg_qpos[1]/5.0, -self.vel_lim, self.vel_lim))
                self.w  = float(np.clip(leg_qpos[2]/1.0, -self.omega_lim, self.omega_lim))
                if (abs(self.vx) < 0.2):
                    self.vx = 0.0
                else:
                    self.vx -= 0.2 * np.sign(self.vx)

                if (abs(self.vy) < 0.01):
                    self.vy = 0.0
                else:
                    self.vy -= 0.01 * np.sign(self.vy)
                if (abs(self.w) < 0.01):
                    self.w = 0.0
                else:
                    self.w -= 0.01 * np.sign(self.w)
                    
                ### send velocity self.command to lower body controller
                self.sport_client.SetVelocity(self.vx, self.vy, self.w, self.vel_cmd_duration)

            else:
                #pass
                self.sport_client.StopMove() # command zero velocity

            loop_time = time.time() - loop_start
            left_time = max(self.control_dt_ - loop_time, 0.0)
            time.sleep(left_time)
        print("Shutting down leg teleop...")
        return

    def armCmdWrite(self):
        while not self.teleop_ended:
            loop_start = time.time()
            if self.teleoperating and not self.child.require_end:
                with self.command_lock:
                    self.command = self.child.get_status()[0]
        
                if self.left_leg_activated:
                    left_arm_qpos = self.last_left_arm_qpos # actually we don't need to update this since transition_filter is 0
                else:
                    self.transition_filter[self.left_arm_ind_array] += 0.01
                    left_arm_qpos = self.command['left_arm']

                if self.right_leg_activated:
                    right_arm_qpos = self.last_right_arm_qpos
                else:
                    self.transition_filter[self.right_arm_ind_array] += 0.01
                    right_arm_qpos = self.command['right_arm']

                self.transition_filter = np.clip(self.transition_filter, 0.0, 1.0)
                target_qpos = np.concatenate([left_arm_qpos, right_arm_qpos])
                if self.prev_target_qpos is not None:
                    diff = target_qpos - self.prev_target_qpos
                    if (abs(diff) > 0.3).any(): break

                self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1  # 1:Enable arm_sdk, 0:Disable arm_sdk
                for i, joint in enumerate(self.child2g1_idx_mapping_array):
                    self.low_cmd.motor_cmd[joint].tau = 0.
                    self.low_cmd.motor_cmd[joint].q = (1.0 - self.transition_filter[joint]) * self.low_state.motor_state[joint].q + self.transition_filter[joint] * target_qpos[i]
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

                self.prev_target_qpos = target_qpos
            loop_time = time.time() - loop_start
            left_time = max(self.control_dt_ - loop_time, 0.0)
            time.sleep(left_time)

        print("Shutting down arm teleop...")
        return 


    def LowStateHandler(self, msg: LowState_):
        self.low_state = msg

        if self.first_update_low_state == False:
            self.first_update_low_state = True


    def interpolate_to_this_pose(self, target_qpos, duration=3.0, desc='Going to Pose'):
        time_ = 0
        pbar = tqdm(total=int(duration/self.control_dt_), desc=desc)
        while time_ < duration:
            start_time = time.time()
            time_ += self.control_dt_
            self.low_cmd.motor_cmd[G1JointIndex.kNotUsedJoint].q = 1  # 1:Enable arm_sdk, 0:Disable arm_sdk
            ratio = np.clip(time_ / duration, 0.0, 1.0)
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


    def run(self):

        ## Phase1: Initialize the arm to zero pose
        zero_pose = np.zeros(len(self.total_control_joint_array))
        zero_pose[self.g12child_idx_mapping_dict[G1JointIndex.LeftElbow]] = 0.8
        zero_pose[self.g12child_idx_mapping_dict[G1JointIndex.RightElbow]] = 0.8

        self.interpolate_to_this_pose(zero_pose, duration=1.0, desc='Going to zero Pose')
        print("Zero Pose Done, Going to intialize pose")

        self.last_left_arm_qpos = zero_pose[:7]
        self.last_right_arm_qpos = zero_pose[7:]

        ## Wait for the child to be ready 
        while not self.child.is_ready:
            print("Waiting for the child...", end='\r')
            time.sleep(0.01)

        ## Phase2: Initialize the arm to initial pose
        self.command = initial_command = self.child.get_status()[0]
        # interpolate to initial command
        target_qpos = np.concatenate([initial_command['left_arm'],initial_command['right_arm']])
        self.interpolate_to_this_pose(target_qpos, duration=1.5, desc='Going to initial Pose')

        self.last_left_arm_qpos = initial_command['left_arm']
        self.last_right_arm_qpos = initial_command['right_arm']

        self.teleoperating = True

        status_str = "Running Teleop... "
        bar_list = ["|", "|", "|", "|", "/", "/", "/", "/", "-", "-", "-", "-", "\\", "\\", "\\", "\\"]
        iteration = 0
        while self.child.require_end == False:
            iteration += 1
            #print(f"{status_str} {bar_list[iteration % len(bar_list)]} : LL {self.left_leg_activated}, RL {self.right_leg_activated} vx {self.vx:.2f} vy {self.vy:.2f} vz {self.w:.2f}  ", end='\r')
            print(f"{status_str} {bar_list[iteration % len(bar_list)]} : LL {self.left_leg_activated}, RL {self.right_leg_activated} vx {self.vx:.2f} vy {self.vy:.2f} w {self.w:.2f} p {self.command['waist'][0]:.2f} r {self.command['waist'][1]:.2f} y {self.command['waist'][2]:.2f}" , end='\r')
            time.sleep(0.01)

        self.teleoperating = False
        self.teleop_ended = True
        print("End signal detected.. waiting for other threads to finish")
        self.velCmdWriteThread.join()
        self.armCmdWriteThread.join()
        self.setledThread.join()

        print("Going back to zero pose")
        zero_duration = 3.0
        self.command['waist'] = [0.0, 0.0, 0.0]
        self.interpolate_to_this_pose(zero_pose, duration=zero_duration, desc='Going back to zero pose')

        sys.exit(-1)
        return

if __name__ == '__main__':
    ChannelFactoryInitialize(0, 'eth0')
    runner = Runner(device_config)
    runner.Init()
    runner.run()
