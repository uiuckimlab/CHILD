import copy
import sys
import rclpy
from rclpy.node import Node

import pinocchio as pin
from sensor_msgs.msg import JointState
import numpy as np

from pytransform3d import transformations as pt
from omegaconf import OmegaConf
import threading
from threading import Lock
from configs.utils import add_info_robot_config
import logging
import time
from teleop_msgs.srv import SetBasePose
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R

INDEX_LEFT_LEG = 0
INDEX_RIGHT_LEG = 1
MODE_LEG_DEACTIVATED = 0
MODE_LEG_ACTIVATED = 1


class Child:
    def __init__(self, robot_config, control_config, logger=None):
        self.logger = logger if logger is not None else logging.getLogger(__name__)
        self.robot_config = robot_config
        self.num_limbs = robot_config.robot_cfg.num_limbs
        self.last_command = None
        self.waist_euler = None

        rclpy.init()
        self.node = Node("Child")
        self.set_base_pose_pub = self.node.create_publisher(JointState, '/leaders/set_base_pos', 10)

        self.control_config = control_config
        self.leader_states = None
        self.leader_joint_names = None
        self.leader_limb_names = list(control_config.limb_joint_names.keys())
        self.follower_limb_names = list(robot_config.robot_cfg.limb_joint_names.keys())
        self.direct_joint_mapping = robot_config.robot_cfg.name in self.control_config.direct_mapping_available_robots
        assert self.direct_joint_mapping == True
        self.controller_info_for_vis = {'color': 'blue', 'log': 'Child Leaders Controller'}
        self.is_ready, self.require_end = False, False
        self.shutdown = False

        self.gripper_th = 0.7
        self.last_pitch, self.last_roll, self.last_yaw = None, None, None


        # load the leader model
        self.model, self.collision_model, self.visual_model = pin.buildModelsFromUrdf(self.control_config.asset_cfg.urdf_path,
                                                                                      package_dirs=[self.control_config.asset_cfg.asset_dir])
        self.data = self.model.createData()
        self.pin_model_joint_names = [name for name in self.model.names]
        if 'universe' in self.pin_model_joint_names:
            self.pin_model_joint_names.remove('universe')
        frame_mapping = {}
        for i, frame in enumerate(self.model.frames):
            frame_mapping[frame.name] = i
        self.end_effector_frame_ids = [frame_mapping[name] for limb_name, name in self.control_config.end_effector_link.items()]
        self.motion_scale = self.control_config.motion_scale
        self.neutral_pos = getattr(self.control_config, 'neutral_pos', pin.neutral(self.model))
        self.neutral_pos = np.array(self.neutral_pos, dtype=np.float32)

        self.leader_config = add_info_robot_config(self.control_config, verbose=False)

        self.pos_lock = Lock()
        self.command_lock = Lock()

        self.idx_map_name_arms, self.idx_map_name_hands = {}, {}
        self.all_hand_names, self.hand_offsets, self.hand_scales = [], [], []
        # order mapping from follower joint names to leader joint names
        for idx, (leader_name, follower_name) in enumerate(self.control_config.limb_mapping.items()):
            arm_indices, hand_indices = [], []
            hand_offsets, hand_scales = [], []

            leader_joint_names = self.control_config.limb_joint_names[leader_name]
            self.idx_map_name_arms[leader_name] = leader_joint_names
            for hand_name in self.control_config.hand_joint_names[leader_name]:
                hand_indices.append(hand_name)
                self.all_hand_names.append(hand_name)
                if "joint_limits" in self.robot_config.robot_cfg:
                    follower_min_val = self.robot_config.robot_cfg.joint_limits[0][joint_idx]
                    follower_max_val = self.robot_config.robot_cfg.joint_limits[1][joint_idx]
                else:
                    follower_min_val = 0.
                    follower_max_val = 1.
                leader_min_val, leader_max_val = self.control_config.hand_limits[hand_name]
                scale = (follower_max_val - follower_min_val) / (leader_max_val - leader_min_val)
                offset = follower_min_val - leader_min_val * scale
                hand_offsets.append(offset)
                hand_scales.append(scale)
            if len(hand_indices) > 0:
                self.idx_map_name_hands[leader_name] = hand_indices
                self.hand_offsets.append(hand_offsets)
                self.hand_scales.append(hand_scales)
        self.hand_offsets = np.array(self.hand_offsets).squeeze()
        self.hand_scales = np.array(self.hand_scales).squeeze()
        assert len(self.idx_map_name_arms) == self.num_limbs

        self.subscriber = self.node.create_subscription(JointState, control_config.leader_subscribe_topic, self.joint_state_callback, 10)
        self.imu_subscriber = self.node.create_subscription(Imu, '/leaders/data', self.imu_state_callback, 10)
        self.leader_joint_names = None
        self.log_info = self.node.get_logger().info

        def spin_executor(arm):
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor()
            executor.add_node(arm)
            executor.spin()
            return
        self.sub_thread = threading.Thread(target=spin_executor, args=(self.node,))
        self.sub_thread.start()
        self.ros_shutdown = lambda : (self.node.executor is not None and self.node.executor._is_shutdown)

        while self.leader_states is None:
            #print("Waiting for the leader joint states...")
            time.sleep(0.01)

        self.end_detection_thread = threading.Thread(target=self.detect_end_signal)
        self.end_detection_thread.start()

        self.leg_activation_thread = threading.Thread(target=self.detect_leg_activate_signal)
        self.leg_activation_thread.start()

    def launch_init(self, init_qpos=None):
        gripper_th, iteration = self.gripper_th, 0
        self.log_info("[Child Leaders] Initializing the controller...")

        while not ((self.positions[self.hand_inds] < gripper_th).all()):
            print(self.positions[self.hand_inds])
            print("Open the gripper to initialize the controller....", end="\r")
            time.sleep(0.1)
        self.init_thread = threading.Thread(target=self.initialize, args=(init_qpos,))
        self.init_thread.start()
        return

    def close_init(self):
        self.init_thread.join()
        return

    def initialize(self, init_qpos=None):
        self.last_command = None
        gripper_th, iteration, initialize_progress = self.gripper_th, 0, 0
        dt, threshold_time = 0.01, 3
        prev_pose = None
        # Squeeze all the grippers more than the threshold time (3 sec)
        while not initialize_progress >= threshold_time:
            if self.shutdown or self.ros_shutdown(): return

            curr_pose = copy.deepcopy(self.positions)
            #print(self.hand_inds, 'current_pose', curr_pose)
            hand_closed = ((curr_pose[self.hand_inds] > gripper_th).all())
            if hand_closed:
                initialize_progress += dt
            else:
                initialize_progress = 0

            curr_status =' '.join([f"{p:.3f}" for p in curr_pose[self.hand_inds[::-1]].squeeze()])

            # if init_qpos is not None:
            #     diff_from_init = abs(curr_pose[self.arm_inds] - init_qpos[0])
            #     ri, ji = np.where(diff_from_init > 0.2)
            #     curr_status += f" diff_init: "
            #     for rii, jii in zip(ri, ji):
            #         curr_status += f"r{rii}-j{jii}:{diff_from_init[rii][jii]:.2f} "

            diff = np.linalg.norm(curr_pose[self.arm_inds] - prev_pose) if prev_pose is not None else 0
            if diff > 0.01:
                initialize_progress = 0
                curr_status += "Don't move!"

            print(
                f"Close the gripper to initialize the controller.... {initialize_progress:.2f}/{threshold_time}   gripper: " + curr_status,
                end="\r")
            self.controller_info_for_vis['color'] = 'blue'
            self.controller_info_for_vis['log'] = f"Initialize Gripper  {initialize_progress:.2f}/{threshold_time}  gripper:" + curr_status

            time.sleep(0.01)

            iteration += 1
            prev_pose = curr_pose[self.arm_inds]

        self.log_info("[Child Leaders] Gripper closed. Initializing the controller...")
        self.controller_info_for_vis['color'] = 'red'
        self.controller_info_for_vis['log'] = 'Initializing the controller...'

        if not self.direct_joint_mapping:
            pin_qpos = pin.neutral(self.model)
            pin_qpos[self.idx_state2pin[:,0]] = curr_pose[self.idx_state2pin[:,1]]

            # pin_qpos = self.positions[self.idx_state2pin]

            self.init_eef_poses = self.get_eef_poses(pin_qpos)

            self.init_ts, self.init_Rs = [], []
            for Rt in self.init_eef_poses:
                self.init_ts.append(Rt[:3])
                self.init_Rs.append(Rt[:3, :3])

        self.controller_info_for_vis['color'] = 'green'
        self.controller_info_for_vis['log'] = 'Controller initialized!'

        self.is_ready = True
        self.require_end = False
        # self.logger.info("[Child Leaders] Controller initialized!")
        self.log_info("[Child Leaders] Controller initialized!")
        return

    def get_ready(self):
        return self.is_ready

    def detect_leg_activate_signal(self):
        dt = 0.01
        gripper_th, iteration = self.gripper_th, 0
        enough_to_switch, threshold_time = np.zeros([len(self.hand_inds)]), 1
        self.leg_mode = np.array([MODE_LEG_DEACTIVATED] * len(self.hand_inds))
        safe_transition = np.ones([len(self.hand_inds)]).astype(bool)
        while not self.shutdown and not self.ros_shutdown():
            start_time = time.time()
            if self.leader_states is None or not self.is_ready:
                time.sleep(dt)
                continue
            with self.pos_lock:
                positions = self.positions.copy()
            hand_closed = (positions[self.hand_inds] > gripper_th)
            increase_dt = hand_closed * safe_transition
            if safe_transition[0] == 0:
                if hand_closed[0] == 0: safe_transition[0] = 1
                else: print("Let go the left gripper")
            if safe_transition[1] == 0:
                if hand_closed[1] == 0: safe_transition[1] = 1
                else: print("Let go the right gripper")

            num_need_to_increase = np.sum(increase_dt)
            if not self.require_end and num_need_to_increase > 0 and num_need_to_increase != len(self.hand_inds):
                enough_to_switch[increase_dt] += dt
                enough_to_switch[~increase_dt] = 0
                print(f"Switching legs... {enough_to_switch[0]:.2f}/{threshold_time} {enough_to_switch[1]:.2f}/{threshold_time}", end="\r")

            enough_to_switch_mask = (enough_to_switch > threshold_time)

            if enough_to_switch_mask.all(): # This should never happen
                enough_to_switch = np.zeros([len(self.hand_inds)])
            elif enough_to_switch_mask[0] == True:
                self.leg_mode[0] = MODE_LEG_ACTIVATED if self.leg_mode[0] == MODE_LEG_DEACTIVATED else MODE_LEG_DEACTIVATED
                enough_to_switch[0], safe_transition[0] = 0, 0
            elif enough_to_switch_mask[1] == True:
                self.leg_mode[1] = MODE_LEG_ACTIVATED if self.leg_mode[1] == MODE_LEG_DEACTIVATED else MODE_LEG_DEACTIVATED
                enough_to_switch[1], safe_transition[1] = 0, 0

            left_time = max(dt - (time.time() - start_time), 0)
            time.sleep(left_time)
            iteration += 1

        
    def detect_end_signal(self):
        dt = 0.01
        gripper_th, iteration = self.gripper_th, 0
        enough_to_end, threshold_time = 0.0, 3
        while not self.shutdown and not self.ros_shutdown():
            if self.leader_states is None or not self.is_ready:
                time.sleep(dt)
                continue
            start_time = time.time()
            self.controller_info_for_vis['color'] = 'green'
            with self.pos_lock:
                positions = self.positions.copy()
            #pin_qpos = pin.neutral(self.model)
            #pin_qpos[self.idx_state2pin[:,0]] = positions[self.idx_state2pin[:,1]]
            self.controller_info_for_vis['log'] = 'Running.' + '.' * (iteration % 5)
            #new_eef_Rts = self.get_eef_poses(pin_qpos)

            # diffs = []
            # for i in range(self.num_limbs):
            #     #print(new_eef_Rts[i][:3, 3], self.control_config.reset_pose[i])
            #     diff = abs(new_eef_Rts[i][:3, 3] - np.array(self.control_config.reset_pose[i])) < np.array(self.control_config.reset_cube)
            #     diffs.append(diff.all())

            hand_closed = (positions[self.hand_inds] > gripper_th).all()
            #wrist_up = np.array(diffs).all()
            if not self.require_end and hand_closed:# and wrist_up:
                enough_to_end += dt
                #self.controller_info_for_vis['color'] = (enough_to_end * 255 / threshold_time, 255 - enough_to_end * 255 / threshold_time, 0)
                #self.controller_info_for_vis['log'] = f"Running... End signal detected! {enough_to_end:.2f}/{threshold_time:d}"
                print(f"Running... End signal detected! {enough_to_end:.2f}/{threshold_time:d}", end="\r")
            else:
                if enough_to_end > 0:
                    print(f"Running... End signal detected! 0.0/{threshold_time:d}", end="\r")
                enough_to_end = 0

            if enough_to_end > threshold_time:
                self.require_end = True
                self.controller_info_for_vis['color'] = 'red'
                self.controller_info_for_vis['log'] = 'End signal detected! Resetting the controller...'
                self.is_ready = False
                enough_to_end = 0
                self.log_info("[Child Leaders] End signal detected!, resetting the controller...")

            left_time = max(dt - (time.time() - start_time), 0)
            time.sleep(left_time)
            iteration += 1
        return
    
    def imu_state_callback(self, msg):
        if self.leader_states is None: return 
        orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z,msg.orientation.w]
        r = R.from_quat(orientation)
        pitch, roll, yaw = np.multiply(np.array(r.as_euler('xyz')), np.array([1, -1, 1]))
        pitch  = pitch + np.pi / 2
        if abs(pitch) < 0.2:
            pitch = 0
        else:
            pitch = pitch - 0.2 * np.sign(pitch)
        if abs(roll) < 0.2:
            roll = 0
        else:
            roll = roll - 0.2 * np.sign(roll)
        if abs(yaw) < 0.2:
            yaw = 0
        else:
            yaw = yaw - 0.2 * np.sign(yaw)

        yaw = np.clip(yaw, -np.pi/3, np.pi/3)
        if self.last_pitch is not None:
            diff = pitch - self.last_pitch
            if abs(diff) > 0.2: pitch = self.last_pitch
        if self.last_roll is not None:
            diff = roll - self.last_roll
            if abs(diff) > 0.2: roll = self.last_roll
        if self.last_yaw is not None:
            diff = yaw - self.last_yaw
            if abs(diff) > 0.2: yaw = self.last_yaw
        self.waist_euler = np.array([pitch, roll, yaw])
        self.last_pitch, self.last_roll, self.last_yaw = pitch, roll, yaw

    def joint_state_callback(self, msg):
        if self.leader_joint_names is None:
            self.leader_joint_names = msg.name
            self.arm_inds, self.hand_inds = [], []
            self.arm_inds_dict, self.hand_inds_dict = {}, {}
            print("direct joint_mapping ", self.direct_joint_mapping, self.idx_map_name_hands)
            if self.direct_joint_mapping:
                for limb_name in self.idx_map_name_arms.keys():
                    idx_map_namd_arm = self.idx_map_name_arms[limb_name]
                    arm_inds = [msg.name.index(name) for name in idx_map_namd_arm]
                    self.arm_inds.extend(arm_inds)
                    self.arm_inds_dict[limb_name] = arm_inds
                    if limb_name in self.idx_map_name_hands:
                        idx_map_namd_hand = self.idx_map_name_hands[limb_name]
                        hand_inds = [msg.name.index(name) for name in idx_map_namd_hand]
                        self.hand_inds.extend(hand_inds)
                        self.hand_inds_dict[limb_name] = hand_inds
            else:
                self.arm_inds_dict, self.hand_inds_dict = {}, {}
                for idx, name in enumerate(msg.name):
                    if name in self.all_hand_names:
                        for limb_name in self.idx_map_name_hands.keys():
                            if name in self.idx_map_name_hands[limb_name]:
                                self.hand_inds.append([idx]) # only assume one hand joint
                    else:
                        self.arm_inds.append(idx)

            #idx_state2pin = [[id,msg.name.index(name)] for id, name in enumerate(self.pin_model_joint_names) if name in msg.name]
            #self.idx_state2pin = np.array(idx_state2pin)

        new_positions = np.array(msg.position)
        new_positions[self.hand_inds] = new_positions[self.hand_inds] * self.hand_scales + self.hand_offsets

        with self.pos_lock:
            self.positions = new_positions
        self.leader_states = self.positions

    def get_eef_poses(self, pin_qpos):
        pin.forwardKinematics(self.model, self.data, pin_qpos)
        eef_poses = []
        for frame_id in self.end_effector_frame_ids:
            oMf: pin.SE3 = pin.updateFramePlacement(self.model, self.data, frame_id)
            xyzw_pose = pin.SE3ToXYZQUAT(oMf)
            pose = np.concatenate(
                [
                    xyzw_pose[:3],
                    np.array([xyzw_pose[6], xyzw_pose[3], xyzw_pose[4], xyzw_pose[5]]),
                ]
            )
            Rt = pt.transform_from_pq(pose)
            eef_poses.append(Rt)
        return eef_poses

    def get_status(self):
        if self.direct_joint_mapping:
            out_pose, out_hand_pose = {}, {}
            for leader_limb_name in self.leader_limb_names:
                inds = self.arm_inds_dict[leader_limb_name]
                follower_limb_name = self.control_config.limb_mapping[leader_limb_name]
                out_pose[follower_limb_name] = self.positions[inds].copy() #* 0.0
                out_hand_pose = None
                if leader_limb_name in inds:
                    inds = self.hand_inds_dict[leader_limb_name]
                    out_hand_pose[follower_limb_name] = self.positions[inds].copy()
            out_pose['waist'] = self.waist_euler
            with self.command_lock:
                self.last_command = (out_pose, out_hand_pose)
            return (out_pose,out_hand_pose)
        else:
            pin_qpos = pin.neutral(self.model)
            pin_qpos[self.idx_state2pin[:,0]] = self.positions[self.idx_state2pin[:,1]]
            new_eef_Rts = self.get_eef_poses(pin_qpos)
            local_pose = {}
            arm_qpos = {}
            if self.joint_mapping_method == 'ik_leader':
                for Rt, init_Rt, T_inv, ik_solver, hand_ind in zip(new_eef_Rts, self.init_eef_poses,
                                                                   self.transformation_matrices, self.ik_solvers,
                                                                   self.hand_inds):
                    Rt = T_inv @ Rt

                    xyzw_pose = pt.pq_from_transform(Rt)
                    pose = xyzw_pose[:3]
                    quat = xyzw_pose[3:]

                    qpos = ik_solver.solve(pose, quat)
                    ik_solver.set_current_qpos(np.concatenate([qpos, [self.positions[hand_ind]]]))
                    arm_qpos.append(qpos)

                    new_pos = init_Rt[:3, :3].T @ Rt[:3, 3] - init_Rt[:3, :3].T @ init_Rt[:3, 3]
                    new_R = init_Rt[:3, :3].T @ Rt[:3, :3]
                    new_pos = new_pos * self.motion_scale
                    new_Rt = pt.transform_from(R=new_R, p=new_pos)
                    local_pose.append(pt.pq_from_transform(new_Rt))
            else:
                out_hand_pose = {}
                for idx, (Rt, init_Rt) in enumerate(zip(new_eef_Rts, self.init_eef_poses)):
                    new_pos = init_Rt[:3,:3].T @ Rt[:3, 3] - init_Rt[:3,:3].T @ init_Rt[:3,3]
                    new_R = init_Rt[:3,:3].T @ Rt[:3,:3]
                    new_pos = new_pos * self.motion_scale
                    new_Rt = pt.transform_from(R=new_R, p=new_pos)
                    leader_limb_name = self.leader_limb_names[idx]
                    follower_limb_name = self.control_config.limb_mapping[leader_limb_name]
                    local_pose[leader_limb_name] = pt.pq_from_transform(new_Rt)
                    inds = self.hand_inds[idx]
                    out_hand_pose[follower_limb_name] = self.positions[inds].copy()
                arm_qpos = local_pose
            with self.command_lock:
                self.last_command = (arm_qpos, out_hand_pose)
            return (arm_qpos, out_hand_pose)


    def close(self):
        self.shutdown = True
        self.end_detection_thread.join()
        if self.init_thread is not None:
            self.init_thread.join()
        self.log_info("[Child Leaders] Controller closed!")
        return

    def reset(self):
        self.last_command = None
        return

    def compute_ee_pose(self, qpos: np.ndarray) -> np.ndarray:
        pin.forwardKinematics(self.model, self.data, qpos)
        oMf: pin.SE3 = pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        xyzw_pose = pin.SE3ToXYZQUAT(oMf)
        return np.concatenate(
            [
                xyzw_pose[:3],
                np.array([xyzw_pose[6], xyzw_pose[3], xyzw_pose[4], xyzw_pose[5]]),
            ]
        )


    def update_vis_info(self, vis_info):
        #vis_info['controller'] = self.controller_info_for_vis
        return vis_info


    def send_request(self, joint_names, joint_poses, joint_kps=None):
        if joint_kps is None:
            joint_kps = [0.02] * len(joint_names)
        elif isinstance(joint_kps, float):
            joint_kps = [joint_kps] * len(joint_names)

        js = JointState()
        js.name = joint_names
        js.position = joint_poses.tolist()
        js.velocity = joint_kps

        for i in range(10):
            self.set_base_pose_pub.publish(js)
            time.sleep(0.01)
        #self.set_base_pose_req.joint_state = js
        #future = self.set_base_pose_cli.call_async(self.set_base_pose_req)
        #rclpy.spin_until_future_complete(self.node, future)
        return 
