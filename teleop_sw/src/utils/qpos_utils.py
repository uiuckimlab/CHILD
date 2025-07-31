import numpy as np

def get_joint_pos_by_limb(robot_config, qpos, type=dict):
    """
    Get joint positions by limb
    """
    limb_joint_names = robot_config.limb_joint_names
    limb_joint_idx_mapping = robot_config.ctrl_joint_idx_mapping
    if type == dict:
        joint_pos_by_limb = {}
        for limb_name, limb_joint_names in limb_joint_names.items():
            # Note: this does not separate the arm and hand joints
            joint_pos_by_limb[limb_name] = qpos[limb_joint_idx_mapping[limb_name]]

    elif type == list:
        joint_pos_by_limb = [[],[]]
        for limb_name, limb_joint_names in limb_joint_names.items():
            type_list = robot_config.ctrl_joint_type[limb_name]
            arm_joint_inds = np.flatnonzero(np.logical_not(type_list))
            hand_joint_inds = np.flatnonzero(type_list)
            joint_pos_by_limb[0].append(qpos[limb_joint_idx_mapping[limb_name]][arm_joint_inds])
            joint_pos_by_limb[1].append(qpos[limb_joint_idx_mapping[limb_name]][hand_joint_inds])
    return joint_pos_by_limb


def get_joint_pos_as_command(robot_config, leader_command, return_hand_inds=False):
    limb_joint_names = robot_config.limb_joint_names
    limb_joint_idx_mapping = robot_config.ctrl_joint_idx_mapping
    total_qpos, hand_inds = [], []
    for limb_name, limb_joint_names in limb_joint_names.items():
        # Note: this does not separate the arm and hand joints
        total_qpos.extend(np.concatenate([leader_command[0][limb_name], leader_command[1][limb_name]]))
        if return_hand_inds:
            hand_joint_inds = np.flatnonzero(robot_config.ctrl_joint_type[limb_name])
            hand_inds.extend(np.array(robot_config.ctrl_joint_idx_mapping[limb_name])[hand_joint_inds].tolist())
    if return_hand_inds:
        return np.array(total_qpos), hand_inds
    return np.array(total_qpos)
