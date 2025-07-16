# Template for the Device class
class Device:
    def __init__(self, robot_config, device_config, env_config, *args, **kwargs):
        self.is_ready = False
        self.require_end = False
        self.shutdown = False
        return

    def reset(self, ):
        return

    def launch_init(self):
        return

    def close_init(self):
        return

    def get_status(self):
        raise NotImplementedError

    def set_feedback(self, feedback):
        # gripper feedback
        # eef direction feedback
        # joint limit feedback (when direct mapping)
        # - rather more meaningful when different configuration?
        return

    def update_vis_info(self, env_vis_info):
        return env_vis_info

    def close(self):
        return
