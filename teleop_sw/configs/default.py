import argparse
from argparse import RawTextHelpFormatter
import os
import time
from omegaconf import OmegaConf
from src.utils.misc import print_color
from configs.utils import add_info_robot_config, sanity_check_device_config

def str2bool(v):
    if isinstance(v, bool): return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'): return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'): return False
    else: raise argparse.ArgumentTypeError('Boolean value expected.')

class BaseConfig:
    def __init__(self):
        # Change working directory to the root of the project
        is_root_dir = 'configs' in os.listdir()
        if not is_root_dir:
            os.chdir(os.path.dirname(os.path.abspath(__file__)) + "/../")


    def parse(self, verbose=True):
        parser = argparse.ArgumentParser(add_help=False, formatter_class=RawTextHelpFormatter)
        parser.add_argument('--robot','-r', type=str, default='g1')
        parser.add_argument('--device', '-d', type=str, default='child_g1')

        parser.add_argument('--no-cam', '--no_cam', action='store_true', default=False)
        # Help
        parser.add_argument('--help', action='help',
                            help=f'Possible options for Robots are: {[k.replace(".yaml", "") for k in os.listdir("configs/robot")]}\n'
                                 f'Possible options for Controllers are: {[k.replace(".yaml", "") for k in os.listdir("configs/device")]}')
        args, unknown = parser.parse_known_args()

        robot_config_file = f'configs/robot/{args.robot}.yaml'
        if not os.path.exists(robot_config_file):
            raise FileNotFoundError(f"Robot config file {robot_config_file} does not exist. \n"
                                    f"Possible options are {[k.replace('.yaml','') for k in os.listdir('configs/robot')]}")
        device_config_file = f'configs/device/{args.device}.yaml'
        if not os.path.exists(device_config_file):
            raise FileNotFoundError(f"Device config file {device_config_file} does not exist. \n"
                                    f"Possible options are {[k.replace('.yaml','') for k in os.listdir('configs/device')]}")

        print_color("-------Configuration-------", color='light_green', attrs=['bold'])
        print(f"🎮 Controller: {args.device}")
        print(f"🤖 Robot: {args.robot}")
        #print(f"🌏 Env: {args.env}")
        print_color("---------------------------", color='light_green', attrs=['bold'])

        override_config = OmegaConf.load(f'configs/robot/{args.robot}.yaml')
        cli_config = OmegaConf.from_dotlist(unknown)
        self.robot_config = OmegaConf.merge( override_config, cli_config)
        self.robot_config.robot_cfg = add_info_robot_config(self.robot_config.robot_cfg)

        override_config = OmegaConf.load(f'configs/device/{args.device}.yaml')
        cli_config = OmegaConf.from_dotlist(unknown)
        self.device_config = OmegaConf.merge(override_config, cli_config)
        self.device_config = sanity_check_device_config(self.device_config, self.robot_config)


        return self.robot_config, self.device_config

if __name__ == '__main__':

    BaseConfig().parse()
