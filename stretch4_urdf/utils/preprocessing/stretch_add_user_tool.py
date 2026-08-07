#!/usr/bin/env python3

import os
import re
import sys
import argparse
import ast
import shutil
import yaml
import glob




def get_fleet_directory():
    """Returns the fleet directory where configuration and user parameter YAML files are located."""
    if 'HELLO_FLEET_PATH' in os.environ and 'HELLO_FLEET_ID' in os.environ:
        return os.environ['HELLO_FLEET_PATH'] + '/' + os.environ['HELLO_FLEET_ID'] + '/'
    
    # Fallback scanning of ~/stretch_user/stretch-* for a directory
    user_dir = os.path.expanduser('~/stretch_user')
    if os.path.exists(user_dir):
        subdirs = [os.path.join(user_dir, d) for d in os.listdir(user_dir) if d.startswith('stretch-')]
        if subdirs:
            return subdirs[0] + '/'
    return '/tmp/'


def derive_gripper_class_name(py_class_name):
    """Name of the actuator driver class in tool.py, kept distinct from the
    EndOfArm class in end_of_arm.py. A tool name already ending in 'gripper'
    keeps its derived name instead of stuttering into e.g. NyuGripperGripper."""
    if py_class_name.lower().endswith('gripper'):
        return py_class_name
    return py_class_name + 'Gripper'


def save_tool_params(tool_name, py_class_name, py_module_name, client_class_name=None, client_module_name=None, tool_path=None, gripper_module_name=None):
    """Automatically writes baseline configuration and collision management to tool_params.yaml in the custom tool folder."""
    if not tool_path:
        print("Warning: Tool path not specified. Skipping tool_params.yaml generation.")
        return

    tool_params_file = os.path.join(tool_path, 'tool_params.yaml')
    
    if os.path.exists(tool_params_file):
        print(f"Configuration for '{tool_name}' already exists in tool_params.yaml. Leaving as is.")
        return

    # Baseline configuration and collision management
    baseline_config = {
        'py_class_name': py_class_name,
        'py_module_name': py_module_name,
    }
    if client_class_name:
        baseline_config['client_class_name'] = client_class_name
    if client_module_name:
        baseline_config['client_module_name'] = client_module_name

    baseline_config.update({
        'use_group_sync_read': 0,
        'use_group_sync_write': 0,
        'retry_on_comm_failure': 1,
        'baud': 1000000,
        'i_feedforward_payload': 0.3,
        'wrist': 'eoaw_dw4',
        'tool': tool_name,
        'stow': {
            'arm': 0.0,
            'lift': 0.15,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.14,
            f'{tool_name}': 0
        },
        'devices': {
            'wrist_pitch': {
                'py_class_name': 'WristPitch',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_pitch',
                'device_params': 'SE4_wrist_pitch_DW4'
            },
            'wrist_roll': {
                'py_class_name': 'WristRoll',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_roll',
                'device_params': 'SE4_wrist_roll_DW4'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch4_body.subsystem.end_of_arm.wrist_yaw',
                'device_params': 'SE4_wrist_yaw_DW4'
            },
            f'{tool_name}': {
                'py_class_name': derive_gripper_class_name(py_class_name),
                'py_module_name': gripper_module_name or py_module_name,
                'device_params': None,
                'id': 24,
                'eeprom_cfg': {
                    'temperature_limit': 72,
                    'max_voltage_limit': 29,
                    'min_voltage_limit': 11,
                    'pid': [32, 32, 0],
                    'return_delay_time': 0,
                    'angular_resolution': 1.0,
                    'phase': 45,
                    'max_pos_limit': 4095,
                    'min_pos_limit': 0,
                    'max_load_limit_pct': 48.0,
                    'overload_safe': 25.0,
                    'overload_time_ms': 1000,
                    'overload_thresh': 48.0,
                    'overcurrent': 150,
                    'overcurrent_time_ms': 200,
                    'protection_torque': 20,
                    'overload_protection_time': 200,
                    'enable_protection_overload': 0,
                    'enable_protection_current': 1,
                    'enable_protection_temp': 1,
                    'enable_protection_sensor': 0,
                    'enable_protection_voltage': 0
                },
                'motion': {
                    'default': {'accel': 6.0, 'vel': 6.0},
                    'fast': {'accel': 6.0, 'vel': 6.0},
                    'max': {'accel': 6.0, 'vel': 6.0},
                    'slow': {'accel': 4.0, 'vel': 1.0},
                    'vel_brakezone_factor': 1,
                    'vel_is_moving_thresh': 0.01
                },
                'set_safe_velocity': 1,
                'req_calibration': 1,
                'gr': 1.0,
                'usb_name': '/dev/hello-feetech-wrist',
                'retry_on_comm_failure': 1,
                'baud': 1000000,
                'range_pad_deg': [0.0, 0.0],
                'range_mm': 80.0,
                'range_deg': [0, 116.5],
                'homing_offset_bias_t': 0,
                'homing_to_neg_limit': 1,
                'homing_pwm': -150,
                'flip_encoder_polarity': 1,
                'kL': 30.25,
                'kR': 22.0,
                'kT0': 44.0,
                'kX0': 10.5,
                'stall_backoff': 0.017,
                'stall_max_effort': 20.0,
                'stall_max_time': 1.0,
                'stall_min_vel': 0.1,
                'disable_torque_on_runstop': 0,
                'enable_torque_after_runstop': 1,
                'enable_runstop': 1
            }
        },
        'collision_mgmt': {
            'k_brake_distance': {
                'wrist_pitch': 0.25,
                'wrist_yaw': 0.25,
                'wrist_roll': 0.25
            },
            'collision_pairs': {
                'wrist_pitch_link_TO_base_link': {
                    'link_pts': 'wrist_pitch_link',
                    'link_cube': 'base_link',
                    'detect_as': 'pts'
                },
                'wrist_yaw_bottom_link_TO_base_link': {
                    'link_pts': 'wrist_yaw_bottom_link',
                    'link_cube': 'base_link',
                    'detect_as': 'pts'
                }
            },
            'joints': {
                'lift': [
                    {'motion_dir': 'neg', 'collision_pair': 'wrist_pitch_link_TO_base_link'},
                    {'motion_dir': 'neg', 'collision_pair': 'wrist_yaw_bottom_link_TO_base_link'}
                ]
            }
        },
        'ros': {
            'joints': [
                {
                    'py_module_name': 'command_group',
                    'py_class_name': f'{py_class_name}CommandGroup'
                }
            ]
        },
        'self_collision_mujoco': {
            'k_brake_distance': {},
            'exclusions': []
        }
    })

    try:
        with open(tool_params_file, 'w') as f:
            yaml.dump(baseline_config, f, default_flow_style=False, sort_keys=False)
        print(f"Successfully generated tool-specific parameters at: {tool_params_file}")
    except Exception as e:
        print(f"Error writing tool-specific parameters to tool_params.yaml: {e}")


def get_user_tools_dirs():
    _dirs = []
    _fleet_path = os.environ.get('HELLO_FLEET_PATH')
    if _fleet_path:
        _shared_dir = os.path.join(_fleet_path, 'user_tools')
        if os.path.exists(_shared_dir):
            _dirs.append(_shared_dir)
    else:
        _default_dir = os.path.expanduser('~/stretch_user/user_tools')
        if os.path.exists(_default_dir):
            _dirs.append(_default_dir)
    return _dirs


def check_user_tool(tool_name, tool_path):
    print(f"\n=========================================")
    print(f"🔍 CHECKING USER TOOL: {tool_name}")
    print(f"Path: {tool_path}")
    print(f"=========================================\n")
    
    passed = True
    
    # 1. Check tool_params.yaml exists
    yaml_path = os.path.join(tool_path, 'tool_params.yaml')
    if not os.path.exists(yaml_path):
        print("❌ FAILED: tool_params.yaml not found.")
        return False
    print("✅ PASSED: tool_params.yaml exists.")
    
    # 2. Parse tool_params.yaml
    try:
        with open(yaml_path, 'r') as f:
            params = yaml.safe_load(f)
        print("✅ PASSED: tool_params.yaml parsed successfully as valid YAML.")
    except Exception as e:
        print(f"❌ FAILED: Failed to parse tool_params.yaml: {e}")
        return False
        
    # Ensure sys.path is added
    from stretch4_body.utils.user_tool_utils import add_user_tool_to_sys_path
    add_user_tool_to_sys_path(tool_name)
    from stretch4_body.core.robot_params import RobotParams
    
    # 3. Verify Server-side driver config
    server_module = params.get('py_module_name')
    server_class = params.get('py_class_name')
    
    if not server_module or not server_class:
        print("❌ FAILED: 'py_module_name' or 'py_class_name' not defined in tool_params.yaml.")
        passed = False
    else:
        try:
            mod = RobotParams.import_user_tool_module(tool_name, server_module, is_server=True)
            if not mod:
                raise ImportError(f"Could not load module '{server_module}'")
            if not hasattr(mod, server_class):
                raise AttributeError(f"Module '{server_module}' has no class '{server_class}'")
            print(f"✅ PASSED: Server class '{server_class}' successfully loaded from module '{server_module}'.")
        except Exception as e:
            print(f"❌ FAILED: Could not load server class '{server_class}' from module '{server_module}': {e}")
            passed = False
            
    # 4. Verify Client-side driver config
    client_module = params.get('client_module_name')
    client_class = params.get('client_class_name')
    
    if client_module or client_class:
        if not client_module or not client_class:
            print("❌ FAILED: Both 'client_module_name' and 'client_class_name' must be defined together.")
            passed = False
        else:
            try:
                mod = RobotParams.import_user_tool_module(tool_name, client_module, is_server=False)
                if not mod:
                    raise ImportError(f"Could not load module '{client_module}'")
                if not hasattr(mod, client_class):
                    raise AttributeError(f"Module '{client_module}' has no class '{client_class}'")
                print(f"✅ PASSED: Client class '{client_class}' successfully loaded from module '{client_module}'.")
            except Exception as e:
                print(f"❌ FAILED: Could not load client class '{client_class}' from module '{client_module}': {e}")
                passed = False
                
    # 5. Verify devices configuration
    devices = params.get('devices', {})
    if not devices:
        print("⚠️  WARNING: No devices configuration found under 'devices' key.")
    else:
        print("\nChecking devices:")
        for dev_name, dev_cfg in devices.items():
            if not isinstance(dev_cfg, dict):
                continue
            d_class = dev_cfg.get('py_class_name')
            d_module = dev_cfg.get('py_module_name')
            
            if not d_class or not d_module:
                print(f"  ❌ Device '{dev_name}': missing py_class_name or py_module_name.")
                passed = False
                continue
                
            try:
                if "stretch4_body" in d_module:
                    import importlib
                    mod = importlib.import_module(d_module)
                else:
                    mod = RobotParams.import_user_tool_module(tool_name, d_module, is_server=True)
                
                if not mod:
                    raise ImportError(f"Could not load module '{d_module}'")
                if not hasattr(mod, d_class):
                    raise AttributeError(f"Module '{d_module}' has no class '{d_class}'")
                print(f"  ✅ Device '{dev_name}': class '{d_class}' successfully loaded from module '{d_module}'.")
            except Exception as e:
                print(f"  ❌ Device '{dev_name}': Failed to load class '{d_class}' from module '{d_module}': {e}")
                passed = False
                
    # 6. Verify collision mapper
    collision_file = os.path.exists(os.path.join(tool_path, 'collision.py'))
    if collision_file:
        try:
            sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', tool_name)
            if sanitized and sanitized[0].isdigit():
                sanitized = "_" + sanitized
            clean_class_base = re.sub(r'[^a-zA-Z0-9]', ' ', tool_name)
            if clean_class_base and clean_class_base[0].isdigit():
                clean_class_base = "Tool " + clean_class_base
            server_class_name = clean_class_base.title().replace(' ', '')
            
            mod = RobotParams.import_user_tool_module(tool_name, 'collision', is_server=True)
            collision_class = f"{server_class_name}Collision"
            if hasattr(mod, collision_class):
                print(f"✅ PASSED: Collision mapper class '{collision_class}' successfully loaded from collision.py.")
            else:
                print(f"⚠️  WARNING: collision.py exists but class '{collision_class}' was not found inside it.")
        except Exception as e:
            print(f"❌ FAILED: Error importing custom collision mapper collision.py: {e}")
            passed = False
    else:
        print("⚠️  WARNING: Custom collision.py mapper not found. Will use default visualizer joint mapping.")
        
    # 7. Verify pose models
    pose_yaml_path = os.path.join(tool_path, 'pose_models.yaml')
    if os.path.exists(pose_yaml_path):
        try:
            from stretch4_body.utils.stretch_pose_models import RobotPose
            poses = RobotPose.load_tool_pose_models(tool_name)
            if poses:
                print(f"✅ PASSED: pose_models.yaml successfully loaded and validated ({', '.join(poses.keys())}).")
            else:
                print("❌ FAILED: pose_models.yaml exists but loaded as empty or failed validation.")
                passed = False
        except Exception as e:
            print(f"❌ FAILED: Error parsing or validating pose_models.yaml: {e}")
            passed = False
    else:
        print("⚠️  WARNING: Custom pose_models.yaml not found.")
        
    # 8. Verify gripper conversion
    gripper_conv_file = os.path.join(tool_path, 'gripper_conversion.py')
    if os.path.exists(gripper_conv_file):
        try:
            mod = RobotParams.import_user_tool_module(tool_name, 'gripper_conversion', is_server=True)
            sanitized = re.sub(r'[^a-zA-Z0-9_]', '_', tool_name)
            if sanitized and sanitized[0].isdigit():
                sanitized = "_" + sanitized
            func_name = f"{sanitized}_servo_rad_to_mm"
            if hasattr(mod, func_name):
                print(f"✅ PASSED: Gripper conversion function '{func_name}' successfully loaded from gripper_conversion.py.")
            else:
                print(f"⚠️  WARNING: gripper_conversion.py exists but function '{func_name}' was not found inside.")
        except Exception as e:
            print(f"❌ FAILED: Error importing custom gripper conversion logic gripper_conversion.py: {e}")
            passed = False
    else:
        print("⚠️  WARNING: Custom gripper_conversion.py not found.")

    # 9. Verify custom gamepad class
    gamepad_file = os.path.join(tool_path, 'gamepad.py')
    if os.path.exists(gamepad_file):
        try:
            mod = RobotParams.import_user_tool_module(tool_name, 'gamepad', is_server=True)
            clean_class_base = re.sub(r'[^a-zA-Z0-9]', ' ', tool_name)
            if clean_class_base and clean_class_base[0].isdigit():
                clean_class_base = "Tool " + clean_class_base
            server_class_name = clean_class_base.title().replace(' ', '')
            custom_gripper_class_name = f"Command{server_class_name}Position"
            
            if hasattr(mod, custom_gripper_class_name):
                print(f"✅ PASSED: Gamepad class '{custom_gripper_class_name}' successfully loaded from gamepad.py.")
            else:
                print(f"⚠️  WARNING: gamepad.py exists but class '{custom_gripper_class_name}' was not found inside.")
        except Exception as e:
            print(f"❌ FAILED: Error importing custom gamepad logic gamepad.py: {e}")
            passed = False
    else:
        print("⚠️  WARNING: Custom gamepad.py not found.")
        
    print(f"\n-----------------------------------------")
    if passed:
        print(f"🎉 SUCCESS: Tool '{tool_name}' configuration is completely valid and correctly hooked up!")
    else:
        print(f"🚨 FAILURE: Tool '{tool_name}' configuration contains errors that will prevent it from loading.")
    print(f"-----------------------------------------\n")
    return passed


def main():
    parser = argparse.ArgumentParser(description="Process a user-defined custom tool using environment variables or fallbacks.")
    parser.add_argument('tool_name', nargs='?', help="Name of the custom tool subfolder.")
    parser.add_argument('--check', action='store_true', help="Verify that the tool configuration, parameters, classes, and modules resolve correctly.")
    
    args = parser.parse_args()
    
    user_tools_dirs = get_user_tools_dirs()
    if not user_tools_dirs:
        default_dir = os.path.expanduser('~/stretch_user/user_tools')
        print(f"Creating user tools directory at: {default_dir}")
        os.makedirs(default_dir, exist_ok=True)
        user_tools_dirs = [default_dir]

    # Gather all subdirectories across all active user tools directories
    subdirs = []
    subdir_paths = {}
    for d_path in user_tools_dirs:
        if os.path.exists(d_path):
            for d in os.listdir(d_path):
                if os.path.isdir(os.path.join(d_path, d)):
                    if d not in subdirs:
                        subdirs.append(d)
                        subdir_paths[d] = os.path.join(d_path, d)
    
    if args.tool_name:
        selected_tool = args.tool_name
        tool_name = os.path.basename(selected_tool.rstrip('/'))
        if not re.match(r'^[a-zA-Z0-9_]+$', tool_name):
            print(f"Error: Custom tool name '{tool_name}' must only contain alphanumeric characters and underscores (no hyphens or other special characters).")
            sys.exit(1)
        if selected_tool not in subdir_paths:
            target_abs_path = os.path.abspath(selected_tool)
            is_path_specified = (os.sep in selected_tool) or (selected_tool.startswith('.'))
            
            in_scanned = False
            for d_path in user_tools_dirs:
                parent_dir = os.path.abspath(d_path)
                if target_abs_path.startswith(parent_dir + os.sep) or target_abs_path == parent_dir:
                    in_scanned = True
                    tool_path = os.path.join(parent_dir, tool_name)
                    break
            
            if not in_scanned:
                if is_path_specified:
                    tool_path = target_abs_path
                else:
                    tool_path = os.path.join(user_tools_dirs[0], tool_name)
            
            print(f"Creating custom tool directory at: {tool_path}")
            os.makedirs(tool_path, exist_ok=True)
            os.makedirs(os.path.join(tool_path, 'meshes'), exist_ok=True)
            selected_tool = tool_name
        else:
            tool_path = subdir_paths[selected_tool]
    else:
        if not subdirs:
            print(f"No custom tools found under scanned directories: {user_tools_dirs}.")
            print("To add a tool, create a folder like 'user_eoa_mytool' with a URDF and 'meshes' folder.")
            return

        print("Available user tools:")
        for i, d in enumerate(subdirs):
            print(f"  {i+1}: {d} (located in {os.path.dirname(subdir_paths[d])})")

        print(f"\nSelect a user tool to process (1-{len(subdirs)}, or 'all'):")
        try:
            choice = input("> ").strip().lower()
        except KeyboardInterrupt:
            print("\nExiting.")
            sys.exit(0)

        if not choice:
            print("No selection made. Exiting.")
            sys.exit(0)

        if choice == 'all':
            if args.check:
                all_passed = True
                for d in subdirs:
                    all_passed = check_user_tool(d, subdir_paths[d]) and all_passed
                sys.exit(0 if all_passed else 1)
            else:
                for d in subdirs:
                    process_single_tool(d, subdir_paths[d])
                return
        else:
            try:
                idx = int(choice) - 1
                if 0 <= idx < len(subdirs):
                    selected_tool = subdirs[idx]
                    tool_path = subdir_paths[selected_tool]
                else:
                    print(f"Invalid index: {choice}")
                    sys.exit(1)
            except ValueError:
                print(f"Invalid input: {choice}")
                sys.exit(1)

    if args.check:
        passed = check_user_tool(selected_tool, tool_path)
        sys.exit(0 if passed else 1)

    process_single_tool(selected_tool, tool_path)


CUSTOM_TOOL_TEMPLATE = """#!/usr/bin/env python3
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

class {gripper_class_name}(FeetechSMHello):
    \"\"\"
    A completely custom gripper driver subclassing FeetechSMHello directly.
    \"\"\"
    def __init__(self, chain=None, usb=None, name='{tool_name}', is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb, is_direct=is_direct)
"""

CUSTOM_TOOL_END_OF_ARM_TEMPLATE = """#!/usr/bin/env python3
import time
import threading
from stretch4_body.subsystem.end_of_arm.end_of_arm import EndOfArm
from stretch4_body.subsystem.end_of_arm.end_of_arm_tools import home_dw4_joints

class {class_name}(EndOfArm):
    \"\"\"
    Server-side driver for custom end-of-arm tool {tool_name}.
    \"\"\"
    def __init__(self, name='{tool_name}'):
        EndOfArm.__init__(self, name)
        self.urdf_map = {{
            'wrist_yaw_joint': 'wrist_yaw',
            'wrist_pitch_joint': 'wrist_pitch',
            'wrist_roll_joint': 'wrist_roll'
        }}

    def stow(self):
        self.logger.info(f'--------- Stowing Custom Tool: {{self.name}} ---------')
        self.move_to('wrist_yaw', self.params['stow']['wrist_yaw'])
        self.move_to('wrist_roll', self.params['stow']['wrist_roll'])
        time.sleep(3.0)
        self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
        
        # Safely actuate custom {sanitized_tool_name} if it is registered in devices
        if '{sanitized_tool_name}' in self.motors:
            self.move_to('{sanitized_tool_name}', self.params['stow']['{sanitized_tool_name}'])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing Custom Tool: {{self.name}}')
            self.status['is_homing'] = True
            
            # Home the general wrist pitch, roll, and yaw joints
            success = home_dw4_joints(self)
            
            # Safely home the custom gripper motor if registered
            if '{sanitized_tool_name}' in self.motors:
                success = success and self.motors['{sanitized_tool_name}'].home()
                
            self.status['is_homing'] = False
            return success

        if wait_on_completion:
            return _do_home()
        
        thread = threading.Thread(target=_do_home)
        thread.start()
        return None

    def pre_stow(self, robot=None):
        if robot:
            robot.end_of_arm.move_to('wrist_pitch', robot.end_of_arm.params['stow']['wrist_pitch'])
        else:
            self.move_to('wrist_pitch', self.params['stow']['wrist_pitch'])
"""

CLIENT_TEMPLATE = """#!/usr/bin/env python3
from stretch4_body.robot.robot_client import EndOfArmClient

class {client_class_name}(EndOfArmClient):
    \"\"\"
    Client-side interface for custom end-of-arm tool {tool_name}.
    Sends command queues and receives status updates.
    \"\"\"
    def __init__(self, parent=None):
        EndOfArmClient.__init__(self, name='{tool_name}', parent=parent)
"""

COMMAND_GROUP_TEMPLATE = """#!/usr/bin/env python3
class {class_name}CommandGroup:
    \"\"\"
    ROS Command Group template for {tool_name}.
    Maps ROS JointState and trajectory commands to the custom python driver.
    \"\"\"
    def __init__(self, robot, node=None):
        self.robot = robot
        self.node = node
        self.joint_name = "{sanitized_tool_name}_joint"

    def get_joint_state(self):
        \"\"\"
        Returns current state of the joint to publish on ROS joint_states.
        \"\"\"
        status = self.robot.end_of_arm.status.get('{sanitized_tool_name}', {{}})
        return {{
            'name': self.joint_name,
            'pos': status.get('pos', 0.0),
            'vel': status.get('vel', 0.0),
            'effort': status.get('effort', 0.0)
        }}

    def command_joint(self, position, velocity=None, acceleration=None):
        \"\"\"
        Applies target command to the joint driver.
        \"\"\"
        self.robot.end_of_arm.move_to('{sanitized_tool_name}', position)
"""

GAMEPAD_TELEOP_TEMPLATE = """#!/usr/bin/env python3
from stretch4_body.core.robot_params import RobotParams

class {class_name}GamepadTeleop:
    \"\"\"
    Gamepad teleoperation control mapping template for {tool_name}.
    Maps joystick button and axis events to joint commands.
    \"\"\"
    def __init__(self, robot):
        self.robot = robot

    def update_teleop(self, gamepad_state):
        \"\"\"
        Processes raw joystick states and queues commands.
        gamepad_state: dict containing buttons (0/1) and axes (float -1.0 to 1.0)
        \"\"\"
        # Example: Command incremental positive movement when Right Trigger is pressed
        rt_val = gamepad_state.get('right_button_pressed', 0.0)
        if rt_val > 0.1:
            if hasattr(self.robot, 'end_of_arm') and self.robot.end_of_arm is not None:
                self.robot.end_of_arm.move_by('{sanitized_tool_name}', 0.01 * rt_val)
            else:
                self.robot.move_by('{sanitized_tool_name}', 0.01 * rt_val)
            
        # Example: Command incremental negative movement when Left Trigger is pressed
        lt_val = gamepad_state.get('bottom_button_pressed', 0.0)
        if lt_val > 0.1:
            if hasattr(self.robot, 'end_of_arm') and self.robot.end_of_arm is not None:
                self.robot.end_of_arm.move_by('{sanitized_tool_name}', -0.01 * lt_val)
            else:
                self.robot.move_by('{sanitized_tool_name}', -0.01 * lt_val)


class Command{class_name}Position:
    \"\"\"
    Custom Tool motion command class for gamepad teleoperation.
    For this class, simple open and close methods are provided
    and expected only to be controlled on a button state.
    \"\"\"
    def __init__(self, motion_profile:str = 'max'):
        from stretch4_body.utils.stretch_pose_models import RobotJoints
        self.name = RobotJoints.gripper.value or '{sanitized_tool_name}'
        self.params = RobotParams().get_params()[1][self.name]
        self.gripper_step_m = 0.01
        self.gripper_accel = self.params.get('motion', {{}}).get(motion_profile, {{}}).get('accel', 6.0)
        self.gripper_vel = self.params.get('motion', {{}}).get(motion_profile, {{}}).get('vel', 6.0)
        self.precision_mode = 0.0
        self.stop_reqd = False

    def _move(self, dx_m, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        dx_m = dx_m * scale
        robot.end_of_arm.move_by(self.name, dx_m, self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True
    
    def open_gripper(self, robot):
        self._move(self.gripper_step_m, robot)
        
    def close_gripper(self, robot):
        self._move(-self.gripper_step_m, robot)

    def stop_gripper(self, robot):
        if self.stop_reqd:
            robot.end_of_arm.move_by(self.name, 0.0)
            self.stop_reqd = False
"""

COLLISION_TEMPLATE = """#!/usr/bin/env python3
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.subsystem.end_of_arm.gripper_conversion import get_finger_joint_limits

class {class_name}Collision:
    \"\"\"
    Collision joint state mapping template for {tool_name}.
    Maps custom joint state values to Mujoco visualizer joints.
    \"\"\"
    def __init__(self, robot=None):
        self.robot = robot
        self._finger_limits = None

    def get_mujoco_joints(self, state):
        \"\"\"
        Given the raw robot status dictionary,
        return a dictionary mapping Mujoco joint names to their target positions.
        \"\"\"
        eoa = state.get('end_of_arm', {{}})

        # Look for the custom tool within the end of arm status
        tool_status = eoa.get('{sanitized_tool_name}') or {{}}

        # Device params for the gripper device named after the tool are merged
        # into the tool's top-level params block
        _, robot_params = RobotParams.get_params()
        tool_params = robot_params.get('{sanitized_tool_name}', {{}})

        # Drivers that publish a gripper_conversion status report the finger
        # angle directly; parallel-jaw drivers report an aperture in mm
        conversion = tool_status.get('gripper_conversion') or {{}}
        if 'finger_rad' in conversion:
            joint_val = conversion['finger_rad']
        else:
            if self._finger_limits is None:
                # Cached: this regenerates the robot URDF, too slow to call per cycle
                self._finger_limits = get_finger_joint_limits()
            lower, upper = self._finger_limits
            range_mm = tool_params.get('range_mm', 80.0)
            pct = tool_status.get('pos_mm', 0.0) / range_mm if range_mm else 0.0
            joint_val = upper + pct * (lower - upper)

        # Replace these with the finger joint names from your tool's URDF
        return {{
            'finger_left_joint': joint_val,
            'finger_right_joint': joint_val
        }}
"""

POSE_MODELS_TEMPLATE = """- name: stow
  timestamp: 0.0
  joints:
    wrist_pitch: {{position: 0.0, velocity: 0.0, effort: 0.0}}
    wrist_roll: {{position: 0.0, velocity: 0.0, effort: 0.0}}
    wrist_yaw: {{position: 3.14, velocity: 0.0, effort: 0.0}}
    {sanitized_tool_name}: {{position: 0.0, velocity: 0.0, effort: 0.0}}
- name: zero
  timestamp: 0.0
  joints:
    wrist_pitch: {{position: 0.0, velocity: 0.0, effort: 0.0}}
    wrist_roll: {{position: 0.0, velocity: 0.0, effort: 0.0}}
    wrist_yaw: {{position: 0.0, velocity: 0.0, effort: 0.0}}
    {sanitized_tool_name}: {{position: 0.0, velocity: 0.0, effort: 0.0}}
"""


GRIPPER_CONVERSION_TEMPLATE = """#!/usr/bin/env python3
import math

# def {sanitized_tool_name}_servo_rad_to_mm(servo_rad, params):
#     \"\"\"
#     Convert custom gripper servo angle (in radians) to gap width (in mm).
#     \"\"\"
#     range_rad = math.radians(params.get('range_deg', [0.0, 100.0])[1] - params.get('range_deg', [0.0, 100.0])[0])
#     range_mm = params.get('range_mm', 80.0)
#     if range_rad == 0:
#         return 0.0
#     return (servo_rad / range_rad) * range_mm
# 
# 
# def {sanitized_tool_name}_mm_to_servo_rad(x_mm, params):
#     \"\"\"
#     Convert custom gripper gap width (in mm) to servo angle (in radians).
#     \"\"\"
#     range_rad = math.radians(params.get('range_deg', [0.0, 100.0])[1] - params.get('range_deg', [0.0, 100.0])[0])
#     range_mm = params.get('range_mm', 80.0)
#     if range_mm == 0:
#         return 0.0
#     return (x_mm / range_mm) * range_rad
# 
# 
# def {sanitized_tool_name}_pos_mm_to_urdf_m(pos_mm, params):
#     \"\"\"
#     Convert custom gripper finger aperture (in mm) to URDF finger joint value (in meters).
#     \"\"\"
#     range_mm = params.get('range_mm', 80.0)
#     pct = pos_mm / range_mm if range_mm != 0 else 0.0
#     lower = -0.04
#     upper = 0.0
#     return upper + pct * (lower - upper)
# 
# 
# def {sanitized_tool_name}_urdf_to_subsystem(position, params):
#     \"\"\"
#     Convert URDF finger joint value (in meters/radians) to custom gripper subsystem units.
#     \"\"\"
#     # For custom parallel grippers, subsystem units are typically in meters.
#     return position
"""

GAMEPAD_TEMPLATE = """#!/usr/bin/env python3
from stretch4_body.core.robot_params import RobotParams

class CommandCustomToolPosition:
    \"\"\"
    Custom Tool motion command class for gamepad teleoperation.
    For this class, simple open and close methods are provided
    and expected only to be controlled on a button state.
    \"\"\"
    def __init__(self, motion_profile:str = 'max'):
        from stretch4_body.utils.stretch_pose_models import RobotJoints
        self.name = RobotJoints.gripper.value or '{sanitized_tool_name}'
        self.params = RobotParams().get_params()[1][self.name]
        self.gripper_step_m = 0.01
        self.gripper_accel = self.params.get('motion', {}).get(motion_profile, {}).get('accel', 6.0)
        self.gripper_vel = self.params.get('motion', {}).get(motion_profile, {}).get('vel', 6.0)
        self.precision_mode = 0.0
        self.stop_reqd = False

    def _move(self, dx_m, robot):
        scale = 1.0 - 0.75 * self.precision_mode
        dx_m = dx_m * scale
        robot.end_of_arm.move_by(self.name, dx_m, self.gripper_vel, self.gripper_accel)
        self.stop_reqd = True
    
    def open_gripper(self, robot):
        self._move(self.gripper_step_m, robot)
        
    def close_gripper(self, robot):
        self._move(-self.gripper_step_m, robot)

    def stop_gripper(self, robot):
        if self.stop_reqd:
            robot.end_of_arm.move_by(self.name, 0.0)
            self.stop_reqd = False
"""


PLACEHOLDER_URDF = '<?xml version="1.0"?>\n<robot name="tool">\n  <link name="quick_connect_interface_link" />\n</robot>\n'


def is_placeholder_urdf(urdf_file):
    """True if urdf_file still holds the untouched generated placeholder (ignoring whitespace)."""
    try:
        with open(urdf_file, 'r') as f:
            content = f.read()
    except Exception:
        return False
    return ''.join(content.split()) == ''.join(PLACEHOLDER_URDF.split())


def process_single_tool(tool_name, tool_path):
    print(f"\nProcessing user tool: {tool_name} (located in {tool_path})")
    os.makedirs(os.path.join(tool_path, 'meshes'), exist_ok=True)

    # Sanitized tool name for Python module and file names
    sanitized_tool_name = re.sub(r'[^a-zA-Z0-9_]', '_', tool_name)
    if sanitized_tool_name and sanitized_tool_name[0].isdigit():
        sanitized_tool_name = "_" + sanitized_tool_name

    # Generate standard PascalCase Python class names
    clean_class_base = re.sub(r'[^a-zA-Z0-9]', ' ', tool_name)
    if clean_class_base and clean_class_base[0].isdigit():
        clean_class_base = "Tool " + clean_class_base
    server_class_name = clean_class_base.title().replace(' ', '')
    client_class_name = server_class_name + "_Client"
    
    # 1. Generate templates if they do not exist
    tool_py_file = os.path.join(tool_path, "tool.py")
    end_of_arm_py_file = os.path.join(tool_path, "end_of_arm.py")
    client_py_file = os.path.join(tool_path, "client.py")
    command_group_py_file = os.path.join(tool_path, "command_group.py")
    gamepad_py_file = os.path.join(tool_path, "gamepad.py")
    collision_py_file = os.path.join(tool_path, "collision.py")
    tool_urdf_file = os.path.join(tool_path, "tool.urdf")
    pose_models_file = os.path.join(tool_path, "pose_models.yaml")
    
    if (not os.path.exists(tool_py_file) and
        not os.path.exists(end_of_arm_py_file) and
        not os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}.py")) and
        not os.path.exists(os.path.join(tool_path, f"{tool_name}.py"))):
        print(f"Generating custom FeetechSMHello driver template at: {tool_py_file}")
        try:
            with open(tool_py_file, 'w') as f:
                f.write(CUSTOM_TOOL_TEMPLATE.format(gripper_class_name=derive_gripper_class_name(server_class_name), tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate custom FeetechSMHello template: {e}")
            
        print(f"Generating custom EndOfArm driver template at: {end_of_arm_py_file}")
        try:
            with open(end_of_arm_py_file, 'w') as f:
                f.write(CUSTOM_TOOL_END_OF_ARM_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate custom EndOfArm template: {e}")
            
    if not os.path.exists(client_py_file) and not os.path.exists(os.path.join(tool_path, "tool_client.py")) and not os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}_client.py")):
        print(f"Generating client-side python driver template at: {client_py_file}")
        try:
            with open(client_py_file, 'w') as f:
                f.write(CLIENT_TEMPLATE.format(client_class_name=client_class_name, tool_name=tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate client-side python template: {e}")

    if not os.path.exists(command_group_py_file) and not os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}_command_group.py")):
        print(f"Generating ROS CommandGroup template at: {command_group_py_file}")
        try:
            with open(command_group_py_file, 'w') as f:
                f.write(COMMAND_GROUP_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate CommandGroup template: {e}")

    if not os.path.exists(gamepad_py_file) and not os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}_gamepad.py")):
        print(f"Generating GamepadTeleop template at: {gamepad_py_file}")
        try:
            with open(gamepad_py_file, 'w') as f:
                f.write(GAMEPAD_TELEOP_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate GamepadTeleop template: {e}")

    if not os.path.exists(collision_py_file) and not os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}_collision.py")):
        print(f"Generating Collision mapping template at: {collision_py_file}")
        try:
            with open(collision_py_file, 'w') as f:
                f.write(COLLISION_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate Collision template: {e}")

    if not glob.glob(os.path.join(tool_path, '*.urdf')):
        print(f"Generating placeholder tool URDF at: {tool_urdf_file}")
        try:
            with open(tool_urdf_file, 'w') as f:
                f.write(PLACEHOLDER_URDF)
        except Exception as e:
            print(f"Warning: Failed to generate placeholder URDF: {e}")

    gripper_conversion_file = os.path.join(tool_path, "gripper_conversion.py")
    if not os.path.exists(gripper_conversion_file):
        print(f"Generating gripper conversion logic template at: {gripper_conversion_file}")
        try:
            with open(gripper_conversion_file, 'w') as f:
                f.write(GRIPPER_CONVERSION_TEMPLATE.format(sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate gripper conversion template: {e}")

    if not os.path.exists(pose_models_file):
        print(f"Generating Pose Models template at: {pose_models_file}")
        try:
            with open(pose_models_file, 'w') as f:
                f.write(POSE_MODELS_TEMPLATE.format(sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate Pose Models template: {e}")

    # 2. Check for populated URDF files
    urdf_files = [f for f in glob.glob(os.path.join(tool_path, '*.urdf'))
                  if os.path.getsize(f) > 0
                  and not is_placeholder_urdf(f)]
    if not urdf_files:
        copied_readme = False
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            src_readme = os.path.abspath(os.path.join(script_dir, '../../SE4_tools/user_tool.md'))
            if os.path.exists(src_readme):
                dest_readme = os.path.join(tool_path, 'user_tool.md')
                shutil.copy(src_readme, dest_readme)
                copied_readme = True
                print(f"Copied custom tool documentation to: {dest_readme}")
        except Exception as e:
            print(f"Warning: Failed to copy user_tool.md: {e}")

        # Generate baseline collision_mesh_config.yaml (with root link placeholder)
        config_path = os.path.join(tool_path, 'collision_mesh_config.yaml')
        if not os.path.exists(config_path):
            try:
                with open(config_path, 'w') as f:
                    yaml.dump({
                        'links': {
                            'quick_connect_interface_link': {
                                'action': 'qem',
                                'simplification_ratio': 0.1
                            }
                        }
                    }, f, default_flow_style=False, sort_keys=False)
                print(f"Generated baseline collision_mesh_config.yaml at: {config_path}")
            except Exception as e:
                print(f"Warning: Failed to generate baseline collision_mesh_config.yaml: {e}")

        # Generate baseline tool_params.yaml
        try:
            save_tool_params(tool_name, server_class_name, "end_of_arm", client_class_name, "client", tool_path, gripper_module_name="tool")
        except Exception as e:
            print(f"Warning: Failed to generate baseline tool_params.yaml: {e}")

        print(f"\nCreated custom tool subdirectory at: {tool_path}")
        print("No populated URDF file was found in your tool directory.")
        print("\n================================================================================")
        print("NEXT STEPS:")
        if copied_readme:
            print(f"1. Read the newly created guide: {os.path.join(tool_path, 'user_tool.md')}")
        else:
            print("1. Read the guide 'user_tool.md' to know how to set up your tool structure.")
        print(f"2. Place your custom tool's visual/CAD meshes inside '{os.path.join(tool_path, 'meshes/')}'")
        print(f"3. Populate '{tool_urdf_file}' with your tool's kinematics")
        print(f"4. Re-run 'stretch_add_user_tool {tool_name}' to complete processing & registration.")
        print("================================================================================\n")
        return

    # Now detect server-side
    py_file = None
    module_name = 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools'
    class_name = 'EOA_Wrist_DW4_Tool_NIL'
    
    if os.path.exists(os.path.join(tool_path, "end_of_arm.py")):
        py_file = os.path.join(tool_path, "end_of_arm.py")
        module_name = "end_of_arm"
    elif os.path.exists(os.path.join(tool_path, "custom_tool_end_of_arm.py")):
        py_file = os.path.join(tool_path, "custom_tool_end_of_arm.py")
        module_name = "custom_tool_end_of_arm"
    elif os.path.exists(os.path.join(tool_path, f"{tool_name}.py")):
        py_file = os.path.join(tool_path, f"{tool_name}.py")
        module_name = tool_name
    elif os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}.py")):
        py_file = os.path.join(tool_path, f"{sanitized_tool_name}.py")
        module_name = sanitized_tool_name
    elif os.path.exists(os.path.join(tool_path, "tool.py")):
        py_file = os.path.join(tool_path, "tool.py")
        module_name = "tool"
        
    if py_file:
        try:
            with open(py_file, 'r') as f:
                tree = ast.parse(f.read())
            classes = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)]
            if classes:
                class_name = classes[0]
                print(f"Detected custom python class '{class_name}' in module '{module_name}' within user tool folder.")
            else:
                print(f"No classes found in python file {os.path.basename(py_file)}. Falling back to passive tool driver.")
        except Exception as e:
            print(f"Warning: Failed to parse Python class: {e}. Falling back to passive tool driver.")
            
    # Now detect client-side
    client_py = None
    client_module = None
    client_class = None
    
    if os.path.exists(os.path.join(tool_path, "client.py")):
        client_py = os.path.join(tool_path, "client.py")
        client_module = "client"
    elif os.path.exists(os.path.join(tool_path, f"{tool_name}_client.py")):
        client_py = os.path.join(tool_path, f"{tool_name}_client.py")
        client_module = f"{tool_name}_client"
    elif os.path.exists(os.path.join(tool_path, f"{sanitized_tool_name}_client.py")):
        client_py = os.path.join(tool_path, f"{sanitized_tool_name}_client.py")
        client_module = f"{sanitized_tool_name}_client"
    elif os.path.exists(os.path.join(tool_path, "tool_client.py")):
        client_py = os.path.join(tool_path, "tool_client.py")
        client_module = "tool_client"
        
    if client_py:
        try:
            with open(client_py, 'r') as f:
                tree = ast.parse(f.read())
            classes = [node.name for node in ast.walk(tree) if isinstance(node, ast.ClassDef)]
            if classes:
                client_class = classes[0]
                print(f"Detected custom client python class '{client_class}' in module '{client_module}' within user tool folder.")
        except Exception as e:
            print(f"Warning: Failed to parse Python client class: {e}")
        


    # 2. Invoke URDF / Mesh preprocessing
    try:
        try:
            from stretch4_urdf.utils.preprocessing.process_new_tool import process_tool_urdf
        except ImportError:
            sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            from stretch4_urdf.utils.preprocessing.process_new_tool import process_tool_urdf
        # process_tool_urdf and the collision mesh generator scan the folder and expect
        # exactly one URDF, so clear out the ones that hold nothing: empty files and the
        # untouched generated placeholder
        populated = [os.path.abspath(f) for f in urdf_files]
        for unpopulated in glob.glob(os.path.join(tool_path, '*.urdf')):
            if os.path.abspath(unpopulated) not in populated:
                os.remove(unpopulated)
                print(f"Removed unpopulated URDF: {unpopulated}")

        if len(urdf_files) == 1:
            source_urdf = urdf_files[0]
        else:
            # tool.urdf already holds a previous run's result, so a second populated URDF
            # is the user replacing it
            replacements = [f for f in urdf_files
                            if os.path.abspath(f) != os.path.abspath(tool_urdf_file)]
            if len(replacements) != 1:
                print(f"Error: Expected one URDF in '{tool_path}', found {len(urdf_files)}: "
                      f"{', '.join(sorted(os.path.basename(f) for f in urdf_files))}")
                print("Leave only the URDF you want processed and re-run.")
                sys.exit(1)
            source_urdf = replacements[0]

        # The runtime loads a user tool's kinematics from tool.urdf, and processing
        # rewrites its input in place, so tool.urdf is what gets processed
        previous_urdf = None
        renamed_from = None
        if os.path.abspath(source_urdf) != os.path.abspath(tool_urdf_file):
            if os.path.exists(tool_urdf_file):
                previous_urdf = tool_urdf_file + '.prev'
                shutil.move(tool_urdf_file, previous_urdf)
            shutil.move(source_urdf, tool_urdf_file)
            renamed_from = source_urdf
            print(f"Installed '{os.path.basename(source_urdf)}' as: {tool_urdf_file}")
        try:
            if not process_tool_urdf(tool_path, tool_path):
                sys.exit(1)
            print(f"Processed URDF in place: {tool_urdf_file}")
        except BaseException:
            if renamed_from:
                shutil.move(tool_urdf_file, renamed_from)
            if previous_urdf and os.path.exists(previous_urdf):
                shutil.move(previous_urdf, tool_urdf_file)
                print(f"Processing failed; restored the previous {tool_urdf_file}")
            raise
        if previous_urdf and os.path.exists(previous_urdf):
            os.remove(previous_urdf)
    except Exception as e:
        print(f"Error during URDF and mesh preprocessing: {e}")
        sys.exit(1)

    # 3. Write baseline parameters and collision management to tool_params.yaml inside custom tool folder
    gripper_module = "tool" if module_name == "end_of_arm" else ("custom_tool" if module_name == "custom_tool_end_of_arm" else None)
    save_tool_params(tool_name, class_name, module_name, client_class, client_module, tool_path, gripper_module_name=gripper_module)
    
    # 4. Copy user_tool.md template to user tool directory
    try:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        src_readme = os.path.abspath(os.path.join(script_dir, '../../SE4_tools/user_tool.md'))
        if os.path.exists(src_readme):
            shutil.copy(src_readme, os.path.join(tool_path, 'user_tool.md'))
            print(f"Copied custom tool documentation to: {os.path.join(tool_path, 'user_tool.md')}")
    except Exception as e:
        print(f"Warning: Failed to copy user_tool.md: {e}")

    print(f"Done processing custom tool '{tool_name}'!")


if __name__ == '__main__':
    main()
