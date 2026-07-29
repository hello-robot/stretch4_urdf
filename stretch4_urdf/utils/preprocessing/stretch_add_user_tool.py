#!/usr/bin/env python3

import os
import sys
import argparse
import ast
import yaml
import glob

# Ensure we can import from stretch4_urdf
try:
    from stretch4_urdf.utils.preprocessing.process_new_tool import process_tool_urdf
except ImportError:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    from stretch4_urdf.utils.preprocessing.process_new_tool import process_tool_urdf


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


def save_tool_params(tool_name, py_class_name, py_module_name, client_class_name=None, client_module_name=None, tool_path=None):
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
                'py_class_name': f'{py_class_name}Gripper',
                'py_module_name': py_module_name,
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
                    'py_module_name': py_class_name,
                    'py_class_name': f'{tool_name}CommandGroup'
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
    _fleet_id = os.environ.get('HELLO_FLEET_ID')
    if _fleet_path:
        if _fleet_id:
            _specific_dir = os.path.join(_fleet_path, _fleet_id, 'user_tools')
            if os.path.exists(_specific_dir):
                _dirs.append(_specific_dir)
        _shared_dir = os.path.join(_fleet_path, 'user_tools')
        if os.path.exists(_shared_dir):
            _dirs.append(_shared_dir)
    else:
        _default_dir = os.path.expanduser('~/stretch_user/user_tools')
        if os.path.exists(_default_dir):
            _dirs.append(_default_dir)
    return _dirs


def main():
    parser = argparse.ArgumentParser(description="Process a user-defined custom tool using environment variables or fallbacks.")
    parser.add_argument('tool_name', nargs='?', help="Name of the custom tool subfolder.")
    
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
        import re
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
                    print(f"Error: Custom tool subdirectory '{selected_tool}' does not exist inside scanned directories: {user_tools_dirs}.")
                    sys.exit(1)
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

    process_single_tool(selected_tool, tool_path)


SERVER_TEMPLATE = """#!/usr/bin/env python3
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


from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello

class {class_name}Gripper(FeetechSMHello):
    \"\"\"
    A completely custom parallel gripper driver subclassing FeetechSMHello directly.
    \"\"\"
    def __init__(self, chain=None, usb=None, name='{tool_name}', is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb, is_direct=is_direct)
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
        rt_val = gamepad_state.get('right_trigger', 0.0)
        if rt_val > 0.1:
            self.robot.end_of_arm.move_by('{sanitized_tool_name}', 0.01 * rt_val)
            
        # Example: Command incremental negative movement when Left Trigger is pressed
        lt_val = gamepad_state.get('left_trigger', 0.0)
        if lt_val > 0.1:
            self.robot.end_of_arm.move_by('{sanitized_tool_name}', -0.01 * lt_val)
"""

COLLISION_TEMPLATE = """#!/usr/bin/env python3
class {class_name}Collision:
    \"\"\"
    Collision joint state mapping template for {tool_name}.
    Maps custom joint state values to Mujoco visualizer joints.
    \"\"\"
    def __init__(self, robot=None):
        self.robot = robot

    def get_mujoco_joints(self, state):
        \"\"\"
        Given the raw robot status dictionary,
        return a dictionary mapping Mujoco joint names to their target positions.
        \"\"\"
        eoa = state.get('end_of_arm', {{}})
        
        # Look for parallel_gripper or the sanitized tool name within the end of arm status
        parallel_gripper = eoa.get('parallel_gripper') or eoa.get('{sanitized_tool_name}') or {{}}
        pos_mm = parallel_gripper.get('pos_mm', 0.0)
        
        from stretch4_body.subsystem.end_of_arm.gripper_conversion import parallel_gripper_pos_mm_to_urdf_m
        from stretch4_body.core.robot_params import RobotParams
        _, robot_params = RobotParams.get_params()
        pg_params = robot_params.get('parallel_gripper', {{}})
        joint_val = parallel_gripper_pos_mm_to_urdf_m(pos_mm, pg_params)
        
        return {{
            'finger_left_joint': joint_val,
            'finger_right_joint': joint_val
        }}
"""


def process_single_tool(tool_name, tool_path):
    print(f"\nProcessing user tool: {tool_name} (located in {tool_path})")
    
    # Sanitized tool name for Python module and file names
    import re
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
    server_py_file = os.path.join(tool_path, f"{sanitized_tool_name}.py")
    client_py_file = os.path.join(tool_path, f"{sanitized_tool_name}_client.py")
    command_group_py_file = os.path.join(tool_path, f"{sanitized_tool_name}_command_group.py")
    gamepad_py_file = os.path.join(tool_path, f"{sanitized_tool_name}_gamepad.py")
    collision_py_file = os.path.join(tool_path, f"{sanitized_tool_name}_collision.py")
    
    if not os.path.exists(server_py_file) and not os.path.exists(os.path.join(tool_path, "tool.py")):
        print(f"Generating server-side python driver template at: {server_py_file}")
        try:
            with open(server_py_file, 'w') as f:
                f.write(SERVER_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate server-side python template: {e}")
            
    if not os.path.exists(client_py_file) and not os.path.exists(os.path.join(tool_path, "tool_client.py")):
        print(f"Generating client-side python driver template at: {client_py_file}")
        try:
            with open(client_py_file, 'w') as f:
                f.write(CLIENT_TEMPLATE.format(client_class_name=client_class_name, tool_name=tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate client-side python template: {e}")

    if not os.path.exists(command_group_py_file):
        print(f"Generating ROS CommandGroup template at: {command_group_py_file}")
        try:
            with open(command_group_py_file, 'w') as f:
                f.write(COMMAND_GROUP_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate CommandGroup template: {e}")

    if not os.path.exists(gamepad_py_file):
        print(f"Generating GamepadTeleop template at: {gamepad_py_file}")
        try:
            with open(gamepad_py_file, 'w') as f:
                f.write(GAMEPAD_TELEOP_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate GamepadTeleop template: {e}")

    if not os.path.exists(collision_py_file):
        print(f"Generating Collision mapping template at: {collision_py_file}")
        try:
            with open(collision_py_file, 'w') as f:
                f.write(COLLISION_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name, sanitized_tool_name=sanitized_tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate Collision template: {e}")

    # Check if a URDF file exists
    urdf_files = glob.glob(os.path.join(tool_path, '*.urdf'))
    if not urdf_files:
        copied_readme = False
        try:
            import shutil
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
            save_tool_params(tool_name, server_class_name, sanitized_tool_name, client_class_name, f"{sanitized_tool_name}_client", tool_path)
        except Exception as e:
            print(f"Warning: Failed to generate baseline tool_params.yaml: {e}")

        print(f"\nCreated custom tool subdirectory at: {tool_path}")
        print("No URDF file was found in your tool directory.")
        print("\n================================================================================")
        print("NEXT STEPS:")
        if copied_readme:
            print(f"1. Read the newly created guide: {os.path.join(tool_path, 'user_tool.md')}")
        else:
            print("1. Read the guide 'user_tool.md' to know how to set up your tool structure.")
        print(f"2. Place your custom tool's visual/CAD meshes inside '{os.path.join(tool_path, 'meshes/')}'")
        print(f"3. Place your CAD/visual URDF file (ending in .urdf) directly in '{tool_path}/'")
        print(f"4. Re-run 'stretch_add_user_tool {tool_name}' to complete processing & registration.")
        print("================================================================================\n")
        return

    # Now detect server-side
    py_file = None
    module_name = 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools'
    class_name = 'EOA_Wrist_DW4_Tool_NIL'
    
    if os.path.exists(os.path.join(tool_path, f"{tool_name}.py")):
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
    
    if os.path.exists(os.path.join(tool_path, f"{tool_name}_client.py")):
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
        process_tool_urdf(tool_path, tool_path)
    except Exception as e:
        print(f"Error during URDF and mesh preprocessing: {e}")
        sys.exit(1)

    # 3. Write baseline parameters and collision management to tool_params.yaml inside custom tool folder
    save_tool_params(tool_name, class_name, module_name, client_class, client_module, tool_path)
    
    # 4. Copy user_tool.md template to user tool directory
    try:
        import shutil
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
