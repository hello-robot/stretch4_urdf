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
        'wrist': 'eoaw_dw4',
        'tool': 'eoat_nil',
        'stow': {
            'arm': 0.0,
            'lift': 0.15,
            'wrist_pitch': 0.0,
            'wrist_roll': 0.0,
            'wrist_yaw': 3.14
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
        if selected_tool not in subdir_paths:
            print(f"Error: Custom tool subdirectory '{selected_tool}' does not exist inside scanned directories: {user_tools_dirs}.")
            sys.exit(1)
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
        
        # Safely actuate custom parallel_gripper if it is registered in devices
        if 'parallel_gripper' in self.motors:
            self.move_to('parallel_gripper', self.params['stow']['parallel_gripper'])

    def home(self, wait_on_completion=True):
        def _do_home():
            self.logger.info(f'Homing Custom Tool: {{self.name}}')
            self.status['is_homing'] = True
            
            # Home the general wrist pitch, roll, and yaw joints
            success = home_dw4_joints(self)
            
            # Safely home the custom gripper motor if registered
            if 'parallel_gripper' in self.motors:
                success = success and self.motors['parallel_gripper'].home(end_pos=0)
                
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


def process_single_tool(tool_name, tool_path):
    print(f"\nProcessing user tool: {tool_name} (located in {tool_path})")
    
    # 1. Generate templates if they do not exist
    server_class_name = tool_name.replace('_', ' ').title().replace(' ', '')
    client_class_name = server_class_name + "_Client"
    
    server_py_file = os.path.join(tool_path, f"{tool_name}.py")
    client_py_file = os.path.join(tool_path, f"{tool_name}_client.py")
    
    if not os.path.exists(server_py_file) and not os.path.exists(os.path.join(tool_path, "tool.py")):
        print(f"Generating server-side python driver template at: {server_py_file}")
        try:
            with open(server_py_file, 'w') as f:
                f.write(SERVER_TEMPLATE.format(class_name=server_class_name, tool_name=tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate server-side python template: {e}")
            
    if not os.path.exists(client_py_file) and not os.path.exists(os.path.join(tool_path, "tool_client.py")):
        print(f"Generating client-side python driver template at: {client_py_file}")
        try:
            with open(client_py_file, 'w') as f:
                f.write(CLIENT_TEMPLATE.format(client_class_name=client_class_name, tool_name=tool_name))
        except Exception as e:
            print(f"Warning: Failed to generate client-side python template: {e}")

    # Now detect server-side
    py_file = None
    module_name = 'stretch4_body.subsystem.end_of_arm.end_of_arm_tools'
    class_name = 'EOA_Wrist_DW4_Tool_NIL'
    
    if os.path.exists(os.path.join(tool_path, f"{tool_name}.py")):
        py_file = os.path.join(tool_path, f"{tool_name}.py")
        module_name = tool_name
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
