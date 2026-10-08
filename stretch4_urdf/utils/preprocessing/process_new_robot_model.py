#!/usr/bin/env python3
import glob
import importlib.resources as importlib_resources
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

from stretch4_urdf.utils.preprocessing.update_urdf_with_collision_mesh_filepath import (
    remove_collision_from_optical_links, update_urdf_collision_meshes)



def _get_joint_to_tool_param_key_map(robot_params, tool_name=None):
    """
    Returns a map from the joint names for a tool to the key in the robot parameter dictionary that holds
    the motion/effort limits, e.g. {"finger_left_joint": "parallel_gripper", "finger_right_joint": "parallel_gripper"}.

    This is necessary because built-in tools and their aliases, (e.g. "eoa_wrist_dw4_tool_pg4") key their motion/effort
    params under a canonical name ("parallel_gripper") that can differ from tool_name

    Custom/user tools declare their own joints directly in robot_params[tool_name]["tool_joints"]
    (see LinearToolMetadata), so no per-tool special-casing is needed here.
    """
    try:
        # import here to avoid circular dependencies
        from stretch4_body.utils.tool_metadata import (
            BUILTIN_TOOL_MODELS,
            ToolConfigurationError,
            get_tool_metadata,
        )
    except Exception as e:
        print(f"Tool metadata unavailable, falling back to joint-name heuristics: {e}")
        return {}, tool_name

    if tool_name:
        try:
            tool_meta = get_tool_metadata(tool_name)
        except ToolConfigurationError as e:
            print(
                f"Tool metadata incomplete for '{tool_name}' ({e}); using '{tool_name}' as the "
                "robot_params key directly for every joint in its URDF."
            )
            return {}, tool_name

        param_key = (
            BUILTIN_TOOL_MODELS[tool_name].joint_name
            if tool_name in BUILTIN_TOOL_MODELS
            else tool_name
        )
        return {joint: param_key for joint in tool_meta.tool_joints}, tool_name

    joint_to_param_key = {}

    for param_key, tool_meta in BUILTIN_TOOL_MODELS.items():
        if tool_meta.joint_name != param_key:
            continue
        for joint in tool_meta.tool_joints:
            joint_to_param_key[joint] = param_key


    for param_key, params in robot_params.items():
        if isinstance(params, dict) and params.get("tool_joints"):
            for joint in params["tool_joints"]:
                joint_to_param_key[joint] = param_key

    return joint_to_param_key, None


def update_urdf_joint_limits(input_file, output_file, tool_name=None):
    print("Updating URDF joint limits from robot parameters...")
    try:
        # avoid a global stretch4_body dependency in stretch4_urdf
        from stretch4_body.core.robot_params import RobotParams

        _, robot_params = RobotParams.get_params()
    except Exception as e:
        print(f"Failed to fetch robot parameters for limits: {e}")
        return

    joint_to_tool_param_key, default_tool_param_key = _get_joint_to_tool_param_key_map(
        robot_params, tool_name=tool_name
    )

    tree = ET.parse(input_file)
    root = tree.getroot()

    for joint in root.findall("joint"):
        if joint.get("type") == "fixed":
            limit = joint.find("limit")
            if limit is not None and not limit.attrib:
                joint.remove(limit)
            continue

        limit = joint.find("limit")
        is_vel_zero = False
        is_eff_zero = False
        new_limit = False

        if limit is not None:
            vel = limit.get("velocity")
            eff = limit.get("effort")

            try:
                is_vel_zero = float(vel) == 0.0
            except (ValueError, TypeError):
                is_vel_zero = False

            try:
                is_eff_zero = float(eff) == 0.0
            except (ValueError, TypeError):
                is_eff_zero = False
        else:
            limit = ET.Element("limit")
            new_limit = True
            is_vel_zero = True
            is_eff_zero = True

        if is_vel_zero or is_eff_zero:
            joint_name = joint.get("name")
            if joint_name is None:
                continue
            param_key = joint_name.replace("_joint", "")
            if joint_name in joint_to_tool_param_key:
                param_key = joint_to_tool_param_key[joint_name]
            elif "arm" in param_key:
                param_key = "arm"
            elif "lift" in param_key:
                param_key = "lift"
            elif "head" in param_key:
                param_key = "head"
            elif default_tool_param_key:
                param_key = default_tool_param_key

            if param_key in robot_params:
                p = robot_params[param_key]
                device_params = p.get("devices", {}).get(param_key, {})
                motion = p.get("motion", device_params.get("motion"))
                stall_max_effort = p.get(
                    "stall_max_effort", device_params.get("stall_max_effort")
                )

                max_vel = None
                max_eff = None

                if is_vel_zero and motion and "max" in motion:
                    max_vel = motion["max"].get("vel", motion["max"].get("vel_m"))

                if is_eff_zero and stall_max_effort is not None:
                    max_eff = stall_max_effort

                if max_vel is not None:
                    limit.set("velocity", str(max_vel))
                if max_eff is not None:
                    limit.set("effort", str(max_eff))

                if max_vel is not None or max_eff is not None:
                    print(
                        f"Updated {joint_name} limits from params: vel={max_vel if is_vel_zero else 'unchanged'}, effort={max_eff if is_eff_zero else 'unchanged'}"
                    )

        if new_limit:
            if "velocity" in limit.attrib and "effort" in limit.attrib:
                joint.append(limit)
            elif "velocity" in limit.attrib or "effort" in limit.attrib:
                if "velocity" not in limit.attrib:
                    limit.set("velocity", "0")
                if "effort" not in limit.attrib:
                    limit.set("effort", "0")
                joint.append(limit)
        else:
            if not limit.attrib:
                joint.remove(limit)

    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(output_file, encoding="utf-8", xml_declaration=False)
    print(f"Updated URDF limits saved to: {output_file}")


def create_collision_config_if_missing(base_urdf, root_dir):
    config_path = os.path.join(root_dir, 'collision_mesh_config.yaml')
    if not os.path.exists(config_path):
        print(f"Creating default collision_mesh_config.yaml in {root_dir}")
        tree = ET.parse(base_urdf)
        root = tree.getroot()
        
        # Ensure suffix naming 
        for tag in root.iter():
            for attr in ['name', 'link']:
                val = tag.get(attr)
                if val:
                    if val.startswith('link_'):
                        tag.set(attr, val[5:] + '_link')
                    elif val.startswith('joint_'):
                        tag.set(attr, val[6:] + '_joint')
                        
        links_dict = {}
        for link in root.findall('link'):
            link_name = link.get('name')
            visual = link.find('visual')
            if visual is not None and 'optical' not in link_name.lower():
                geom = visual.find('geometry')
                if geom is not None and geom.find('mesh') is not None:
                    links_dict[link_name] = {'action': 'qem', 'simplification_ratio': 0.1}
        with open(config_path, 'w') as f:
            yaml.dump({'links': links_dict}, f, default_flow_style=False, sort_keys=False)

def generate_collision_meshes(model_name):
    print(f"Generating collision meshes for model: {model_name}...")
    gen_script = os.path.join(os.path.dirname(__file__), 'generate_collision_mesh.py')
    try:
        subprocess.run([sys.executable, gen_script, '--model', model_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error generating collision meshes for {model_name}: {e}")

def remove_visual_and_collision_from_sensors_in_base_and_head(urdf_path):
    print('Removing visual and collision tags from sensor frames in base and head links...')
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    
    # Map joints to find parent-child relationships
    child_to_parent = {}
    for joint in root.findall('joint'):
        parent_el = joint.find('parent')
        child_el = joint.find('child')
        if parent_el is not None and child_el is not None:
            parent = parent_el.get('link')
            child = child_el.get('link')
            child_to_parent[child] = parent
            
    def is_descendant_of(child, target_parents):
        curr = child
        while curr in child_to_parent:
            parent = child_to_parent[curr]
            if parent in target_parents:
                return True
            curr = parent
        return False

    # Assemblies to include
    target_assemblies = ['base_link', 'link_head', 'head_link', 'head_pan_link', 'head_tilt_link']
    # Assemblies to exclude (specifically the arm to avoid removing gripper camera visual)
    exclude_assemblies = ['link_mast', 'mast_link', 'link_lift', 'lift_link', 'link_arm_l4', 'arm_l4_link']
    
    sensor_keywords = ['camera', 'lidar', 'line_sensor', 'imu']
    
    for link in root.findall('link'):
        link_name = link.get('name', '')
        link_name_lower = link_name.lower()
        if any(kw in link_name_lower for kw in sensor_keywords):
            # Check if it's in the target assemblies and NOT in the excluded ones
            if is_descendant_of(link_name, target_assemblies) and not is_descendant_of(link_name, exclude_assemblies):
                visual = link.find('visual')
                collision = link.find('collision')
                if visual is not None:
                    print(f"  -> Removing visual from sensor link: {link_name}")
                    link.remove(visual)
                if collision is not None:
                    print(f"  -> Removing collision from sensor link: {link_name}")
                    link.remove(collision)
                     
    tree.write(urdf_path)

def finalize_xacro_and_cleanup_meshes(stretch_main_xacro, model_name, root_dir, content):
    import re
    print(f'Converting mesh paths to use $(arg model_mesh_dir)...')
    
    # Replace <robot name="..."> with <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="stretch">
    content = re.sub(r'<robot[^>]*name="[^"]+"[^>]*>', '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="stretch">', content)
    
    # Replace anything prepended to meshes/ in filename attributes with $(arg model_mesh_dir)/
    content = re.sub(r'filename="[^"]*meshes/', 'filename="$(arg model_mesh_dir)/', content)
    
    with open(stretch_main_xacro, 'w') as f:
        f.write(content)
        
    print(f'Successfully generated {stretch_main_xacro}')

    # Check for unreferenced meshes
    referenced_meshes = set()
    for match in re.finditer(r'filename="([^"]+)"', content):
        referenced_meshes.add(os.path.basename(match.group(1)))

    meshes_dir = os.path.join(root_dir, 'meshes')
    if os.path.exists(meshes_dir):
        all_meshes = [f for f in os.listdir(meshes_dir) if f.lower().endswith(('.stl', '.dae', '.obj'))]
        unreferenced = [os.path.join(meshes_dir, m) for m in all_meshes if m not in referenced_meshes]
        
        if unreferenced:
            print(f"\nThe following meshes exist in the 'meshes' folder but are not used by {model_name}:")
            for m in unreferenced:
                print(f"  - {os.path.basename(m)}")
            try:
                ans = input("Delete these meshes? (y/N): ").strip().lower()
                if ans == 'y':
                    for m in unreferenced:
                        os.remove(m)
                    print(f"Deleted {len(unreferenced)} unreferenced meshes.")
            except (KeyboardInterrupt, EOFError):
                pass

def generate_xacro_from_base_urdf(model_name, root_dir, xacro_dir):
    # Find the base URDF file
    urdf_files = glob.glob(os.path.join(root_dir, '*.urdf'))
    xacro_files = glob.glob(os.path.join(xacro_dir, '*.xacro')) if os.path.exists(xacro_dir) else []
    
    if len(urdf_files) == 0 and len(xacro_files) > 0:
        # If there are multiple xacro files, try to prioritize stretch_main.xacro or model_name.xacro
        if len(xacro_files) == 1:
            target_xacro = xacro_files[0]
        else:
            matched = [f for f in xacro_files if os.path.basename(f) in ['stretch_main.xacro', f"{model_name}.xacro"]]
            if matched:
                target_xacro = matched[0]
            else:
                target_xacro = xacro_files[0]

        print(f"No base URDF file found, but found existing xacro: {target_xacro}. Using it directly.")
        create_collision_config_if_missing(target_xacro, root_dir)
        generate_collision_meshes(model_name)
        with open(target_xacro, 'r') as f:
            content = f.read()
        finalize_xacro_and_cleanup_meshes(target_xacro, model_name, root_dir, content)
        return

    if len(urdf_files) != 1:
        print(f"Error: Expected exactly one base URDF file in {root_dir}, found {len(urdf_files)}: {urdf_files}")
        return

    base_urdf = urdf_files[0]
    print(f"Found base URDF: {base_urdf}")

    create_collision_config_if_missing(base_urdf, root_dir)
    generate_collision_meshes(model_name)

    os.makedirs(xacro_dir, exist_ok=True)
    stretch_main_xacro = os.path.join(xacro_dir, 'stretch_main.xacro')
    shutil.copy(base_urdf, stretch_main_xacro)

    print('Updating the URDF with collision mesh filepaths...')
    temp_urdf = os.path.join(os.path.dirname(base_urdf), 'temp.urdf')
    shutil.copy(stretch_main_xacro, temp_urdf)
    update_urdf_collision_meshes(temp_urdf, temp_urdf)
    shutil.copy(temp_urdf, stretch_main_xacro)
    os.remove(temp_urdf)
    remove_collision_from_optical_links(stretch_main_xacro, stretch_main_xacro)
    update_urdf_joint_limits(stretch_main_xacro, stretch_main_xacro)

    # Remove visual tags from sensors in base and head
    remove_visual_and_collision_from_sensors_in_base_and_head(stretch_main_xacro)

    with open(stretch_main_xacro, 'r') as f:
        content = f.read()
    finalize_xacro_and_cleanup_meshes(stretch_main_xacro, model_name, root_dir, content)


def get_all_model_names():
    try:
        urdf_pkg_path = str(importlib_resources.files("stretch4_urdf"))
        entries = os.listdir(urdf_pkg_path)
    except Exception:
        urdf_pkg_path = os.path.join(os.path.dirname(__file__), 'stretch4_urdf')
        entries = os.listdir(urdf_pkg_path)
        
    models = []
    urdf_map = {}
    for entry in entries:
        full_path = os.path.join(urdf_pkg_path, entry)
        if os.path.isdir(full_path) and not entry.startswith("__") and not entry.endswith("_tools") and entry not in ["tools", "utils", "SE4_accessories"]:
            models.append(entry)
            urdfs = glob.glob(os.path.join(full_path, "*.urdf"))
            if len(urdfs) > 0:
                urdf_map[entry] = urdfs
            else:
                xacros = glob.glob(os.path.join(full_path, "xacro", "*.xacro"))
                if len(xacros) > 0:
                    urdf_map[entry] = xacros
                
    return sorted(models), urdf_map, urdf_pkg_path


def main(model_names_to_generate:list[str]|None = None):
    all_models, urdf_map, pkg_path = get_all_model_names()
    models_to_process = model_names_to_generate if model_names_to_generate else list(urdf_map.keys())

    for model_name in models_to_process:
        print('Generating URDF for', model_name)
        root_dir = os.path.join(pkg_path, model_name) + '/'
        xacro_dir = root_dir + 'xacro/'
        generate_xacro_from_base_urdf(model_name, root_dir, xacro_dir)


if __name__ == '__main__':
    all_models, urdf_map, pkg_path = get_all_model_names()

    print(f"Searching in: {pkg_path}")
    print("Existing robot model directories in stretch4_urdf:")
    
    for i, m in enumerate(all_models):
        has_model_source = m in urdf_map
        if has_model_source:
            is_xacro = any(f.endswith('.xacro') for f in urdf_map[m])
            if is_xacro:
                model_str = "(existing .xacro available for processing)"
            else:
                model_str = "(raw .urdf available for processing)"
        else:
            model_str = ""
        print(f"  {i+1}: {m} {model_str}")
        
    print(f"\nSelect a robot model directory to process (comma-separated indices e.g., '1, 3', or 'all'):")
    try:
        choice = input("> ").strip().lower()
    except KeyboardInterrupt:
        print("\nExiting.")
        sys.exit(0)
        
    if not choice:
        print("No selection made. Exiting.")
        sys.exit(0)
        
    if choice == 'all':
        model_names_to_generate = list(urdf_map.keys())
    else:
        model_names_to_generate = []
        for part in choice.split(','):
            try:
                idx = int(part.strip()) - 1
                if 0 <= idx < len(all_models):
                    selected = all_models[idx]
                    if selected in urdf_map:
                        if selected not in model_names_to_generate:
                            model_names_to_generate.append(selected)
                    else:
                        print(f"  -> Skipping '{selected}' as a .urdf or .xacro file was not found.")
                else:
                    print(f"  -> Invalid index: {idx+1}")
            except ValueError:
                print(f"  -> Invalid input: {part}")
                
    if not model_names_to_generate:
        print("No valid robot model directories selected.")
        sys.exit(0)
        
    main(model_names_to_generate)
