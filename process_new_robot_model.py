#!/usr/bin/env python3
import glob
import importlib.resources as importlib_resources
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

from stretch4_urdf.utils.update_urdf_with_collision_mesh_filepath import (
    remove_collision_from_optical_links, update_urdf_collision_meshes)


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
    gen_script = os.path.join(os.path.dirname(__file__), 'stretch4_urdf', 'utils', 'generate_collision_mesh.py')
    try:
        subprocess.run(['python3', gen_script, '--model', model_name], check=True)
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
                if 'lidar' not in link_name_lower and collision is not None:
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
        if os.path.isdir(full_path) and not entry.startswith("__") and not entry.endswith("_tools") and entry not in ["tools", "utils"]:
            models.append(entry)
            urdfs = glob.glob(os.path.join(full_path, "*.urdf"))
            if len(urdfs) > 0:
                urdf_map[entry] = urdfs
                
    return sorted(models), urdf_map, urdf_pkg_path


def main(model_names_to_generate:list[str]|None = None):
    all_models, urdf_map, _ = get_all_model_names()
    models_to_process = model_names_to_generate if model_names_to_generate else list(urdf_map.keys())
    
    for model_name in models_to_process:
        print('Generating URDF for', model_name)
        root_dir = './stretch4_urdf/' + model_name + '/'
        xacro_dir = root_dir + 'xacro/'
        generate_xacro_from_base_urdf(model_name, root_dir, xacro_dir)


if __name__ == '__main__':
    all_models, urdf_map, pkg_path = get_all_model_names()

    print(f"Searching in: {pkg_path}")
    print("Existing robot model directories in stretch4_urdf:")
    
    for i, m in enumerate(all_models):
        has_urdf = m in urdf_map
        urdf_str = "(raw .urdf available for processing)" if has_urdf else ""
        print(f"  {i+1}: {m} {urdf_str}")
        
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
                        print(f"  -> Skipping '{selected}' as a .urdf file was not found.")
                else:
                    print(f"  -> Invalid index: {idx+1}")
            except ValueError:
                print(f"  -> Invalid input: {part}")
                
    if not model_names_to_generate:
        print("No valid robot model directories selected.")
        sys.exit(0)
        
    main(model_names_to_generate)
