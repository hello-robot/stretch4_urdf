#!/usr/bin/env python3

"""This script generates Factory tool to generate URDFs from Xacros"""

import glob
import importlib.resources as importlib_resources
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

from stretch4_urdf.utils.preprocessing.update_urdf_with_collision_mesh_filepath import (
    remove_collision_from_optical_links, update_urdf_collision_meshes)
from stretch4_urdf.utils.preprocessing.process_new_robot_model import update_urdf_joint_limits


def create_collision_config_if_missing(base_urdf, root_dir):
    config_path = os.path.join(root_dir, 'collision_mesh_config.yaml')
    if not os.path.exists(config_path):
        print(f"Creating default collision_mesh_config.yaml in {root_dir}")
        tree = ET.parse(base_urdf)
        root = tree.getroot()
                        
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
    print(f"Generating collision meshes for tool: {model_name}...")
    gen_script = os.path.join(os.path.dirname(__file__), 'generate_collision_mesh.py')
    try:
        subprocess.run(['python3', gen_script, '--model', model_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error generating collision meshes for {model_name}: {e}")


def process_tool_urdf(model_name, root_dir):
    # Find the base URDF file
    urdf_files = glob.glob(os.path.join(root_dir, '*.urdf'))
    
    if len(urdf_files) != 1:
        print(f"Error: Expected exactly one base URDF file in {root_dir}, found {len(urdf_files)}: {urdf_files}")
        return

    base_urdf = urdf_files[0]
    print(f"Found tool URDF: {base_urdf}")

    # Check that the root link is named quick_connect_interface
    tree = ET.parse(base_urdf)
    root = tree.getroot()
    links = [link.get('name') for link in root.findall('link')]
    child_links = {joint.find('child').get('link') for joint in root.findall('joint') if joint.find('child') is not None}
    root_links = [l for l in links if l not in child_links]
    
    if not root_links:
        sys.exit(f"Error: No root link found in URDF: {base_urdf}")
    
    root_link = root_links[0]
    if root_link != "quick_connect_interface_link":
        print(f"Issue: Root link in URDF '{base_urdf}' is '{root_link}', but it must be named 'quick_connect_interface_link'.")
        try:
            ans = input(f"Would you like to add 'quick_connect_interface_link' as the root link with a fixed identity joint to '{root_link}'? (y/N): ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            sys.exit(1)
        if ans == 'y':
            new_link = ET.Element('link')
            new_link.set('name', 'quick_connect_interface_link')
            root.append(new_link)
            
            new_joint = ET.Element('joint')
            new_joint.set('name', 'quick_connect_interface_joint')
            new_joint.set('type', 'fixed')
            
            parent_el = ET.SubElement(new_joint, 'parent')
            parent_el.set('link', 'quick_connect_interface_link')
            
            child_el = ET.SubElement(new_joint, 'child')
            child_el.set('link', root_link)
            
            origin_el = ET.SubElement(new_joint, 'origin')
            origin_el.set('xyz', '0 0 0')
            origin_el.set('rpy', '0 0 0')
            
            root.append(new_joint)
            
            # Format and write back to base_urdf
            ET.indent(tree, space="  ")
            tree.write(base_urdf, encoding='utf-8', xml_declaration=False)
            print(f"Successfully added 'quick_connect_interface' as the root link to {base_urdf}.")
        else:
            sys.exit(f"Error: Root link must be named 'quick_connect_interface', found '{root_link}' in {base_urdf}")

    create_collision_config_if_missing(base_urdf, root_dir)
    generate_collision_meshes(model_name)
 
    print('Updating the URDF with collision mesh filepaths...')
    update_urdf_collision_meshes(base_urdf, base_urdf)
    remove_collision_from_optical_links(base_urdf, base_urdf)
    update_urdf_joint_limits(base_urdf, base_urdf)

    # Convert mesh paths to use $(arg tool_mesh_dir)
    print(f'Converting mesh paths to use $(arg tool_mesh_dir)...')
    with open(base_urdf, 'r') as f:
        content = f.read()

    # Replace anything prepended to meshes/ in filename attributes with $(arg tool_mesh_dir)/
    content = re.sub(r'filename="[^"]*meshes/', 'filename="$(arg tool_mesh_dir)/', content)
    
    with open(base_urdf, 'w') as f:
        f.write(content)
        
    print(f'Successfully processed {base_urdf}')

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


def get_tools():
    try:
        urdf_pkg_path = str(importlib_resources.files("stretch4_urdf"))
        entries = os.listdir(urdf_pkg_path)
    except Exception:
        urdf_pkg_path = os.path.join(os.path.dirname(__file__), 'stretch4_urdf')
        entries = os.listdir(urdf_pkg_path)
        
    tools = []
    urdf_map = {}
    for entry in entries:
        full_path = os.path.join(urdf_pkg_path, entry)
        if os.path.isdir(full_path) and entry.endswith("_tools") and not entry.startswith("__"):
            tool_entries = os.listdir(full_path)
            for tool_entry in tool_entries:
                tool_path = os.path.join(full_path, tool_entry)
                if os.path.isdir(tool_path) and not tool_entry.startswith("__"):
                    tool_id = f"{entry}/{tool_entry}"
                    tools.append(tool_id)
                    urdfs = glob.glob(os.path.join(tool_path, "*.urdf"))
                    if len(urdfs) > 0:
                        urdf_map[tool_id] = urdfs
                
    return sorted(tools), urdf_map, urdf_pkg_path


def main(model_names_to_generate:list[str]|None = None):
    """The model names match the folder names under the `stretch_urdf/` directory."""
    all_models, urdf_map, _ = get_tools()
    
    # Descriptions should include all configurations that we officially "support"
    models_to_process = model_names_to_generate if model_names_to_generate else list(urdf_map.keys())
    
    for model_name in models_to_process:
        print('Generating tool URDF for', model_name)
        root_dir = './stretch4_urdf/' + model_name + '/'
        process_tool_urdf(model_name, root_dir)


if __name__ == '__main__':
    all_models, urdf_map, pkg_path = get_tools()

    # Interactive section
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
                        print(f"  -> Skipping '{selected}' as a .urdf file was not found in the model directory. A urdf file is required for processing.")
                else:
                    print(f"  -> Invalid index: {idx+1}")
            except ValueError:
                print(f"  -> Invalid input: {part}")
                
    if not model_names_to_generate:
        print("No valid robot model directories selected. Please add a .urdf file to the model directory. Typically, this is exported from CAD software.")
        sys.exit(0)
        
    main(model_names_to_generate)
