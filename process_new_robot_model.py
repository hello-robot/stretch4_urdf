#!/usr/bin/env python3
import glob
import importlib.resources as importlib_resources
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from stretch4_urdf.utils.update_urdf_with_collision_mesh_filepath import (
    remove_collision_from_optical_links, update_urdf_collision_meshes)


def rpy_to_matrix(r, p, y):
    cx, sx = np.cos(r), np.sin(r)
    cy, sy = np.cos(p), np.sin(p)
    cz, sz = np.cos(y), np.sin(y)

    R = [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy]
    ]
    return np.array(R)

def matrix_to_rpy(R):
    sy = np.sqrt(R[0][0]**2 + R[1][0]**2)
    singular = sy < 1e-6
    if not singular:
        x = np.atan2(R[2][1], R[2][2])
        y = np.atan2(-R[2][0], sy)
        z = np.atan2(R[1][0], R[0][0])
    else:
        x = np.atan2(-R[1][2], R[1][1])
        y = np.atan2(-R[2][0], sy)
        z = 0
    return x, y, z


def clean(vals): 
    return " ".join(f"{0.0 if abs(v) < 1e-6 else v:.6g}" for v in vals)

def get_abs_poses(root):
    tree_map = {}
    for joint in root.findall('joint'):
        
        parent = joint.find('parent')
        child = joint.find('child')
        origin = joint.find('origin')
        
        if parent is not None and child is not None:
        
            parent_link = parent.get('link')
            child_link = child.get('link')
        
            rpy = [0.0, 0.0, 0.0]
            xyz = [0.0, 0.0, 0.0]
            if origin is not None:
                if origin.get('rpy'):
                    rpy = [float(x) for x in origin.get('rpy').split()]
                if origin.get('xyz'):
                    xyz = [float(x) for x in origin.get('xyz').split()]
        
            R_parent_to_child = rpy_to_matrix(*rpy)
            t_parent_to_child = np.array(xyz)

            if parent_link not in tree_map:
                tree_map[parent_link] = []
            tree_map[parent_link].append((child_link, R_parent_to_child, t_parent_to_child, joint))

    all_children = set()
    for children in tree_map.values():
        for child, _, _, _ in children:
            all_children.add(child)
            
    root_links = [parent for parent in tree_map.keys() if parent not in all_children]
    if len(root_links) != 1:
        raise ValueError(f"Found {len(root_links)} root links, expected 1: {root_links}")
    root_link = root_links[0]
    
    R_original_root_to_link_dict = {root_link: np.eye(3)}
    t_original_root_to_link_dict = {root_link: [0.0, 0.0, 0.0]}
    queue = [root_link]
    
    R_original_root_to_joint_dict = {}
    while queue:
        link_name = queue.pop(0)
        R_root_to_parent = R_original_root_to_link_dict[link_name]
        t_root_to_parent = t_original_root_to_link_dict[link_name]
        if link_name in tree_map:
            for child, R_parent_to_child, t_parent_to_child, joint in tree_map[link_name]:
                R_root_to_child = R_root_to_parent @ R_parent_to_child
                R_original_root_to_link_dict[child] = R_root_to_child
                R_original_root_to_joint_dict[joint.get('name')] = R_root_to_child
                
                t_root_to_child = (np.array(t_root_to_parent) + R_root_to_parent @ t_parent_to_child).tolist()
                t_original_root_to_link_dict[child] = t_root_to_child
                
                queue.append(child)
                
    return R_original_root_to_joint_dict, R_original_root_to_link_dict, t_original_root_to_link_dict


def create_collision_config_if_missing(base_urdf, root_dir):
    config_path = os.path.join(root_dir, 'collision_mesh_config.yaml')
    if not os.path.exists(config_path):
        print(f"Creating default collision_mesh_config.yaml in {root_dir}")
        tree = ET.parse(base_urdf)
        root = tree.getroot()
        
        # Rule 7: Enforce suffix naming early 
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

def apply_rule2_proximal_distal_arm_naming(root):
    rule5_arm_renamed = False
    for j in root.findall('joint'):
        if j.get('name') in ['joint_arm_l4', 'arm_l4_joint']:
            pt = j.find('parent')
            if pt is not None and pt.get('link') == 'lift_link':
                rule5_arm_renamed = True
                break
                
    if rule5_arm_renamed:
        arm_indices = [0, 1, 2, 3, 4]
        for tag in root.findall('.//'):
            for attr in ['name', 'link']:
                val = tag.get(attr)
                if val and 'arm_l' in val:
                    for idx in arm_indices:
                        if f'arm_l{idx}' in val:
                            tag.set(attr, val.replace(f'arm_l{idx}', f'arm_T{idx}'))
                            break
        for tag in root.findall('.//'):
            for attr in ['name', 'link']:
                val = tag.get(attr)
                if val and 'arm_T' in val:
                    for idx in arm_indices:
                        if f'arm_T{idx}' in val:
                            tag.set(attr, val.replace(f'arm_T{idx}', f'arm_l{4-idx}'))
                            break
    return rule5_arm_renamed

def apply_rule6_wheel_renaming(root, P_root_to_original_link):
    wheels = []
    for joint in root.findall('joint'):
        jname = joint.get('name')
        jtype = joint.get('type')
        if jname and 'wheel' in jname.lower() and jtype in ['continuous', 'revolute']:
            child = joint.find('child')
            if child is not None:
                c_link = child.get('link')
                pos = P_root_to_original_link.get(c_link, [0, 0, 0])
                wheels.append({'name': jname, 'link': c_link, 'pos': pos})

    rule6_wheels_renamed = False
    if wheels:
        for w in wheels:
            w['theta'] = np.atan2(w['pos'][1], w['pos'][0])
            
        def ang_diff(a1, a2):
            diff = a1 - a2
            while diff > np.pi: diff -= 2*np.pi
            while diff < -np.pi: diff += 2*np.pi
            return diff
            
        closest_wheel = min(wheels, key=lambda w: abs(ang_diff(w['theta'], 0)))
        start_theta = closest_wheel['theta']
        
        def counter_clockwise_dist(w):
            d = w['theta'] - start_theta
            while d < 0: d += 2*np.pi
            while d >= 2*np.pi: d -= 2*np.pi
            return d
            
        wheels_sorted = sorted(wheels, key=counter_clockwise_dist)
        
        wheel_map = {}
        import re
        for i, w in enumerate(wheels_sorted):
            m = re.search(r'wheel_(\d+)', w['name'])
            old_idx = m.group(1) if m else w['name']
            if old_idx != str(i):
                rule6_wheels_renamed = True
                
            new_jname = w['name']
            new_lname = w['link']
            if m:
                new_jname = w['name'].replace(f'wheel_{old_idx}', f'wheel_{i}')
            elif 'wheel' in w['name'].lower():
                new_jname = w['name'].replace('wheel', f'wheel_{i}')
                
            m_link = re.search(r'wheel_(\d+)', w['link'])
            if m_link:
                new_lname = w['link'].replace(f'wheel_{m_link.group(1)}', f'wheel_{i}')
            elif 'wheel' in w['link'].lower():
                new_lname = w['link'].replace('wheel', f'wheel_{i}')
                
            wheel_map[w['name']] = new_jname
            wheel_map[w['link']] = new_lname
            
        if rule6_wheels_renamed:
            for w, w_new in wheel_map.items():
                w_temp = w_new.replace('wheel_', 'wheelT_')
                for tag in root.findall('.//'):
                    for attr in ['name', 'link']:
                        if tag.get(attr) == w:
                            tag.set(attr, w_temp)
                            
            for w, w_new in wheel_map.items():
                w_temp = w_new.replace('wheel_', 'wheelT_')
                for tag in root.findall('.//'):
                    for attr in ['name', 'link']:
                        if tag.get(attr) == w_temp:
                            tag.set(attr, w_new)
                            
    return rule6_wheels_renamed, wheels

def compute_updated_child_rotation(child_name, jname, jtype, R_root_to_nominal_child, P_root_to_original_link):
    is_wheel = ('wheel' in jname.lower() and jtype in ['continuous', 'revolute'])
    is_grasp = (jname and 'grasp' in jname.lower())
    is_fingertip_right = ('fingertip' in child_name.lower() and 'right' in child_name.lower() and 'aruco' not in child_name.lower())
    is_fingertip_left = ('fingertip' in child_name.lower() and 'left' in child_name.lower() and 'aruco' not in child_name.lower())
    is_sensor_base = any(x in child_name.lower() for x in ['camera_', 'line_sensor_']) and 'optical' not in child_name.lower()
    is_optical_frame = ('optical' in child_name.lower() or 'lidar' in child_name.lower())

    if is_optical_frame or is_sensor_base:
        child_lower = child_name.lower()
        if 'camera_left' in child_lower or 'camera_center' in child_lower:
            if is_optical_frame:
                return R_root_to_nominal_child @ rpy_to_matrix(r=0, p=0, y=np.pi/2)
            else:
                return R_root_to_nominal_child @ rpy_to_matrix(r=-np.pi/2, p=-np.pi/2, y=-np.pi/2)
        elif 'camera_right' in child_lower:
            if is_optical_frame:
                return R_root_to_nominal_child @ rpy_to_matrix(r=0, p=0, y=-np.pi/2)
            else:
                return R_root_to_nominal_child @ rpy_to_matrix(r=0, p=-np.pi/2, y=0)
        elif 'line_sensor' in child_lower:
            if is_optical_frame: 
                return R_root_to_nominal_child @ rpy_to_matrix(r=0, p=-np.pi/2, y=np.pi)
            else: 
                return R_root_to_nominal_child @ rpy_to_matrix(r=-np.pi/2, p=0, y=0)
        elif 'gripper' in child_lower:
            if is_optical_frame:
                return R_root_to_nominal_child @ rpy_to_matrix(r=-np.pi/2, p=0, y=-np.pi/2)
            else:
                return R_root_to_nominal_child @ rpy_to_matrix(r=np.pi, p=0, y=0)
        else:
            return R_root_to_nominal_child

    elif is_grasp:
        return rpy_to_matrix(r=0,p=0,y=np.pi/2)
    elif is_fingertip_right:
        return rpy_to_matrix(r=0, p=np.pi, y=0)
    elif is_fingertip_left:
        return rpy_to_matrix(r=np.pi, p=0, y=0)
    elif is_wheel:
        w_pos = P_root_to_original_link.get(child_name, [0, 0, 0])
        theta = np.atan2(w_pos[1], w_pos[0])
        return rpy_to_matrix(r=-np.pi/2, p=0, y=theta + np.pi/2)
    else:
        return R_root_to_nominal_child

def apply_geometric_rules(root, R_root_to_original_link, P_root_to_original_link, R_root_to_original_joint):
    validation_data = {
        'rule1_changed': [],
        'rule3_rotation_warn': [],
        'rule4_sensor_bases': [],
        'rule4_optical_frames': [],
        'rule5_grasp_changed': [],
        'rule_polarity_flip_warn': []
    }

    tree_map = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            tree_map.setdefault(parent.get('link'), []).append((child.get('link'), joint))

    all_children = set(ch for children in tree_map.values() for ch,_ in children)
    root_links = [p for p in tree_map.keys() if p not in all_children]
    root_link = root_links[0] if root_links else 'base_link'

    R_root_to_updated_link = {root_link: np.eye(3)}
    queue = [root_link]

    while queue:
        parent_link = queue.pop(0)
        R_root_to_updated_parent = R_root_to_updated_link.get(parent_link, np.eye(3))
        R_root_to_original_parent = R_root_to_original_link.get(parent_link, np.eye(3))
        R_updated_parent_to_original_parent = R_root_to_updated_parent.T @ R_root_to_original_parent

        for child_name, joint in tree_map.get(parent_link, []):
            jname = joint.get('name')
            jtype = joint.get('type')
            
            is_prismatic = (jtype == 'prismatic')
            is_wrist_rotation = (jname and 'wrist' in jname.lower() and jtype in ['revolute', 'continuous'])
            is_arm_link = child_name in [f'arm_l{i}_link' for i in range(5)]
            
            is_wheel = ('wheel' in jname.lower() and jtype in ['continuous', 'revolute'])
            is_grasp = (jname and 'grasp' in jname.lower())
            is_tool_attachment_site = (child_name == 'tool_attachment_site_link')
            is_quick_connect_interface = (child_name == 'quick_connect_interface_link')
            is_link_wrist = (child_name == 'wrist_link')
            is_sensor_base = any(x in child_name.lower() for x in ['camera_', 'line_sensor_']) and 'optical' not in child_name.lower()
            is_optical_frame = ('optical' in child_name.lower() or 'lidar' in child_name.lower())
            
            R_root_to_original_child = R_root_to_original_link.get(child_name, np.eye(3))
            R_root_to_nominal_child = R_root_to_updated_parent @ R_updated_parent_to_original_parent @ R_root_to_original_parent.T @ R_root_to_original_child 
            changed_for_rule1 = False
            
            if is_prismatic or is_wrist_rotation or is_arm_link or is_tool_attachment_site or is_link_wrist or is_quick_connect_interface:
                R_root_to_updated_child = np.eye(3)
            else:
                R_root_to_updated_child = compute_updated_child_rotation(child_name, jname, jtype, R_root_to_nominal_child, P_root_to_original_link)
                
            if is_wrist_rotation or is_prismatic:
                axis = joint.find('axis')
                if axis is None: 
                    axis = ET.SubElement(joint, 'axis')
                    axis.set('xyz', "1 0 0")
                ax, ay, az = 0.0, 0.0, 1.0
                if axis.get('xyz'): ax, ay, az = [float(x) for x in axis.get('xyz').split()]
                
                b = R_root_to_original_child @ np.array([ax, ay, az])
                c = R_root_to_updated_child.T @ b
                cx, cy, cz = c
                
                s_axis = "1 0 0" 
                max_val = -1
                for axis_v, s_ in [([1,0,0], "1 0 0"), ([-1,0,0], "-1 0 0"), ([0,1,0], "0 1 0"), ([0,-1,0], "0 -1 0"), ([0,0,1], "0 0 1"), ([0,0,-1], "0 0 -1")]:
                    dot = cx*axis_v[0] + cy*axis_v[1] + cz*axis_v[2]
                    if dot > max_val:
                        max_val = dot
                        s_axis = s_
                        
                if s_axis == "-1 0 0":
                    validation_data['rule_polarity_flip_warn'].append(jname)
                    s_axis = "1 0 0"
                elif s_axis == "0 -1 0":
                    validation_data['rule_polarity_flip_warn'].append(jname)
                    s_axis = "0 1 0"
                elif s_axis == "0 0 -1":
                    validation_data['rule_polarity_flip_warn'].append(jname)
                    s_axis = "0 0 1"
                    
                if axis.get('xyz') != s_axis: changed_for_rule1 = True
                axis.set('xyz', s_axis)

            if is_wheel:
                axis = joint.find('axis')
                if axis is None: 
                    axis = ET.SubElement(joint, 'axis')
                
                ax, ay, az = 0.0, 0.0, 1.0
                if axis.get('xyz'): ax, ay, az = [float(x) for x in axis.get('xyz').split()]
                
                b = R_root_to_original_child @ np.array([ax, ay, az])
                c = R_root_to_updated_child.T @ b
                cx, cy, cz = c
                
                if cz < -0.5:
                    validation_data['rule_polarity_flip_warn'].append(jname)
                    
                if axis.get('xyz') != "0 0 1": changed_for_rule1 = True
                axis.set('xyz', "0 0 1")

            R_root_to_updated_link[child_name] = R_root_to_updated_child
            if is_grasp: validation_data['rule5_grasp_changed'].append(jname)
            if is_optical_frame or is_sensor_base:
                is_changed = not np.allclose(R_root_to_updated_child, R_root_to_nominal_child, atol=1e-5)
                if is_changed:
                    if is_optical_frame:
                        validation_data['rule4_optical_frames'].append(child_name)
                    elif is_sensor_base:
                        validation_data['rule4_sensor_bases'].append(child_name)
            
            origin = joint.find('origin')
            if origin is None: origin = ET.SubElement(joint, 'origin')
            xyz_old = [0.0]*3
            if origin.get('xyz'): xyz_old = [float(x) for x in origin.get('xyz').split()]
            
            n_xyz = R_updated_parent_to_original_parent @ np.array(xyz_old)
            new_xyz = clean(n_xyz)
            if origin.get('xyz') != new_xyz: changed_for_rule1 = True
            origin.set('xyz', new_xyz)
            
            R_updated_parent_to_updated_child = R_root_to_updated_parent.T @ R_root_to_updated_child
            nrpy = matrix_to_rpy(R_updated_parent_to_updated_child)
            new_rpy = clean(nrpy)
            if origin.get('rpy', '0 0 0') != new_rpy: changed_for_rule1 = True
            origin.set('rpy', new_rpy)
                
            if changed_for_rule1 and (is_prismatic or is_wrist_rotation or is_arm_link or is_wheel):
                validation_data['rule1_changed'].append(jname)
                
            if jtype in ['revolute', 'continuous']:
                axis = joint.find('axis')
                if axis is not None and axis.get('xyz'):
                    try:
                        if any(float(v) < 0 for v in axis.get('xyz').split()):
                            validation_data['rule3_rotation_warn'].append(jname)
                    except ValueError: pass

            queue.append(child_name)

    return R_root_to_updated_link, validation_data

def update_link_origins(root, R_root_to_original_link, R_root_to_updated_link):
    for link in root.findall('link'):
        lname = link.get('name')
        if lname in R_root_to_updated_link:
            R_root_to_updated_child = R_root_to_updated_link[lname]
            R_root_to_original_child = R_root_to_original_link.get(lname, np.eye(3))
            R_original_child_to_updated_child = np.matmul(np.transpose(R_root_to_updated_child), R_root_to_original_child)
            
            is_ident = all(abs(R_original_child_to_updated_child[i][j] - (1.0 if i==j else 0.0)) < 1e-6 for i in range(3) for j in range(3))
            if is_ident: continue

            for tag in ['visual', 'collision', 'inertial']:
                for el in link.findall(tag):
                    origin = el.find('origin')
                    if origin is None: origin = ET.SubElement(el, 'origin')
                    
                    xyz_original = np.zeros(shape=(1,3))
                    rpy_original = [0]*3
                    if origin.get('xyz'): xyz_original = np.array([float(x) for x in origin.get('xyz').split()]).reshape(1,3)
                    if origin.get('rpy'): rpy_original = [float(x) for x in origin.get('rpy').split()]
                    
                    xyz_updated = xyz_original @ R_original_child_to_updated_child
                    rpy_updated = rpy_to_matrix(*rpy_original) @ R_original_child_to_updated_child 
                    
                    xyz = clean(xyz_updated.tolist()[0])
                    rpy = clean(matrix_to_rpy(rpy_updated))

                    origin.set('xyz', xyz)
                    origin.set('rpy', rpy)
                    print(f"{lname}: xyz_original: {xyz_original}, rpy_original: {rpy_original}, xyz_updated: {xyz}, rpy_updated: {rpy}")

def print_validation_report(validation_data, rule5_arm_renamed, rule6_wheels_renamed, wheels):
    print("\n--- Validation Report ---")

    print("""Rule 1: Identity Alignment for Primary Structural Elements""")
    if validation_data['rule1_changed']:
        print(f"  -> Changed frames: {', '.join(validation_data['rule1_changed'])}")
    else:
        print("  -> 👍 No frames required changes.")
        
    print("""\nRule 2: Proximal to Distal Naming Convention""")
    if rule5_arm_renamed:
        print("  -> Reversed arm joint/link numerical sequences to enforce proximal-to-distal ordering.")
    else:
        print("  -> 👍 Arm naming sequence is correctly ordered.")
        
    print("""\nRule 3: Positive Axis Consistency""")
    if validation_data['rule3_rotation_warn']:
        print(f"  -> WARNING! The following frames violate this rule inherently: {', '.join(validation_data['rule3_rotation_warn'])}")
    elif validation_data['rule_polarity_flip_warn']:
        print(f"  -> WARNING! The following joints had their rotation axis forced positive (requires firmware flip): {', '.join(validation_data['rule_polarity_flip_warn'])}")
    else:
        print("  -> 👍 No frames flagged.")

    print("""\nRule 4: Sensor & Optical Link Coordinate Conventions""")
    if validation_data['rule4_optical_frames']:
        print(f"  -> Forced Optical Frames (Z-forward): {', '.join(validation_data['rule4_optical_frames'])}")
    if validation_data['rule4_sensor_bases']:
        print(f"  -> Forced Sensor Base Frames (X-forward): {', '.join(validation_data['rule4_sensor_bases'])}")
    if not validation_data['rule4_optical_frames'] and not validation_data['rule4_sensor_bases']:
        print("  -> 👍 No frames flagged.")
        
    print("""\nRule 5: Grasping Geometry Conventions""")
    if validation_data['rule5_grasp_changed']:
        print(f"  -> Changed frames: {', '.join(validation_data['rule5_grasp_changed'])}")
    else:
        print("  -> 👍 No frames required changes.")
        
    print("""\nRule 6: Wheel Conventions""")
    if wheels:
        if rule6_wheels_renamed:
            print("  -> Renamed wheels to ordered counter-clockwise naming and enforced center-pointing rotation axles.")
        else:
            print("  -> 👍 Wheels matched counter-clockwise naming. Enforced center-pointing rotation axles.")
    else:
        print("  -> 👍 No wheels detected.")
        
    print("-------------------------\n")

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

    print('Translating link_ convention to _link convention...')
    import re
    with open(stretch_main_xacro, 'r') as f:
        content = f.read()
    content = re.sub(r'\blink_([a-zA-Z0-9_]+)', r'\1_link', content)
    content = re.sub(r'\bjoint_([a-zA-Z0-9_]+)', r'\1_joint', content)
    with open(stretch_main_xacro, 'w') as f:
        f.write(content)

    print('Updating the URDF with collision mesh filepaths. If there is a _collision.STL file, its file path will be used in the collision geometry - replacing the existing mesh in the final XACRO file.')
    temp_urdf = os.path.join(os.path.dirname(base_urdf), 'temp.urdf')
    shutil.copy(stretch_main_xacro, temp_urdf)
    update_urdf_collision_meshes(temp_urdf, temp_urdf)
    shutil.copy(temp_urdf, stretch_main_xacro)
    os.remove(temp_urdf)
    remove_collision_from_optical_links(stretch_main_xacro, stretch_main_xacro)
    
    print('Ensuring prismatic joints share base link orientation and validating rotation joint axes...')
    tree = ET.parse(stretch_main_xacro)
    root = tree.getroot()
    
    rule5_arm_renamed = apply_rule2_proximal_distal_arm_naming(root)
    
    R_original_root_to_joint_dict, R_original_root_to_link_dict, t_original_root_to_link_dict = get_abs_poses(root)
    rule6_wheels_renamed, wheels = apply_rule6_wheel_renaming(root, t_original_root_to_link_dict)
    
    if rule6_wheels_renamed:
        # Update absolute poses after rename
        R_original_root_to_joint_dict, R_original_root_to_link_dict, t_original_root_to_link_dict = get_abs_poses(root)
        
    R_root_to_updated_link, validation_data = apply_geometric_rules(root, R_original_root_to_link_dict, t_original_root_to_link_dict, R_original_root_to_joint_dict)
    
    update_link_origins(root, R_original_root_to_link_dict, R_root_to_updated_link)
    
    tree.write(stretch_main_xacro)
    
    print_validation_report(validation_data, rule5_arm_renamed, rule6_wheels_renamed, wheels)
    
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
    """The model names match the folder names under the `stretch_urdf/` directory."""
    all_models, urdf_map, _ = get_all_model_names()
    
    # Descriptions should include all configurations that we officially "support"
    models_to_process = model_names_to_generate if model_names_to_generate else list(urdf_map.keys())
    
    for model_name in models_to_process:
        print('Generating URDF for',model_name)
        root_dir = './stretch4_urdf/'+model_name+'/'
        xacro_dir = root_dir + 'xacro/'
        generate_xacro_from_base_urdf(model_name, root_dir, xacro_dir)


if __name__ == '__main__':
    all_models, urdf_map, pkg_path = get_all_model_names()

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
