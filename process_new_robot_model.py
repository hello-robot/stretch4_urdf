#!/usr/bin/env python3

"""This script generates Factory tool to generate URDFs from Xacros"""

import argparse
import subprocess
import shlex
import sys
import os
import glob
import yaml
import shutil
import re
from stretch4_urdf.utils.update_urdf_with_collision_mesh_filepath import update_urdf_collision_meshes, remove_collision_from_optical_links
import xacrodoc
import importlib.resources as importlib_resources
import math

def rpy_to_matrix(r, p, y):
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)

    R = [
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,            cx*cy]
    ]
    return R

def matrix_to_rpy(R):
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2][1], R[2][2])
        y = math.atan2(-R[2][0], sy)
        z = math.atan2(R[1][0], R[0][0])
    else:
        x = math.atan2(-R[1][2], R[1][1])
        y = math.atan2(-R[2][0], sy)
        z = 0
    return x, y, z

def mult_matrix(A, B):
    return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

def transpose(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]

def matrix_to_rpy(R):
    import math
    sy = math.sqrt(R[0][0]**2 + R[1][0]**2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2][1], R[2][2])
        y = math.atan2(-R[2][0], sy)
        z = math.atan2(R[1][0], R[0][0])
    else:
        x = math.atan2(-R[1][2], R[1][1])
        y = math.atan2(-R[2][0], sy)
        z = 0
    return x, y, z

def get_abs_poses(root):
    tree_map = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        origin = joint.find('origin')
        if parent is not None and child is not None:
            p_name = parent.get('link')
            c_name = child.get('link')
            rpy = [0.0, 0.0, 0.0]
            xyz = [0.0, 0.0, 0.0]
            if origin is not None:
                if origin.get('rpy'):
                    rpy = [float(x) for x in origin.get('rpy').split()]
                if origin.get('xyz'):
                    xyz = [float(x) for x in origin.get('xyz').split()]
            R_rel = rpy_to_matrix(*rpy)
            if p_name not in tree_map:
                tree_map[p_name] = []
            tree_map[p_name].append((c_name, R_rel, xyz, joint))

    all_children = set()
    for children in tree_map.values():
        for ch, _, _, _ in children:
            all_children.add(ch)
            
    root_links = [p for p in tree_map.keys() if p not in all_children]
    root_link = root_links[0] if root_links else 'base_link'
    
    abs_rot = {root_link: [[1,0,0],[0,1,0],[0,0,1]]}
    abs_pos = {root_link: [0.0, 0.0, 0.0]}
    queue = [root_link]
    
    joint_abs_rot = {}
    while queue:
        curr = queue.pop(0)
        curr_R = abs_rot[curr]
        curr_pos = abs_pos[curr]
        if curr in tree_map:
            for child, R_rel, xyz, joint in tree_map[curr]:
                child_R = mult_matrix(curr_R, R_rel)
                abs_rot[child] = child_R
                joint_abs_rot[joint.get('name')] = child_R
                
                pos_B = [
                    curr_pos[0] + curr_R[0][0]*xyz[0] + curr_R[0][1]*xyz[1] + curr_R[0][2]*xyz[2],
                    curr_pos[1] + curr_R[1][0]*xyz[0] + curr_R[1][1]*xyz[1] + curr_R[1][2]*xyz[2],
                    curr_pos[2] + curr_R[2][0]*xyz[0] + curr_R[2][1]*xyz[1] + curr_R[2][2]*xyz[2],
                ]
                abs_pos[child] = pos_B
                
                queue.append(child)
                
    return joint_abs_rot, abs_rot, abs_pos


def generate_xacro_from_base_urdf(model_name, root_dir, xacro_dir):
    # Find the base URDF file
    urdf_files = glob.glob(os.path.join(root_dir, '*.urdf'))
    
    if len(urdf_files) != 1:
        print(f"Error: Expected exactly one base URDF file in {root_dir}, found {len(urdf_files)}: {urdf_files}")
        return

    base_urdf = urdf_files[0]
    print(f"Found base URDF: {base_urdf}")

    import xml.etree.ElementTree as ET
    tree = ET.parse(base_urdf)
    root = tree.getroot()
    meshes_path = os.path.join(root_dir, 'meshes')

    def rename_everywhere(old_str, new_str):
        if not old_str or not new_str or old_str == new_str: return
        # XML
        for tag in root.iter():
            for attr in ['name', 'link']:
                val = tag.get(attr)
                if val == old_str:
                    tag.set(attr, new_str)
            
            # For filename, use substring replacement since it's often a path
            val = tag.get('filename')
            if val and old_str in val:
                tag.set('filename', val.replace(old_str, new_str))

        # Files
        if os.path.exists(meshes_path):
            for filename in os.listdir(meshes_path):
                if old_str in filename:
                    old_p = os.path.join(meshes_path, filename)
                    new_p = os.path.join(meshes_path, filename.replace(old_str, new_str))
                    if os.path.exists(old_p) and not os.path.exists(new_p):
                        os.rename(old_p, new_p)

    # Rule 2: Arm Renaming
    rule2_arm_renamed = False
    for j in root.findall('joint'):
        jname = j.get('name') or ""
        if jname in ['joint_arm_l4', 'arm_l4_joint']:
            pt = j.find('parent')
            if pt is not None and pt.get('link') == 'lift_link':
                rule2_arm_renamed = True
                break
                
    if rule2_arm_renamed:
        arm_indices = [0, 1, 2, 3, 4]
        for idx in arm_indices:
            rename_everywhere(f'link_arm_l{idx}', f'link_arm_T{idx}')
            rename_everywhere(f'joint_arm_l{idx}', f'joint_arm_T{idx}')
            rename_everywhere(f'arm_l{idx}', f'arm_T{idx}')
        for idx in arm_indices:
            rename_everywhere(f'link_arm_T{idx}', f'link_arm_l{4-idx}')
            rename_everywhere(f'joint_arm_T{idx}', f'joint_arm_l{4-idx}')
            rename_everywhere(f'arm_T{idx}', f'arm_l{4-idx}')

    # Rule 6: Wheel Renaming
    _, _, link_abs_P = get_abs_poses(root)
    wheels = []
    for joint in root.findall("joint"):
        jname, jtype = joint.get("name"), joint.get("type")
        if jname and "wheel" in jname.lower() and jtype in ["continuous", "revolute"]:
            child = joint.find("child")
            if child is not None:
                c_link = child.get("link")
                pos = link_abs_P.get(c_link, [0, 0, 0])
                wheels.append({"name": jname, "link": c_link, "pos": pos})

    if wheels:
        for w in wheels: w["theta"] = math.atan2(w["pos"][1], w["pos"][0])
        def ang_diff(a1, a2):
            diff = a1 - a2
            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi
            return diff
        closest_wheel = min(wheels, key=lambda w: abs(ang_diff(w["theta"], 0)))
        start_theta = closest_wheel["theta"]
        def ccw_dist(w):
            d = w["theta"] - start_theta
            while d < 0: d += 2*math.pi
            while d >= 2*math.pi: d -= 2*math.pi
            return d
        wheels_sorted = sorted(wheels, key=ccw_dist)
        wheel_map = []
        rule6_renamed = False
        for i, w in enumerate(wheels_sorted):
            m = re.search(r"wheel_(\d+)", w["name"])
            old_idx = m.group(1) if m else None
            if old_idx != str(i): rule6_renamed = True
            wheel_map.append((f"wheel_{old_idx}" if old_idx else "wheel", f"wheel_{i}"))
        
        if rule6_renamed:
            for old_base, new_base in wheel_map: rename_everywhere(f"link_{old_base}", f"link_{new_base.replace('wheel_', 'wheelT_')}")
            for old_base, new_base in wheel_map: rename_everywhere(f"joint_{old_base}", f"joint_{new_base.replace('wheel_', 'wheelT_')}")
            for old_base, new_base in wheel_map: rename_everywhere(old_base, new_base.replace("wheel_", "wheelT_"))

            for old_base, new_base in wheel_map: rename_everywhere(f"link_{new_base.replace('wheel_', 'wheelT_')}", f"link_{new_base}")
            for old_base, new_base in wheel_map: rename_everywhere(f"joint_{new_base.replace('wheel_', 'wheelT_')}", f"joint_{new_base}")
            for old_base, new_base in wheel_map: rename_everywhere(new_base.replace("wheel_", "wheelT_"), new_base)

    # Rule 7: Enforce suffix naming
    renames = {}
    for tag in root.iter():
        for attr in ['name', 'link', 'filename']:
            val = tag.get(attr)
            if val:
                if attr in ['name', 'link']:
                    if val.startswith('link_'):
                        renames[val] = val[5:] + '_link'
                    elif val.startswith('joint_'):
                        renames[val] = val[6:] + '_joint'
                elif attr == 'filename':
                    basename = os.path.basename(val)
                    if basename.startswith('link_'):
                        name_part, ext = os.path.splitext(basename[5:])
                        renames[basename] = name_part + '_link' + ext
    
    # Sort by length descending to avoid partial renames (e.g., link_wrist vs link_wrist_pitch)
    for old_name in sorted(renames.keys(), key=len, reverse=True):
        rename_everywhere(old_name, renames[old_name])
    
    # --- PHASE 2: Collision Generation ---

    temp_urdf = os.path.join(root_dir, "convention_corrected.urdf")
    tree.write(temp_urdf)
    
    bak_urdf = base_urdf + ".bak"
    if os.path.exists(bak_urdf): os.remove(bak_urdf)
    os.rename(base_urdf, bak_urdf)

    config_path = os.path.join(root_dir, "collision_mesh_config.yaml")
    if not os.path.exists(config_path):
        print(f"Creating default collision_mesh_config.yaml in {root_dir}")
        links_dict = {}
        for link in root.findall("link"):
            link_name = link.get("name")
            visual = link.find("visual")
            if visual is not None and "optical" not in link_name.lower():
                geom = visual.find("geometry")
                if geom is not None and geom.find("mesh") is not None:
                    links_dict[link_name] = {"action": "qem", "simplification_ratio": 0.1}
        with open(config_path, "w") as f:
            yaml.dump({"links": links_dict}, f, default_flow_style=False, sort_keys=False)

    print(f"Generating collision meshes for model: {model_name}...")
    gen_script = os.path.join(os.path.dirname(__file__), "stretch4_urdf", "utils", "generate_collision_mesh.py")
    try:
        subprocess.run(["python3", gen_script, "--model", model_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error generating collision meshes for {model_name}: {e}")

    if os.path.exists(base_urdf): os.remove(base_urdf)
    os.rename(bak_urdf, base_urdf)

    os.makedirs(xacro_dir, exist_ok=True)
    stretch_main_xacro = os.path.join(xacro_dir, "stretch_main.xacro")
    
    print("Updating the URDF with collision mesh filepaths...")
    update_urdf_collision_meshes(temp_urdf, stretch_main_xacro)
    remove_collision_from_optical_links(stretch_main_xacro, stretch_main_xacro)
    if os.path.exists(temp_urdf): os.remove(temp_urdf)

    # --- PHASE 3: Geometric Rules ---

    tree = ET.parse(stretch_main_xacro)
    root = tree.getroot()
    
    print("Applying geometric rules and axis validations...")
    joint_abs_R, link_abs_R, link_abs_P = get_abs_poses(root)
    sensor_opt_old = {}
    for joint in root.findall('joint'):
        if 'optical' in joint.get('name', '').lower():
            pt = joint.find('parent')
            ch = joint.find('child')
            if pt is not None and ch is not None:
                sensor_opt_old[pt.get('link')] = link_abs_R.get(ch.get('link'), [[1,0,0],[0,1,0],[0,0,1]])
                
    rule1_changed = []
    rule3_rotation_warn = []
    rule4_sensor_bases = []
    rule4_optical_frames = []
    rule5_grasp_changed = []
    rule_polarity_flip_warn = []

    def clean(val): return 0.0 if abs(val) < 1e-6 else val

    tree_map = {}
    for joint in root.findall('joint'):
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            tree_map.setdefault(parent.get('link'), []).append((child.get('link'), joint))

    all_children = set(ch for children in tree_map.values() for ch,_ in children)
    root_links = [p for p in tree_map.keys() if p not in all_children]
    root_link = root_links[0] if root_links else 'base_link'

    abs_R_new = {root_link: [[1,0,0],[0,1,0],[0,0,1]]}
    queue = [root_link]

    while queue:
        curr = queue.pop(0)
        R_new_A = abs_R_new[curr]
        R_new_A_inv = transpose(R_new_A)
        R_old_A = link_abs_R.get(curr, [[1,0,0],[0,1,0],[0,0,1]])
        delta_A = mult_matrix(R_new_A_inv, R_old_A)
        
        for child_name, joint in tree_map.get(curr, []):
            jname = joint.get('name')
            jtype = joint.get('type')
            
            is_prismatic = (jtype == 'prismatic')
            is_wrist_rotation = (jname and 'wrist' in jname.lower() and jtype in ['revolute', 'continuous'])
            is_arm_link = child_name in [f'arm_l{i}_link' for i in range(5)]
            
            is_wheel = ('wheel' in jname.lower() and jtype in ['continuous', 'revolute'])
            is_grasp = (jname and 'grasp' in jname.lower())
            is_tool_attachment_site = (child_name == 'tool_attachment_site_link')
            is_link_wrist = (child_name == 'wrist_link')
            
            is_fingertip_right = ('fingertip' in child_name.lower() and 'right' in child_name.lower() and 'aruco' not in child_name.lower())
            is_fingertip_left = ('fingertip' in child_name.lower() and 'left' in child_name.lower() and 'aruco' not in child_name.lower())
            is_sensor_base = child_name in sensor_opt_old
            is_optical_frame = ('optical' in jname.lower() or 'lidar' in child_name.lower())
            
            R_old_B = link_abs_R.get(child_name, [[1,0,0],[0,1,0],[0,0,1]])
            R_rel_old = mult_matrix(transpose(R_old_A) if 'R_old_A' in locals() else transpose(link_abs_R.get(curr, [[1,0,0],[0,1,0],[0,0,1]])), R_old_B)
            R_inherited = mult_matrix(R_new_A, R_rel_old)
            changed_for_rule1 = False
            
            if is_prismatic or is_wrist_rotation or is_arm_link or is_tool_attachment_site or is_link_wrist:
                R_new_B = [[1,0,0],[0,1,0],[0,0,1]]
            elif is_optical_frame:
                diff = abs(R_rel_old[0][0] - 1.0) + abs(R_rel_old[1][1] - 1.0) + abs(R_rel_old[2][2] - 1.0)
                if diff < 0.1:
                    R_new_B = [
                        [-R_inherited[0][1], -R_inherited[0][2], R_inherited[0][0]],
                        [-R_inherited[1][1], -R_inherited[1][2], R_inherited[1][0]],
                        [-R_inherited[2][1], -R_inherited[2][2], R_inherited[2][0]]
                    ]
                elif 'camera_' in child_name.lower() and '_link' in child_name.lower() and '_optical' in child_name.lower():
                    # Rule 4 Edit:
                    # Center matches Right Head Camera logically natively mapping outward.
                    if 'center' in child_name.lower():
                        R_new_B = [
                            [-R_new_A[0][1], -R_new_A[0][2], R_new_A[0][0]],
                            [-R_new_A[1][1], -R_new_A[1][2], R_new_A[1][0]],
                            [-R_new_A[2][1], -R_new_A[2][2], R_new_A[2][0]]
                        ]
                    else:
                        R_new_B = [
                            [R_new_A[0][1], R_new_A[0][2], R_new_A[0][0]],
                            [R_new_A[1][1], R_new_A[1][2], R_new_A[1][0]],
                            [R_new_A[2][1], R_new_A[2][2], R_new_A[2][0]]
                        ]
                else:
                    R_new_B = R_inherited
                if child_name not in rule4_optical_frames: rule4_optical_frames.append(child_name)
            elif is_sensor_base:
                if 'camera_' in child_name.lower() and '_link' in child_name.lower() and 'optical' not in child_name.lower():
                    # Base frames geometrically map native X definitively unequivocally explicitly parallel firmly dynamically into optical Z
                    R_new_B = [
                        [R_inherited[0][2], R_inherited[0][1], -R_inherited[0][0]],
                        [R_inherited[1][2], R_inherited[1][1], -R_inherited[1][0]],
                        [R_inherited[2][2], R_inherited[2][1], -R_inherited[2][0]]
                    ]
                else:
                    R_new_B = R_inherited
                if child_name not in rule4_sensor_bases: rule4_sensor_bases.append(child_name)
            elif is_grasp:
                R_new_B = [[0, 1, 0], [-1, 0, 0], [0, 0, 1]]
            elif is_fingertip_right:
                R_new_B = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            elif is_fingertip_left:
                R_new_B = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
            elif is_wheel:
                w_pos = link_abs_P.get(child_name, [0, 0, 0])
                theta = math.atan2(w_pos[1], w_pos[0])
                R_new_B = [
                    [-math.sin(theta), 0, -math.cos(theta)],
                    [ math.cos(theta),  0, -math.sin(theta)],
                    [               0, -1,                 0]
                ]
            else:
                R_new_B = R_inherited
                
            if is_wrist_rotation or is_prismatic:
                axis = joint.find('axis')
                if axis is None: 
                    axis = __import__('xml.etree.ElementTree', fromlist=['ElementTree']).SubElement(joint, 'axis')
                    axis.set('xyz', "1 0 0")
                ax, ay, az = 0.0, 0.0, 1.0
                if axis.get('xyz'): ax, ay, az = [float(x) for x in axis.get('xyz').split()]
                
                bx = R_old_B[0][0]*ax + R_old_B[0][1]*ay + R_old_B[0][2]*az
                by = R_old_B[1][0]*ax + R_old_B[1][1]*ay + R_old_B[1][2]*az
                bz = R_old_B[2][0]*ax + R_old_B[2][1]*ay + R_old_B[2][2]*az
                
                R_new_B_inv = transpose(R_new_B)
                cx = R_new_B_inv[0][0]*bx + R_new_B_inv[0][1]*by + R_new_B_inv[0][2]*bz
                cy = R_new_B_inv[1][0]*bx + R_new_B_inv[1][1]*by + R_new_B_inv[1][2]*bz
                cz = R_new_B_inv[2][0]*bx + R_new_B_inv[2][1]*by + R_new_B_inv[2][2]*bz
                
                s_axis = "1 0 0" 
                max_val = -1
                for axis_v, s_ in [([1,0,0], "1 0 0"), ([-1,0,0], "-1 0 0"), ([0,1,0], "0 1 0"), ([0,-1,0], "0 -1 0"), ([0,0,1], "0 0 1"), ([0,0,-1], "0 0 -1")]:
                    dot = cx*axis_v[0] + cy*axis_v[1] + cz*axis_v[2]
                    if dot > max_val:
                        max_val = dot
                        s_axis = s_
                        
                if s_axis == "-1 0 0":
                    rule_polarity_flip_warn.append(jname)
                    s_axis = "1 0 0"
                elif s_axis == "0 -1 0":
                    rule_polarity_flip_warn.append(jname)
                    s_axis = "0 1 0"
                elif s_axis == "0 0 -1":
                    rule_polarity_flip_warn.append(jname)
                    s_axis = "0 0 1"
                    
                if axis.get('xyz') != s_axis: changed_for_rule1 = True
                axis.set('xyz', s_axis)

            if is_wheel:
                axis = joint.find('axis')
                if axis is None: 
                    axis = __import__('xml.etree.ElementTree', fromlist=['ElementTree']).SubElement(joint, 'axis')
                
                ax, ay, az = 0.0, 0.0, 1.0
                if axis.get('xyz'): ax, ay, az = [float(x) for x in axis.get('xyz').split()]
                
                bx = R_old_B[0][0]*ax + R_old_B[0][1]*ay + R_old_B[0][2]*az
                by = R_old_B[1][0]*ax + R_old_B[1][1]*ay + R_old_B[1][2]*az
                bz = R_old_B[2][0]*ax + R_old_B[2][1]*ay + R_old_B[2][2]*az
                
                R_new_B_inv = transpose(R_new_B)
                cx = R_new_B_inv[0][0]*bx + R_new_B_inv[0][1]*by + R_new_B_inv[0][2]*bz
                cy = R_new_B_inv[1][0]*bx + R_new_B_inv[1][1]*by + R_new_B_inv[1][2]*bz
                cz = R_new_B_inv[2][0]*bx + R_new_B_inv[2][1]*by + R_new_B_inv[2][2]*bz
                
                if cz < -0.5:
                    rule_polarity_flip_warn.append(jname)
                    
                if axis.get('xyz') != "0 0 1": changed_for_rule1 = True
                axis.set('xyz', "0 0 1")

            abs_R_new[child_name] = R_new_B
            if is_grasp: rule5_grasp_changed.append(jname)
            
            origin = joint.find('origin')
            if origin is None: origin = __import__('xml.etree.ElementTree', fromlist=['ElementTree']).SubElement(joint, 'origin')
            xyz_old = [0.0]*3
            if origin.get('xyz'): xyz_old = [float(x) for x in origin.get('xyz').split()]
            
            nx = delta_A[0][0]*xyz_old[0] + delta_A[0][1]*xyz_old[1] + delta_A[0][2]*xyz_old[2]
            ny = delta_A[1][0]*xyz_old[0] + delta_A[1][1]*xyz_old[1] + delta_A[1][2]*xyz_old[2]
            nz = delta_A[2][0]*xyz_old[0] + delta_A[2][1]*xyz_old[1] + delta_A[2][2]*xyz_old[2]
            
            new_xyz = f"{clean(nx):.6g} {clean(ny):.6g} {clean(nz):.6g}"
            if origin.get('xyz') != new_xyz: changed_for_rule1 = True
            origin.set('xyz', new_xyz)
            
            R_rel_new = mult_matrix(R_new_A_inv, R_new_B)
            nrpy = matrix_to_rpy(R_rel_new)
            new_rpy = f"{clean(nrpy[0]):.6g} {clean(nrpy[1]):.6g} {clean(nrpy[2]):.6g}"
            if origin.get('rpy', '0 0 0') != new_rpy: changed_for_rule1 = True
            origin.set('rpy', new_rpy)
                
            if changed_for_rule1 and (is_prismatic or is_wrist_rotation or is_arm_link or is_wheel):
                rule1_changed.append(jname)
                
            if jtype in ['revolute', 'continuous']:
                axis = joint.find('axis')
                if axis is not None and axis.get('xyz'):
                    try:
                        if any(float(v) < 0 for v in axis.get('xyz').split()):
                            rule3_rotation_warn.append(jname)
                    except ValueError: pass

            queue.append(child_name)

    for link in root.findall('link'):
        lname = link.get('name')
        if lname in abs_R_new:
            R_new_B = abs_R_new[lname]
            R_old_B = link_abs_R.get(lname, [[1,0,0],[0,1,0],[0,0,1]])
            delta_B = mult_matrix(transpose(R_new_B), R_old_B)
            
            is_ident = all(abs(delta_B[i][j] - (1.0 if i==j else 0.0)) < 1e-6 for i in range(3) for j in range(3))
            if is_ident: continue
                
            for tag in ['visual', 'collision', 'inertial']:
                for el in link.findall(tag):
                    origin = el.find('origin')
                    if origin is None: origin = ET.SubElement(el, 'origin')
                    
                    xyz_old = [0.0]*3
                    rpy_old = [0.0]*3
                    if origin.get('xyz'): xyz_old = [float(x) for x in origin.get('xyz').split()]
                    if origin.get('rpy'): rpy_old = [float(x) for x in origin.get('rpy').split()]
                    
                    nx = delta_B[0][0]*xyz_old[0] + delta_B[0][1]*xyz_old[1] + delta_B[0][2]*xyz_old[2]
                    ny = delta_B[1][0]*xyz_old[0] + delta_B[1][1]*xyz_old[1] + delta_B[1][2]*xyz_old[2]
                    nz = delta_B[2][0]*xyz_old[0] + delta_B[2][1]*xyz_old[1] + delta_B[2][2]*xyz_old[2]
                    
                    R_vis_new = mult_matrix(delta_B, rpy_to_matrix(*rpy_old))
                    nrpy = matrix_to_rpy(R_vis_new)
                    
                    origin.set('xyz', f"{clean(nx):.6g} {clean(ny):.6g} {clean(nz):.6g}")
                    origin.set('rpy', f"{clean(nrpy[0]):.6g} {clean(nrpy[1]):.6g} {clean(nrpy[2]):.6g}")

    tree.write(stretch_main_xacro)

    print("\n--- Validation Report ---")

    print("""Rule 1: Identity Alignment for Primary Structural Elements""")
    if rule1_changed:
        print(f"  -> Changed frames: {', '.join(rule1_changed)}")
    else:
        print("  -> 👍 No frames required changes.")
        
    print("""\nRule 2: Proximal to Distal Naming Convention""")
    if rule2_arm_renamed:
        print("  -> Reversed arm joint/link numerical sequences to enforce proximal-to-distal ordering.")
    else:
        print("  -> 👍 Arm naming sequence is correctly ordered.")
        
    print("""\nRule 3: Positive Axis Consistency""")
    if rule3_rotation_warn:
        print(f"  -> WARNING! The following frames violate this rule inherently: {', '.join(rule3_rotation_warn)}")
    elif rule_polarity_flip_warn:
        print(f"  -> WARNING! The following joints had their rotation axis forced positive (requires firmware flip): {', '.join(rule_polarity_flip_warn)}")
    else:
        print("  -> 👍 No frames flagged.")

    print("""\nRule 4: Sensor & Optical Link Coordinate Conventions""")
    if rule4_optical_frames:
        print(f"  -> Forced Optical Frames (Z-forward): {', '.join(rule4_optical_frames)}")
    if rule4_sensor_bases:
        print(f"  -> Forced Sensor Base Frames (X-forward): {', '.join(rule4_sensor_bases)}")
    if not rule4_optical_frames and not rule4_sensor_bases:
        print("  -> 👍 No frames flagged.")
        
    print("""\nRule 5: Grasping Geometry Conventions""")
    if rule5_grasp_changed:
        print(f"  -> Changed frames: {', '.join(rule5_grasp_changed)}")
    else:
        print("  -> 👍 No frames required changes.")
        
    print("""\nRule 6: Wheel Conventions""")
    if 'wheels' in locals() and wheels:
        if rule6_renamed:
            print("  -> Renamed wheels to ordered counter-clockwise naming and enforced center-pointing rotation axles.")
        else:
            print("  -> 👍 Wheels matched counter-clockwise naming. Enforced center-pointing rotation axles.")
    else:
        print("  -> 👍 No wheels detected.")
        
    print("-------------------------\n")
    
    # Convert to actual Xacro by updating the robot tag and mesh paths
    print(f'Converting mesh paths to use $(arg model_mesh_dir)...')
    with open(stretch_main_xacro, 'r') as f:
        content = f.read()

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
        if os.path.isdir(full_path) and not entry.startswith("__") and not entry.endswith("_tools") and entry != "tools":
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
