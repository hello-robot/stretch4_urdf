"""
Reads a URDF file and appends "_collision.STL" to all mesh filenames in the
<collision> tag, but only if that collision STL exists in the ../meshes folder
relative to the URDF file's directory.

Run generate_collision_meshes.py first to generate the collision meshes.
"""

import os
import sys
import xml.etree.ElementTree as ET

def update_urdf_collision_meshes(input_file, output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()

    # Directory containing the URDF file
    urdf_dir = os.path.dirname(os.path.abspath(input_file)) 

    # Mesh directory to search for collision meshes
    collision_mesh_dir = os.path.abspath(os.path.join(urdf_dir, "meshes"))

    changed_links = []

    for collision in root.iter('collision'):
        geometry = collision.find('geometry')
        if geometry is None:
            continue

        mesh = geometry.find('mesh')
        if mesh is None:
            continue

        filename = mesh.get('filename')
        if not filename:
            continue

        # Extract original filename (ignore original dir)
        _, base = os.path.split(filename)
        name, ext = os.path.splitext(base)

        # Only handle mesh types
        if ext.lower() not in ['.stl', '.dae', '.obj']:
            continue

        # Collision filename in ../meshes
        if name.endswith("_collision_link"):
            candidate_collision_filename = name + ext
            collision.set("name", name)
        elif name.endswith("_link"):
            core = name[:-5]
            candidate_collision_filename = core + "_collision_link" + ext
            collision.set("name", core + "_collision_link")
        else:
            candidate_collision_filename = name + "_collision_link" + ext
            collision.set("name", name + "_collision_link")
        collision_path = os.path.join(collision_mesh_dir, candidate_collision_filename)

        # Check if that file exists
        if os.path.exists(collision_path):
            changed_links.append(name)

            rel_collision_path = os.path.relpath(collision_path, start=urdf_dir)

            mesh.set('filename', rel_collision_path)
            print(f"Found collision mesh: {collision_path}.")
        else:
            print(f"Skipping (not found): {collision_path}")

    if len(changed_links) == 0:
        print("No collision meshes found. Run generate_collision_meshes.py first to generate the collision meshes.")
    else:
        print(f"Changed links: {changed_links}")

    tree.write(output_file)
    print(f"Updated URDF saved to: {output_file}")

def remove_collision_from_optical_links(input_file, output_file):
    print("Removing collision from camera optical links")
    
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    for link in root.findall('link'):
        name = link.get('name')
        
        if name and ("_optical" in name or name == "base_footprint"):
            print(f"Processing link: {name}")
            
            # 1. Remove <inertial> tags
            inertials = link.findall('inertial')
            for i in inertials:
                link.remove(i)
                
            # 2. Remove <collision> tags
            collisions = link.findall('collision')
            for c in collisions:
                link.remove(c)
            
            # 3. Modify <visual> tags
            visuals = link.findall('visual')
            for visual in visuals:
                geometry = visual.find('geometry')
                if geometry is not None:
                    # Remove existing children of geometry (mesh, cylinder, etc.)
                    for child in list(geometry):
                        geometry.remove(child)
                    
                    # Add the new box geometry
                    # Creates: <box size="0.001 0.001 0.001"/>
                    box = ET.SubElement(geometry, 'box')
                    box.set('size', '0.001 0.001 0.001')

    tree.write(output_file)
    print(f"Updated URDF saved to: {output_file}")

if __name__ == "__main__":
    args = sys.argv
    if len(args) < 3:
        print("Usage: update_urdf_with_collision_mesh_filepath.py input.urdf output.urdf")
        sys.exit(1)

    update_urdf_collision_meshes(args[1], args[2])
    remove_collision_from_optical_links(args[1], args[2])
