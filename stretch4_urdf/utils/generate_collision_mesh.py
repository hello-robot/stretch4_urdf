#!/usr/bin/env python3

"""
Generates a collision mesh from the bounding box of the input mesh.

Second, third and fourth args allow you to specify padding in the x,y,z axes.

Example usage:
Bounding box cubes for all the meshes in a directory: `cd ./meshes && for f in *.STL*; do uv run ../../generate_collision_mesh.py "$f"; done`
Or run the tool: `python generate_collision_cube.py <file.stl> [pad_x pad_y pad_z]`
"""
import sys
import os
import numpy as np
import yaml
import glob
import xml.etree.ElementTree as ET
from stl import mesh
import trimesh


def convex_hull_STL(stl_path: str) -> str:
    """
    Create a convex hull of the input STL mesh.
    
    Args:
        stl_path: Path to the STL file.
    
    Returns:
        Path to the newly created convex hull STL file.
    """
    mesh_in = trimesh.load(stl_path)
    
    if isinstance(mesh_in, trimesh.Scene):
        if len(mesh_in.geometry) == 0:
            raise ValueError(f"No geometry found in {stl_path}")
        mesh_in = trimesh.util.concatenate(mesh_in.dump())

    hull = mesh_in.convex_hull
    
    base, ext = os.path.splitext(stl_path)
    if base.endswith('_collision') or base.endswith('_collision_link'):
        output_path = f"{base}{ext}"
    elif base.endswith('_link'):
        output_path = f"{base[:-5]}_collision_link{ext}"
    else:
        output_path = f"{base}_collision{ext}"
    hull.export(output_path)

    print(f"Convex hull STL saved to: {output_path}")
    print(f"Original faces: {len(mesh_in.faces)}, Hull faces: {len(hull.faces)}")

    return output_path


def qem_STL(stl_path: str, simplification_ratio: float) -> str:
    """
    Simplify an STL mesh using Quadric Error Metrics (QEM).
    
    Args:
        stl_path: Path to the STL file.
        simplification_ratio: 0.0 to 1.0 (ratio of target faces to original faces).
        1.0: keep as is
        0.0: Maximal simplification
    
    Returns:
        Path to the newly created simplified STL file.
    """
    if simplification_ratio < 0:
        simplification_ratio = 0
    elif simplification_ratio > 1:
        simplification_ratio = 1

    import open3d as o3d

    # Load the mesh using trimesh first to handle potential Scene objects
    # and get face counts easily, then convert to Open3D
    mesh_in = trimesh.load(stl_path)
    
    if isinstance(mesh_in, trimesh.Scene):
        if len(mesh_in.geometry) == 0:
            raise ValueError(f"No geometry found in {stl_path}")
        mesh_in = trimesh.util.concatenate(mesh_in.dump())

    original_face_count = len(mesh_in.faces)
    target_faces = int(max(12, original_face_count * simplification_ratio))

    if target_faces < original_face_count:
        # Convert trimesh to open3d
        o3d_mesh = o3d.geometry.TriangleMesh()
        o3d_mesh.vertices = o3d.utility.Vector3dVector(mesh_in.vertices)
        o3d_mesh.triangles = o3d.utility.Vector3iVector(mesh_in.faces)
        
        # Simplify
        simplified_mesh = o3d_mesh.simplify_quadric_decimation(target_faces)
        
        # Convert back to trimesh for reliable export
        mesh_out = trimesh.Trimesh(vertices=np.asarray(simplified_mesh.vertices), 
                                   faces=np.asarray(simplified_mesh.triangles))
        
        base, ext = os.path.splitext(stl_path)
        if base.endswith('_collision') or base.endswith('_collision_link'):
            output_path = f"{base}{ext}"
        elif base.endswith('_link'):
            output_path = f"{base[:-5]}_collision_link{ext}"
        else:
            output_path = f"{base}_collision{ext}"
        mesh_out.export(output_path)
        final_face_count = len(mesh_out.faces)
    else:
        base, ext = os.path.splitext(stl_path)
        if base.endswith('_collision') or base.endswith('_collision_link'):
            output_path = f"{base}{ext}"
        elif base.endswith('_link'):
            output_path = f"{base[:-5]}_collision_link{ext}"
        else:
            output_path = f"{base}_collision{ext}"
        if stl_path != output_path:
            mesh_in.export(output_path)
        final_face_count = original_face_count

    print(f"QEM simplified STL saved to: {output_path}")
    print(f"Original faces: {original_face_count}, Simplified faces: {final_face_count}")

    return output_path


def nop_STL(stl_path: str) -> str:
    """
    Just copy the input STL to the collision STL path.
    
    Args:
        stl_path: Path to the STL file.
    
    Returns:
        Path to the newly created collision STL file.
    """
    import shutil
    base, ext = os.path.splitext(stl_path)
    if base.endswith('_collision') or base.endswith('_collision_link'):
        output_path = f"{base}{ext}"
    elif base.endswith('_link'):
        output_path = f"{base[:-5]}_collision_link{ext}"
    else:
        output_path = f"{base}_collision{ext}"
    if stl_path != output_path:
        shutil.copy2(stl_path, output_path)
    print(f"Copied {stl_path} to {output_path} (nop)")
    return output_path


def generate_collision_cube(input_path: str, padding: np.ndarray) -> str:
    """
    Generate a cube mesh that bounds the given STL mesh, with optional padding.
    
    Args:
        input_path: Path to the STL file to process.
        padding: np.ndarray of shape (3,) defining padding in X, Y, and Z.

    Returns:
        Path to the newly created collision cube STL file.
    """
    mesh_in = trimesh.load(input_path)
    if isinstance(mesh_in, trimesh.Scene):
        if len(mesh_in.geometry) == 0:
            raise ValueError(f"No geometry found in {input_path}")
        mesh_in = trimesh.util.concatenate(mesh_in.dump())

    min_corner = np.min(mesh_in.vertices, axis=0) - padding
    max_corner = np.max(mesh_in.vertices, axis=0) + padding

    extents = max_corner - min_corner
    center = (max_corner + min_corner) / 2.0
    
    transform = np.eye(4)
    transform[:3, 3] = center
    cube = trimesh.creation.box(extents=extents, transform=transform)

    base, ext = os.path.splitext(input_path)
    if base.endswith('_collision') or base.endswith('_collision_link'):
        output_path = f"{base}{ext}"
    elif base.endswith('_link'):
        output_path = f"{base[:-5]}_collision_link{ext}"
    else:
        output_path = f"{base}_collision{ext}"
    cube.export(output_path)

    print(f"Collision cube saved to: {output_path}")
    print(f"Bounding box min: {min_corner}")
    print(f"Bounding box max: {max_corner}")

    return output_path


def get_triangle_count(stl_path: str) -> int:
    """Return the number of triangles in an STL file."""
    if not os.path.exists(stl_path):
        return 0
    try:
        m = trimesh.load(stl_path)
        if isinstance(m, trimesh.Scene):
            if len(m.geometry) == 0:
                return 0
            m = trimesh.util.concatenate(m.dump())
        return len(m.faces)
    except Exception:
        return 0


def process_model_links(root_dir: str):
    """
    Process all links for a given model directory based on its collision_mesh_config.yaml.
    """
    # Load config
    config_path = os.path.join(root_dir, 'collision_mesh_config.yaml')
    collision_config = {}
    if os.path.exists(config_path):
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
            collision_config = config.get('links', {})
    else:
        print(f"Warning: No collision_mesh_config.yaml found in {root_dir}")
        return

    # Find the first URDF in the directory to find meshes
    urdf_files = glob.glob(os.path.join(root_dir, '*.urdf'))
    if not urdf_files:
        print(f"Error: No URDF files found in {root_dir}")
        return
    # Find all mesh links in all URDFs in the directory
    mesh_links_dict = {} # link_name -> (link_el, filename, urdf_base)
    urdf_files = glob.glob(os.path.join(root_dir, '*.urdf'))
    if not urdf_files:
        print(f"Error: No URDF files found in {root_dir}")
        return

    mesh_dir = os.path.join(root_dir, 'meshes')
    
    for urdf_filepath in urdf_files:
        urdf_base = os.path.basename(urdf_filepath)
        tree = ET.parse(urdf_filepath)
        root = tree.getroot()
        
        for link in root.findall('link'):
            link_name = link.get('name')
            visual = link.find('visual')
            if visual is not None:
                geom = visual.find('geometry')
                if geom is not None:
                    mesh_el = geom.find('mesh')
                    if mesh_el is not None:
                        filename = mesh_el.get('filename')
                        if filename and link_name not in mesh_links_dict:
                            if 'optical' not in link_name.lower():
                                mesh_links_dict[link_name] = (link, filename, urdf_base)

    # Check for missing links in config and add them
    missing_links = [ln for ln in mesh_links_dict if ln not in collision_config]
    if missing_links:
        print(f"Adding missing links to config with default action 'bounding_box': {missing_links}")
        for ln in missing_links:
            collision_config[ln] = {'action': 'bounding_box'}
        
        # Write back to yaml
        with open(config_path, 'w') as f:
            yaml.dump({'links': collision_config}, f, default_flow_style=False, sort_keys=False)
        print(f"Updated {config_path}")

    summary_data = []
    print(f"Processing links found in {root_dir}...")
    
    for link_name, (link, filename, urdf_base) in mesh_links_dict.items():
        # Extract mesh name
        mesh_base = os.path.basename(filename)
        mesh_path = os.path.join(mesh_dir, mesh_base)
        
        if not os.path.exists(mesh_path) or not mesh_path.lower().endswith(('.stl', '.obj', '.dae')):
            continue

        original_faces = get_triangle_count(mesh_path)
        action = None
        params = ""
        
        cfg = collision_config[link_name]
        if not isinstance(cfg, dict):
            print(f"Error: Configuration for link '{link_name}' must be a dictionary. Got: {cfg}")
            continue

        action = cfg.get('action')
        if not action:
            print(f"Error: Missing 'action' key for link '{link_name}'.")
            continue
        
        if action == 'bounding_box':
            padding = cfg.get('padding', [0, 0, 0])
            if isinstance(padding, (int, float)):
                padding = [padding, padding, padding]
            padding = np.array(padding)
            params = f"pad={padding}"
            output_path = generate_collision_cube(mesh_path, padding)
        elif action == 'convex_hull':
            output_path = convex_hull_STL(mesh_path)
        elif action == 'qem':
            ratio = float(cfg.get('simplification_ratio', 0.5))
            params = f"ratio={ratio}"
            output_path = qem_STL(mesh_path, ratio)
        elif action == 'nop':
            output_path = nop_STL(mesh_path)
        else:
            print(f"Error: Unknown or removed action '{action}' for link {link_name}. Allowed: bounding_box, convex_hull, qem, nop")
            continue
        
        collision_faces = get_triangle_count(output_path)
        summary_data.append({
            'link': link_name,
            'mesh': mesh_base,
            'action': action,
            'params': params,
            'orig_faces': original_faces,
            'coll_faces': collision_faces
        })

    # Print summary
    print("\n" + "="*80)
    print(f"{'Link Name':<30} | {'Action':<15} | {'Orig':<8} | {'Coll':<8} | {'Params'}")
    print("-" * 80)
    for row in summary_data:
        print(f"{row['link']:<30} | {row['action']:<15} | {row['orig_faces']:<8} | {row['coll_faces']:<8} | {row['params']}")
    print("="*80 + "\n")
    
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Generate collision meshes for STL files.")
    parser.add_argument('input', nargs='?', help="Path to a single STL file (classic mode) OR model name (SE4_calder)")
    parser.add_argument('--model', help="Specify a robot model (e.g., SE4_calder) to process all links.")
    
    args = parser.parse_args()

    if args.model:
        # Process model directory
        # Attempt to find model root
        potential_roots = [
            f"./stretch4_urdf/{args.model}/",
            f"./{args.model}/",
            args.model
        ]
        root_dir = None
        for r in potential_roots:
            if os.path.isdir(r):
                root_dir = r
                break
        
        if not root_dir:
            print(f"Error: Could not find model directory for '{args.model}'")
            sys.exit(1)
        
        process_model_links(root_dir)
        
    elif args.input:
        if os.path.isfile(args.input):
            # Classic single file mode - defaults to 0 padding now that CLI arg is gone
            generate_collision_cube(args.input, np.zeros(3))
        else:
            # Maybe it's a model name passed as a positional arg
            # Attempt to find model root
            potential_roots = [
                f"./stretch4_urdf/{args.input}/",
                f"./{args.input}/",
                args.input
            ]
            root_dir = None
            for r in potential_roots:
                if os.path.isdir(r):
                    root_dir = r
                    break
            
            if root_dir:
                process_model_links(root_dir)
            else:
                print(f"Error: Input '{args.input}' not found as file or model directory.")
                sys.exit(1)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
