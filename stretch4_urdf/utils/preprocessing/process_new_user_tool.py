#!/usr/bin/env python3

"""
Generates a user tool's collision meshes and points its URDF at them.

This is a convenience function. It may not be necessary if you have already added your own
collision meshes and referenced them from your URDF.

Usage: python3 -m stretch4_urdf.utils.preprocessing.process_new_user_tool <tool_dir>

`tool_dir` is a tool directory under the fleet's user_tools directory. It must hold exactly one
.urdf file and a `meshes/` directory containing the meshes that URDF references.

This is the user-tool counterpart to process_new_tool, which handles the tools shipped inside the
stretch4_urdf package. It does not touch joint velocity or effort limits.
"""

import argparse
import os
import sys
import xml.etree.ElementTree as ET

from stretch4_urdf.utils.preprocessing.process_new_tool import (
    create_collision_config_if_missing, ensure_quick_connect_root_link,
    generate_collision_meshes)
from stretch4_urdf.utils.preprocessing.update_urdf_with_collision_mesh_filepath import (
    remove_collision_from_optical_links, update_urdf_collision_meshes)
from stretch4_urdf.utils.urdf_utils_generate_from_base_xacro import resolve_tool_urdf

MESH_DIR_ARG = '$(arg tool_mesh_dir)'


def normalize_mesh_paths(urdf_file, mesh_dir):
    """
    Rewrites every <mesh filename=...> in `urdf_file` to '$(arg tool_mesh_dir)/<basename>', which is
    how the assembled robot model locates a tool's meshes, and verifies each basename exists in
    `mesh_dir`.

    Matches on the mesh elements rather than on the text of their paths, so a reference written any
    way at all is normalized. Exits listing every basename missing from `mesh_dir`.
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    rewritten = []
    missing = []
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename')
        if not filename:
            continue

        basename = os.path.basename(filename)
        if not os.path.exists(os.path.join(mesh_dir, basename)):
            missing.append((basename, filename))
            continue

        normalized = f"{MESH_DIR_ARG}/{basename}"
        if filename != normalized:
            mesh.set('filename', normalized)
            rewritten.append((filename, normalized))

    if missing:
        lines = "\n".join(f"\t{b}\t(referenced as '{f}')" for b, f in missing)
        sys.exit(
            f"Error: {len(missing)} mesh(es) referenced by the URDF are not in {mesh_dir}:\n{lines}"
        )

    if rewritten:
        print(f"Pointing {len(rewritten)} mesh path(s) at {MESH_DIR_ARG}:")
        for before, after in rewritten:
            print(f"\t{before}\n\t  -> {after}")
        tree.write(urdf_file, encoding='utf-8', xml_declaration=False)
    else:
        print(f"All mesh paths already point at {MESH_DIR_ARG}.")


def process_user_tool_urdf(tool_dir):
    """
    Processes the tool in `tool_dir` in place and returns the path to its URDF.

    Overwrites collision meshes from a previous run and is safe to re-run.
    """
    tool_dir = os.path.abspath(os.path.expanduser(tool_dir))
    if not os.path.isdir(tool_dir):
        sys.exit(f"Error: not a directory:\n\t{tool_dir}")

    mesh_dir = os.path.join(tool_dir, 'meshes')
    if not os.path.isdir(mesh_dir):
        sys.exit(f"Error: no 'meshes' directory in the tool directory:\n\t{tool_dir}")

    try:
        urdf_file = resolve_tool_urdf(tool_dir)
    except ValueError as e:
        sys.exit(f"Error: {e}")

    print(f"Processing user tool: {os.path.basename(tool_dir)} ({tool_dir})")
    print(f"Found tool URDF: {os.path.basename(urdf_file)}")

    ensure_quick_connect_root_link(urdf_file)
    create_collision_config_if_missing(urdf_file, tool_dir)
    generate_collision_meshes(tool_dir)

    print('Updating the URDF with collision mesh filepaths...')
    update_urdf_collision_meshes(urdf_file, urdf_file)
    remove_collision_from_optical_links(urdf_file, urdf_file)

    normalize_mesh_paths(urdf_file, mesh_dir)

    print(f"Done processing user tool '{os.path.basename(tool_dir)}'.")
    return urdf_file


def main():
    parser = argparse.ArgumentParser(
        description="Convenience utility that generates a user tool's collision meshes and points "
                    "its URDF at them. It may not be necessary if you have already added your own "
                    "collision meshes."
    )
    parser.add_argument('tool_dir', help="Path to the user tool directory.")
    args = parser.parse_args()

    process_user_tool_urdf(args.tool_dir)


if __name__ == '__main__':
    main()
