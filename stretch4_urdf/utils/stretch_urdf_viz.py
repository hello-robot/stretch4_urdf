#!/usr/bin/env python3
import argparse
import time
import io
import rerun as rr
import yourdfpy
import numpy as np
import os

# Import stretch4_urdf (should be local)
import stretch4_urdf

# Try to import stretch4_body for live updates and params
try:
    import stretch4_body.core.hello_utils as hu
    from stretch4_body.core.robot_params import RobotParams
    from stretch4_body.robot.robot_client import RobotClient as Robot
    from stretch4_body.behavior.sentries.self_collision.self_collision_loop import SelfCollisionLoop
    HAS_STRETCH_BODY = True
except ImportError:
    HAS_STRETCH_BODY = False

def log_robot_to_rerun(urdf, name, color=[200, 200, 200, 255]):
    """Log the robot structure to Rerun."""
    arrows_origins = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    arrows_vectors = [[0.05, 0, 0], [0, 0.05, 0], [0, 0, 0.05]]
    arrows_colors = [[255, 0, 0], [0, 255, 0], [0, 0, 255]]

    for j_name, joint in urdf.joint_map.items():
        # Mesh path
        mesh_base_path = f"robot/{name}/{j_name}"
        # Frame path
        frame_base_path = f"frames/{name}/{j_name}"
        # Label path
        label_base_path = f"labels/{name}/{j_name}"
        
        # Log axes statically relative to the frame path
        rr.log(f"{frame_base_path}/axes", rr.Arrows3D(
            origins=arrows_origins,
            vectors=arrows_vectors,
            colors=arrows_colors
        ), static=True)
        
        # Log labels statically relative to the label path
        rr.log(f"{label_base_path}/text", rr.Points3D(
            positions=[[0, 0, 0]],
            labels=[j_name],
            colors=[color[:3]]
        ), static=True)

        # Log meshes
        link_name = joint.child
        if link_name in urdf.link_map:
            link = urdf.link_map[link_name]
            for i, visual in enumerate(link.visuals):
                if visual.geometry and visual.geometry.mesh and visual.geometry.mesh.filename:
                    mesh_path = f"{mesh_base_path}/mesh_{i}"
                    if visual.origin is not None:
                        v_trans = visual.origin[0:3, 3]
                        v_mat = visual.origin[0:3, 0:3]
                        rr.log(mesh_path, rr.Transform3D(translation=v_trans, mat3x3=v_mat), static=True)
                    
                    try:
                        # Log asset with color factor
                        rr.log(mesh_path, rr.Asset3D(path=visual.geometry.mesh.filename, albedo_factor=color), static=True)
                    except Exception as e:
                        print(f"Failed to log mesh {visual.geometry.mesh.filename}: {e}")

def update_robot_transforms(urdf, name, joint_states=None):
    """Update and log robot transforms in Rerun."""
    if joint_states:
        try:
            urdf.update_cfg(joint_states)
        except Exception as e:
            pass

    for j_name, joint in urdf.joint_map.items():
        link_name = joint.child
        try:
            matrix, _ = urdf.scene.graph.get(link_name)
            translation = matrix[0:3, 3]
            mat3x3 = matrix[0:3, 0:3]
            
            # Apply same transform to all three trees
            rr.log(f"robot/{name}/{j_name}", rr.Transform3D(translation=translation, mat3x3=mat3x3))
            rr.log(f"frames/{name}/{j_name}", rr.Transform3D(translation=translation, mat3x3=mat3x3))
            rr.log(f"labels/{name}/{j_name}", rr.Transform3D(translation=translation, mat3x3=mat3x3))
        except Exception as e:
            pass

def main():
    if HAS_STRETCH_BODY:
        hu.print_stretch_re_use()
    
    parser = argparse.ArgumentParser(description='Visualize Stretch calibrated and uncalibrated URDFs in Rerun')
    parser.add_argument('--model', type=str, help='Model name (e.g., SE4)', default="SE4")
    parser.add_argument('--batch', type=str, help='Batch name (e.g., francis)', default="francis")
    parser.add_argument('--tool', type=str, help='Tool name (e.g., eoa_wrist_dw4_tool_sg4)', default="eoa_wrist_dw4_tool_sg4")
    args = parser.parse_args()

    model = args.model
    batch = args.batch
    tool = args.tool

    # Fetch defaults from RobotParams if available and args not provided
    if HAS_STRETCH_BODY and (not model or not batch or not tool):
        try:
            _, robot_params = RobotParams.get_params()
            model = model or robot_params['robot']['model_name']
            batch = batch or robot_params['robot']['batch_name']
            tool = tool or robot_params['robot']['tool']
        except Exception as e:
            print(f"Failed to fetch default parameters: {e}")

    if not model or not batch or not tool:
        print("Error: model, batch, and tool must be provided via flags or be available in RobotParams.")
        return

    print(f"Visualizing URDF for Model: {model}, Batch: {batch}, Tool: {tool}")

    # Load uncalibrated URDF
    try:
        urdf_uncal_str = stretch4_urdf.get_urdf(model, batch, tool, do_add_file_prefix_to_absolute_paths=False)
        urdf_uncal = yourdfpy.URDF.load(io.StringIO(urdf_uncal_str))
    except Exception as e:
        print(f"Failed to load uncalibrated URDF: {e}")
        return

    # Load calibrated URDF if available
    urdf_cal = None
    try:
        # We prefer get_urdf_from_robot_params if available as it handles complex overlays
        if HAS_STRETCH_BODY:
            urdf_cal_str = stretch4_urdf.get_urdf_from_robot_params(apply_calibration=True, do_add_file_prefix_to_absolute_paths=False)
            if urdf_cal_str:
                urdf_cal = yourdfpy.URDF.load(io.StringIO(urdf_cal_str))
    except Exception as e:
        # Silently fail if calibration is not available, as it's optional
        pass

    rr.init("stretch_urdf_viz")
    rr.spawn(memory_limit="2GB")

    # Log uncalibrated robot (Grey/Translucent if calibrated is shown)
    uncal_color = [150, 150, 150, 100] if urdf_cal else [200, 200, 200, 255]
    log_robot_to_rerun(urdf_uncal, "uncalibrated", color=uncal_color)
    
    if urdf_cal:
        log_robot_to_rerun(urdf_cal, "calibrated", color=[50, 200, 50, 255]) # Greenish for calibrated

    print("Starting visualization. Press Ctrl+C to stop.")
    try:
        while True:
            # Static visualization for now
            update_robot_transforms(urdf_uncal, "uncalibrated")
            if urdf_cal:
                update_robot_transforms(urdf_cal, "calibrated")
            
            time.sleep(0.5)
                
    except KeyboardInterrupt:
        print("Stopping visualization.")

if __name__ == "__main__":
    main()
