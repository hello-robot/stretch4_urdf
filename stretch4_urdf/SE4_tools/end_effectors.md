# End Effector Tools for Stretch

This document outlines the available end effector tools for Stretch and provides instructions on how to add new tools to the robot.

## Available Tools

### Standard Gripper (SG4)
A compliant gripper with suction cup fingertips and calibrated kinematics for free-space opening/closing using the fingertip ArUco markers, allowing for grasp estimation.

### Parallel Gripper (PG4)
A robust parallel jaw gripper designed for precise manipulation and secure grasping of rigid objects.

### Tablet
A 13" tablet holder. This tool is often paired with Web Teleop, enabling you to use Stretch as a telepresence robot.

### Calibration Grid
A ChArUco calibration grid with visual and retroreflective fiducials. The robot can wave this tool around and calibrate its own head sensing suite autonomously.

## Adding a New Tool To Stretch

To add new end of arm hardware to Stretch and connect it to the software interface, you will need the mesh files and the URDF. These are typically exported from the CAD assembly or obtained from the manufacturer.

### 1. Create Tool Directory Structure

In the `stretch4_urdf` shared model tool directory (e.g. `SE4_tools`), create a directory for your tool with the following structure:

```text
> {model}_tools 
    > {tool_name} 
        > meshes 
        {tool_name}.urdf
```

### 2. Process the auto-generated tool URDF

First, ensure the tool root link is named `link_quick_connect_interface`. This is the connection point to the rest of the arm.
* This can be accomplished in a few ways:
    1. Name the connecting link `link_quick_connect_interface` before generating the URDF.
    2. Rename the existing root link. Ensure all instances of the name are updated.
    3. Add a new link and joint to the URDF file manually. This will be a "ghost" link with no collision or visual geometry, and the joint will be an identity transform to the existing root link.
        * Example:
            ```xml
            <link name="link_quick_connect_interface" />

            <joint name="joint_quick_connect_interface" type="fixed">
                <origin xyz="0 0 0" rpy="0 0 -0" />
                <parent link="link_quick_connect_interface" />
                <child link="{existing_tool_root_link}" />
            </joint>
            ```

Next, use the `process_new_tool.py` script located at the root of the repository to automate the remaining URDF setup.

#### How to Use `process_new_tool.py`

Run the script from the root of the repository:
```bash
python process_new_tool.py
```
The script is interactive. It will list all available tool folders containing a raw `.urdf` file and prompt you to select which one(s) to process.

#### What the Script Does

Once a tool is selected, the script automates the conversion process:

1. **Generates Collision Meshes**: It creates a default `collision_mesh_config.yaml` (if one doesn't exist) and runs `generate_collision_mesh.py` to automatically generate convex hull collision meshes from the visual geometries.
2. **Applies Collision Geometries**: It updates the `.urdf` file to point to the newly generated collision meshes (replacing visual meshes in collision tags where appropriate) and explicitly removes collision geometry from optical links.
3. **Parameterizes Mesh Paths**: It modifies the mesh filepaths inside the `.urdf` file to use the dynamic variable `$(arg tool_mesh_dir)`. This allows `SE4.xacro` to properly resolve the absolute paths at compile time.
4. **Cleans Up Unused Meshes**: It checks for unreferenced meshes in the `meshes/` folder and interactively offers to delete them.

### 3. Update Robot Configuration

Add the tool to `nominal_params` in [robot_params_SE4.py](https://github.com/hello-robot/stretch4_body/blob/main/src/stretch4_body/robot/robot_params_SE4.py#L628).

* Add the tool name to the `supported_eoa` list. This must be the same name as the folder in `stretch4_urdf`.
* Add a new entry for the tool in the [self_collision_mujoco](https://github.com/hello-robot/stretch4_body/blob/main/src/stretch4_body/robot/robot_params_SE4.py#L1462) field.
* Add the tool name as a new key in the `nominal_params` dictionary:
    * If the new tool is not actuated, it can use the existing `SE4_eoa_wrist_dw4_tool_nil` object.
    * If the new tool requires a control interface, add a dictionary for the new tool in the EndOfArm section above. Follow the structure of `SE4_eoa_wrist_dw4_tool_sg4` or another existing end of arm tool.
        * Define a new class in the top level `py_module_name` and create the class in [end_of_arm_tools.py](https://github.com/hello-robot/stretch4_body/blob/main/src/stretch4_body/subsystem/end_of_arm/end_of_arm_tools.py) and a class of the same name with `_Client` appended in [robot_client.py](https://github.com/hello-robot/stretch4_body/blob/main/src/stretch4_body/robot/robot_client.py).
        * Add the tool actuator under `devices` along with the `wrist_pitch`, `wrist_roll`, and `wrist_yaw`.
            * `device_params` expects another dictionary defined in the [EOA joints section](https://github.com/hello-robot/stretch4_body/blob/main/src/stretch4_body/robot/robot_params_SE4.py#L42) above the EndOfArm section.
            * `py_module_name` and `py_class_name` point to the tool's API.
        * For ROS support, include the `ros` section. `py_module_name` and `py_class_name` point to the command group that will be used by the ros driver. The standard module is [command_groups.py](https://github.com/hello-robot/stretch4_ros2/blob/jazzy/stretch_core/stretch_core/command_groups.py).
