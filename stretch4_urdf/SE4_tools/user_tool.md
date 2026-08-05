# Custom End-Of-Arm (EOA) Tool File Structure

This document describes the structure and available parameters for a custom user-defined end-of-arm tool added to Stretch.


## Step-by-Step Guide: Implementing a New Custom Tool

Follow these steps to fully design, register, and integrate your custom end-of-arm tool on Stretch:

### Step 1: Initialize the Tool Directory
Run the registration script with your custom tool name. This automatically initializes the directory structure and driver templates for you:
```bash
stretch_add_user_tool my_custom_tool
```
This command:
*   Creates the `~/stretch_user/user_tools/my_custom_tool/` directory.
*   Creates a `meshes/` subdirectory for CAD meshes.
*   Generates modular, boilerplate python driver files: `tool.py` (gripper), `end_of_arm.py` (overall EndOfArm driver), and `client.py` (client interface).
*   Generates helper configs and placeholder ROS templates: `command_group.py`, `gamepad.py`, `collision.py`, and `tool_params.yaml`.
*   Generates an empty, placeholder kinematics entry point: `tool.urdf`.
*   Copies this `user_tool.md` file into your folder for reference.

### Step 2: Prepare and Copy Meshes
1.  Export your tool's links/meshes from your CAD software (e.g., Solidworks, OnShape) as `.stl`, `.dae`, or `.obj` files.
2.  Copy these visual mesh files into the newly created `my_custom_tool/meshes/` directory.

### Step 3: Define the Kinematics (URDF)
Populate the `tool.urdf` file in the root of your tool folder (`my_custom_tool/tool.urdf`).
*   **Critical Requirement**: The base/root link of your custom tool URDF must be named `quick_connect_interface_link`.
*   Connect any articulating or static links of your tool relative to this root link using joints.

### Step 4: Process and Register the URDF
Re-run the registration tool to process your URDF and meshes:
```bash
stretch_add_user_tool my_custom_tool
```
This script will:
1.  Verify the URDF structure (and offer to automatically insert a `quick_connect_interface_link` root link if it is missing).
2.  Analyze your URDF links and generate a default `collision_mesh_config.yaml` listing links with visual meshes.
3.  Automatically decimate/simplify high-polygon visual meshes into lightweight, performance-friendly collision meshes under `meshes/`.
4.  Write a tailored, baseline configuration file called `tool_params.yaml`.

### Step 5: Configure Tool Parameters (`tool_params.yaml`)
Open `my_custom_tool/tool_params.yaml` and configure:
*   `stow`: Set coordinates for stowing the arm and wrist securely.
*   `devices`: Register specific joint actuators, motor IDs, physical limits, calibration procedures, and parameters.
*   `collision_mgmt`: Configure collision self-exclusion/brake pairs to protect the robot and your tool.

### Step 6: Customize Drivers (Optional)
If your tool has active actuators (like a custom gripper motor or custom sensors), implement the driver logic:
*   **Server-Side (`end_of_arm.py` and `tool.py`)**: Implement custom startup, homing, and stowing routines.
*   **Client-Side (`client.py`)**: Implement high-level Python commands, state checking, or helper functions.

### Step 7: Select and Activate Your Tool
To activate your new tool on the physical robot:
1.  Run the configure script:
2.  ```bash
    stretch_configure_tool
    ```
3.  Select your custom tool from the list (or enter its name manually if it was created in a custom path).
4.  Turn off power to the wrist/EOA when prompted, connect your physical tool, turn on power, and press any key to auto-detect.
5.  Confirm restarting the background services and homing the robot.

### Step 8: Verification

1. Restart the server and home the robotusing `stretch_body_server --restart` and `stretch_robot_home`, if you did not do it during `stretch_configure_tool`.
1. Run `stretch_collision_viz` and make sure the robot model looks correct.
1. Run `stretch_gripper_jog` and use `x` and `y` to open and close your tool. Make sure the tool on the robot matches the collision visualization from the previous step.
1. Run `stretch_gamepad_teleop` and move the wrist and your tool. Make sure the tool on the robot matches the collision visualization from the previous step.

Your custom tool is now active, registered, and ready for use!


## Directory Structure

When a custom tool is registered, the following files are organized in the tool's folder:

```text
> <tool_name>/
    > meshes/
        # Simplified collision STLs and raw visual STLs
    collision_mesh_config.yaml  # Configures collision mesh generation
    tool_params.yaml            # Self-contained runtime parameters
    tool.urdf                   # Parameterized kinematics configuration
    tool.py                     # (Optional) Server-side parallel gripper driver (FeetechSMHello subclass)
    end_of_arm.py               # (Optional) Server-side hardware driver (EndOfArm subclass)
    client.py                   # (Optional) Client-side software interface (EndOfArmClient subclass)
    command_group.py            # (Optional) ROS command group interface mapping
    gamepad.py                  # (Optional) Gamepad teleoperation control mapping
    collision.py                # (Optional) Collision checking configuration
    user_tool.md                # This reference guide
```

---

## File Breakdown

### 1. `tool_params.yaml`
This file isolates all nominal parameters specific to your custom tool. During robot startup, this file is loaded and deep-merged recursively into the system-wide nominal parameter tree.

Key sections of `tool_params.yaml`:
*   `py_class_name` & `py_module_name`: The Python class and module name of your server-side driver (loads from `end_of_arm.py` and maps to `EndOfArm` subclass).
*   `client_class_name` & `client_module_name`: The Python class and module name of your client-side driver (loads from `client.py` and maps to `EndOfArmClient` subclass).
*   `stow`: Defines the joints coordinates where the robot stows the arm and the wrist.
*   `devices`: Maps your joint actuators to physical or virtual drivers (e.g. `WristPitch`, `WristRoll`, `WristYaw`, or custom `ParallelGripper` devices inside `tool.py`).
*   `collision_mgmt`: Identifies bounding collision pairs to prevent self-collision of the tool against other robot parts (like the base).

---

### 2. `collision_mesh_config.yaml`
This file configures the `generate_collision_mesh.py` script during the preprocessing phase to produce optimized, lightweight collision meshes from dense visual mesh geometries.

#### Configuration Options

Each entry under the `links` key corresponds to a physical link name defined in your URDF. Each link supports the following fields:

| Field | Type | Description |
| :--- | :--- | :--- |
| `action` | `string` | The simplification strategy to use. |
| `simplification_ratio` | `float` | Used only with `qem` action. Specifies decimation factor (e.g., `0.1` reduces faces to 10%). |

#### Supported Actions (`action` field):

1.  **`qem`** *(Recommended)*: Uses Quadratic Error Metrics decimation to simplify high-polygon count CAD meshes to a fraction of their size, retaining complex shapes while preventing physics engine slowdowns. Expects `simplification_ratio` field.
2.  **`convex_hull`**: Wraps the visual mesh in a convex hull mesh. Ideal for simple, solid, non-articulating bodies.
3.  **`bounding_box`**: Generates a standard bounding box around the visual mesh. Best for simple protective housings or rectangular segments.
4.  **`nop`**: No simplification is applied. The visual mesh is used directly as the collision mesh.

*Example:*
```yaml
links:
  pjg_body_link:
    action: qem
    simplification_ratio: 0.1
  finger_right_link:
    action: convex_hull
  finger_left_link:
    action: bounding_box
```

---

### 3. `end_of_arm.py` (Server-Side Driver)
Subclasses `EndOfArm`. Loaded dynamically by the robot's server process inside the `EndOfArmLoop` background worker. It consumes commands from the queue (`q_cmd`) and controls physical hardware.

---

### 4. `tool.py` (Gripper / FeetechSMHello Driver)
Subclasses `FeetechSMHello`. Implements low-level control, registers physical parameters, feedback loops, and calibration for custom smart servos of active attachments.

---

### 5. `client.py` (Client-Side Interface)
Subclasses `EndOfArmClient`. Loaded dynamically by the client-side `RobotClient`. It is the interface used by user-scripts (e.g., joggers, teleoperation, or move APIs) to enqueue commands and inspect the tool's status.
