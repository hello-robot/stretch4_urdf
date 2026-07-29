# Custom End-Of-Arm (EOA) Tool File Structure

This document describes the structure and available parameters for a custom user-defined end-of-arm tool added to Stretch.

## Directory Structure

When a custom tool is registered, the following files are organized in the tool's folder:

```text
> <tool_name>/
    > meshes/
        # Simplified collision STLs and raw visual STLs
    collision_mesh_config.yaml  # Configures collision mesh generation
    tool_params.yaml            # Self-contained runtime parameters
    <tool_name>.urdf            # Parameterized kinematics configuration
    <tool_name>.py              # (Optional) Server-side hardware driver
    <tool_name>_client.py       # (Optional) Client-side software interface
    user_tool.md                # This reference guide
```

---

## File Breakdown

### 1. `tool_params.yaml`
This file isolates all nominal parameters specific to your custom tool. During robot startup, this file is loaded and deep-merged recursively into the system-wide nominal parameter tree.

Key sections of `tool_params.yaml`:
*   `py_class_name` & `py_module_name`: The Python class and module name of your server-side driver.
*   `client_class_name` & `client_module_name`: The Python class and module name of your client-side driver.
*   `stow`: Defines the joints coordinates where the robot stows the arm and the wrist.
*   `devices`: Maps your joint actuators to physical or virtual drivers (e.g. `WristPitch`, `WristRoll`, `WristYaw`, or custom `ParallelGripper` devices).
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

### 3. `<tool_name>.py` (Server-Side Driver)
Subclasses `EndOfArm`. Loaded dynamically by the robot's server process inside the `EndOfArmLoop` background worker. It consumes commands from the queue (`q_cmd`) and controls physical hardware.

---

### 4. `<tool_name>_client.py` (Client-Side Driver)
Subclasses `EndOfArmClient`. Loaded dynamically by the client-side `RobotClient`. It is the interface used by user-scripts (e.g., joggers, teleoperation, or move APIs) to enqueue commands and inspect the tool's status.

---

## Step-by-Step Guide: Implementing a New Custom Tool

Follow these steps to fully design, register, and integrate your custom end-of-arm tool on Stretch:

### Step 1: Initialize the Tool Directory
Run the registration script with your custom tool name. This automatically initializes the directory structure and driver templates for you:
```bash
stretch_add_user_tool ./my_custom_tool
```
This command:
*   Creates the `./my_custom_tool/` directory.
*   Creates a `meshes/` subdirectory for CAD meshes.
*   Generates boilerplate server-side (`my_custom_tool.py`) and client-side (`my_custom_tool_client.py`) driver files.
*   Copies this `user_tool.md` file into your folder for reference.

### Step 2: Prepare and Copy Meshes
1.  Export your tool's links/meshes from your CAD software (e.g., Solidworks, OnShape) as `.stl`, `.dae`, or `.obj` files.
2.  Copy these visual mesh files into the newly created `my_custom_tool/meshes/` directory.

### Step 3: Define the Kinematics (URDF)
Create a file named `my_custom_tool.urdf` in the root of your tool folder (`my_custom_tool/my_custom_tool.urdf`).
*   **Critical Requirement**: The base/root link of your custom tool URDF must be named `quick_connect_interface_link`.
*   Connect any articulating or static links of your tool relative to this root link using joints.

### Step 4: Process and Register the URDF
Re-run the registration tool to process your URDF and meshes:
```bash
stretch_add_user_tool ./my_custom_tool
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
*   **Server-Side (`my_custom_tool.py`)**: Implement custom startup, homing, and stowing routines.
*   **Client-Side (`my_custom_tool_client.py`)**: Implement high-level Python commands, state checking, or helper functions.

### Step 7: Select and Activate Your Tool
To activate your new tool on the physical robot:
1.  Run the configure script:
    ```bash
    stretch_configure_tool
    ```
2.  Select your custom tool from the list (or enter its name manually if it was created in a custom path).
3.  Turn off power to the wrist/EOA when prompted, connect your physical tool, turn on power, and press any key to auto-detect.
4.  Confirm restarting the background services and homing the robot.

Your custom tool is now active, registered, and ready for use!
