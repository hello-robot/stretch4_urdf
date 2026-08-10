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
Populate the generated `tool.urdf` in the root of your tool folder (`my_custom_tool/tool.urdf`). This is the file the robot loads, and Step 4 rewrites it in place.
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
4.  Rewrite `tool.urdf` in place with the collision meshes and mesh paths resolved. Re-running the script reprocesses `tool.urdf`, so it is safe to edit the URDF and run again whenever the kinematics change.
5.  Write a tailored, baseline configuration file called `tool_params.yaml`.

### Step 5: Configure Tool Parameters (`tool_params.yaml`)
The generated values describe a generic parallel gripper. Edit them to match your hardware **before** you activate the tool or restart the server. In the device block named after your tool, under `devices`:
*   `id`: the Feetech servo ID on the wrist bus. **Set this first.** A wrong ID stops the whole robot server from starting: the ping fails, the driver falls into a multi-second baud scan, the end-of-arm process misses its startup handshake, and the server restarts in a loop.
*   `eeprom_cfg`: written to the servo at startup, so wrong values reconfigure your hardware. `phase: 61` with `max_pos_limit: 0` and `min_pos_limit: 0` selects multi-turn; `phase: 45` with `max_pos_limit: 4095` selects single-turn. Keep the load and protection limits low during bring-up.
*   `range_deg`, `range_pad_deg`: joint travel measured on your hardware, and the margin held back at each end. Soft motion limits derive from these.
*   `homing_pwm`, `flip_encoder_polarity`: homing direction and drive strength, and the sign convention for positive motion. Start with a low PWM magnitude.
*   `req_calibration`: `1` requires homing before motion. `0` treats the joint as homed at startup, which only makes sense for single-turn or absolute setups.
*   `py_class_name` and `py_module_name`: the class and module of your actuator driver. These must match the class you actually define in Step 6.

Also review `stow` (per-joint stow targets, including your tool's joint), `collision_mgmt` (brake pairs against the base), and `self_collision_mujoco.exclusions` (link pairs that touch by design, such as two fingers when closed).

These edits stay dormant until the tool is activated in Step 7.

### Step 6: Customize Drivers
The generated drivers run as-is, but for a tool with an actuator they are a stub: no homing that suits your mechanism, no unit conversion, and no status for the visualizer. Implement the driver logic before running the tool on hardware.
*   **Naming**: every class's default `name=` argument, and the names in `tool_params.yaml`, must match your tool's folder name. A mismatch stops the server with `Parameters for device <NAME> not found`.
*   **`tool.py`** (actuator driver): `move_to`/`move_by` inherit world radians of the servo; add unit conversions if your users expect percent or millimeters. Publish a `gripper_conversion` dict in your status containing `finger_rad` so the visualizer and pose tools can read the finger angle.
*   **Homing**: the base `home()` detects the hardstop by a velocity stall, which suits rigid joints. Compliant, tendon-driven, or high-friction mechanisms may never trip that stall, and homing then presses the mechanism against its limit until it times out. For those, override `home()` to detect the stop by position settling (sample the position at intervals, declare contact when it stops changing) and keep the calibration bookkeeping identical to the base class.
*   **`end_of_arm.py`**: stow and homing order across the wrist joints and your tool.
*   **`client.py`**: high-level commands and helpers that mirror your driver's API. `stretch_gripper_home` and `stretch_gripper_jog` use this class.
*   **`collision.py`**: replace the placeholder joint names with the finger joint names from **your** URDF, and read the position from your driver's status. Unknown joint names are skipped silently, so a typo shows up as a visualizer that never moves. The server caches this mapper, so restart it after editing.

Validate everything resolves before touching the robot:
```bash
stretch_add_user_tool my_custom_tool --check
```
This imports every class named in `tool_params.yaml` and reports each result. A pass confirms the package is well-formed; it says nothing about the hardware values from Step 5.

### Step 7: Select and Activate Your Tool
To activate your new tool on the physical robot:
1.  Run the configure script:
2.  ```bash
    stretch_configure_tool
    ```
3.  Select your custom tool from the list (or enter its name manually if it was created in a custom path).
4.  Turn off power to the wrist/EOA when prompted, connect your physical tool, turn on power, and press any key to auto-detect.
5.  Confirm restarting the background services and homing the robot. Decline the homing prompt if you have yet to adapt `home()` in Step 6, since the default homing drives the mechanism into its hardstop until it times out.

Activating the tool is what makes your Step 5 parameters take effect, so this is the first point at which a wrong servo ID or eeprom value reaches the hardware.

### Step 8: Verification

1. Restart the server and home the robot using `stretch_body_server --restart` and `stretch_robot_home`, if you did not do it during `stretch_configure_tool`.
1. Run `stretch_collision_viz` and make sure the robot model looks correct.
1. Run `stretch_gripper_jog` and use `x` and `y` to open and close your tool. Make sure the tool on the robot matches the collision visualization from the previous step.
1. Run `stretch_gamepad_teleop` and move the wrist and your tool. Make sure the tool on the robot matches the collision visualization from the previous step.

Client tools read parameters once at startup, so relaunch them after any parameter change.

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
