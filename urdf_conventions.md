# Stretch URDF Standardization Conventions

This document outlines the enforced kinematic tree and coordinate axis conventions for Stretch robot models. The following rules are implemented in `process_new_robot_model.py` and `process_new_tool.py` to create xacro files from CAD-exported URDFs.

## Rule 0: Preserve original mechanical structure and behavior

Enforcing the following rules may result in a change to the defined axes of motion, but crucially, the mechanical structure of the robot will be unchanged.

## Rule 1: Identity Alignment for Primary Structural Elements

Primary structural links inherit the orientation of the base_link frame.

- The base `origin` relative rotation matrix is forced to Identity (`rpy="0 0 0"`).
- For rotation joints (e.g. wrist and wheel links), the zero position is aligned to the base_link frame's orientation.

## Rule 2: Proximal to Distal Naming Convention

Telescoping link chains are named numerically from the proximal link to the distal link.

- All structural arm links and joints `arm_l*` must order incrementally.
- **`arm_l0`** attaches to the lift
- **`arm_l4`** anchors the wrist

## Rule 3: Positive Axis Consistency

All axes of motion are positive unit vectors.

- If after Rule 1 is applied, any joint has a negative axis of motion, the axis is changed to positive and a warning is raised to advise checking the motor polarity.
- If Rule 1 causes the axis of motion to not align with a primary axis (x, y, or z), the axes are reoriented to align the axis of motion with the nearest primary axis.

## Rule 4: Sensor & Optical Link Coordinate Conventions

Optical and sensor frame orientations follow ROS standards

- **Sensor Base Frame (`link_camera_*`, `link_line_sensor_*`):** A sensor's mounting frame's x axis is pointed forward, parallel to the optical z axis.
- **Optical Frame (`_optical`):** The z axis aligns to the sensor's field of view, normal to the image plane. X points along the vertical axis of the image, and y points along the horizontal axis of the image.
  - **Head Cameras (Left/Right):** Physically mounted so the image is rotated 90 degrees outwards towards the robot's left/right, so the x axes point towards the robot's center and the y axes point in opposite directions.
  - **Head Camera (Center):** Physically mounted so the image is rotated 90 degrees to the robot's right, so the image is oriented similarly to the right camera. The x axis points to the robot's left, and the y axis points up.
- **Range Sensors (`link_lidar_`)**: The z axis points out of the sensor, aligning with the field of view. X is the horizontal axis, and y is the vertical axis.

## Rule 5: Grasping Geometry Conventions

A virtual grasp center link is added following the orientation of primary links

- **Grasp Center (`link_grasp_center`):** The main tool center point has the same x forward direction convention as the primary links. It has a parallel roll, pitch, and yaw orientation to the roll, pitch, and yaw wrist joints.
- **Fingertips (`link_fingertip_*`):** The x axes are normal to the finger tip surface, pointing inwards towards the grasp center.

## Rule 6: Wheel Conventions

- Wheels rotate about their z axes
- The z axis points towards the center of the base, so positive wheel rotation results in a ositive rotation about the base link z axis
- At the joint's zero position, the x axis points in the wheel's direction of motion
- Wheels are named counter-clockwise rotating about the base link's z axis, starting with wheel_0 for the wheel closest to forward direction of travel, followed by wheel_1 and wheel_2

## Rule 7: Joint and Link Naming Conventions

Joints and links are named with the suffix _joint or _link, respectively. For a body with multiple frames or a body that is repeated, names are kept consistent, with suffixes added as needed before the final _link or _joint (e.g. line_sensor_1_optical_link)
