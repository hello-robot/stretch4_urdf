# Breaking Kinematic Changes

This document lists the breaking kinematic and coordinate frame changes introduced between releases of the Stretch 4 URDF description. 

---

## 2026.07.20 — Head Camera Coordinate Frame Alignment

### 1. Updated Sensor Link Frames
The camera mounting base links (`camera_left_link`, `camera_right_link`, and `camera_center_link`) have been updated to all share the same roll orientation with their local coordinate z-axes pointing up.

### 2. Corrected Center Optical Frame
The center camera's optical frame joint has been corrected to match the OAK-FFC-IMX378 W sensor orientation.
