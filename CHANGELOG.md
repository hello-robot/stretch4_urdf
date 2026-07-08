# Changelog

The changes between releases of Stretch 4 URDF are documented here.

## [2026.07.08](https://pypi.org/project/hello-robot-stretch4-urdf/2026.7.8)

 - Add docking station URDF
 - Add `get_tranform()` method to get TF between 2 links
 - Organize preprocessing scripts and deps
 - Improve fetching of calibrated URDF

## [2026.07.02](https://pypi.org/project/hello-robot-stretch4-urdf/2026.7.2)

 - Fix to head collision mesh
 - Fix to link name in gripper
 - Fix to line sensor link name

## [2026.06.25](https://pypi.org/project/hello-robot-stretch4-urdf/2026.6.25)

 - Bugfix for outputting urdf to a nonexistant folder - calls os.mkdir
 - Methods to generate planar_ik_urdf, make_rotary_ik_urdf, make_translation_ik_urdf
 - Generate calibrated URDFs
 - Cleanup URDF post processing and optical frames
 - Bugfix for is stretch4_body isn't installed
 - Simplify dependencies
 - Improved docs in README

## [2026.05.27](https://pypi.org/project/hello-robot-stretch4-urdf/2026.5.27)

 - Bugfix for generating URDF with nil tool
 - Bugfix for permission error
 - Util function to read joint limits from URDF

## [2026.05.14](https://pypi.org/project/hello-robot-stretch4-urdf/2026.5.14)

Bugfix to include the .obj meshes in the release

## [2026.05.12](https://pypi.org/project/hello-robot-stretch4-urdf/2026.5.12)

The initial release of this description repository. It contains the URDF and meshes for the "francis" batch.
