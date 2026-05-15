import argparse
import tempfile
import copy 

import numpy as np
from yourdfpy import urdf as ud

from stretch4_urdf import get_urdf_from_robot_params

wrist_pitch_lower_limit = -0.8 * (np.pi / 2.0)


def clip_joint_limits(robot: ud.URDF, use_original_limits=True):
    """
    Enables more conservative joint limits to be set than in the
    original URDF.

    If these limits are outside the originally permitted range,
    the original range is used. Joint limits. Where these limits
    have a value of None, the original limit is used.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation
    use_original_limits : bool
        don't impose any additional limits

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF where joint limits are clipped
    """
    ik_joint_limits: dict[str, tuple[float | None, float | None]] = {}
    if use_original_limits:
        ik_joint_limits = {
            "mobile_base_translation_joint": (None, None),
            "mobile_base_rotation_joint": (None, None),
            "lift_joint": (None, None),
            "arm_l4_joint": (None, None),
            "wrist_yaw_joint": (None, None),
            "wrist_pitch_joint": (
                wrist_pitch_lower_limit,
                None,
            ),  # Beware of gimbal lock if wrist_pitch_joint is too close to -90 deg
            "wrist_roll_joint": (None, None),
        }
    else:
        ik_joint_limits = {
            "mobile_base_translation_joint": (-0.25, 0.25),
            "mobile_base_rotation_joint": (-(np.pi / 2.0), np.pi / 2.0),
            "lift_joint": (0.01, 1.09),
            "arm_l4_joint": (0.01, 0.48),
            "wrist_yaw_joint": (-(np.pi / 4.0), np.pi),
            "wrist_pitch_joint": (-0.9 * (np.pi / 2.0), np.pi / 20.0),
            "wrist_roll_joint": (-(np.pi / 2.0), np.pi / 2.0),
        }
    for j in ik_joint_limits:
        joint = robot.joint_map.get(j, None)
        if joint is not None:
            original_upper = joint.limit.upper
            requested_upper = ik_joint_limits[j][1]
            if requested_upper is not None:
                new_upper = min(requested_upper, original_upper)
                robot.joint_map[j].limit.upper = new_upper

            original_lower = joint.limit.lower
            requested_lower = ik_joint_limits[j][0]
            if requested_lower is not None:
                new_lower = max(requested_lower, original_lower)
                robot.joint_map[j].limit.lower = new_lower


def make_joints_rigid(robot: ud.URDF, ignore_joints=None):
    """
    Change any joint that should be immobile for end effector IK
    into a fixed joint.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation
    ignore_joints : list(str) or None
        which joints to keep as-is

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF where joints are "fixed"
    """
    if ignore_joints is None:
        ignore_joints = []

    for j in robot.joint_map.keys():
        if j not in ignore_joints:
            joint = robot.joint_map[j]
            joint.type = "fixed"


def merge_arm(robot):
    """
    Replace telescoping arm with a single prismatic joint,
    which makes end-effector IK computation easier.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF with single arm joint
    """
    all_arm_joints = [
        "arm_l0_joint",
        "arm_l1_joint",
        "arm_l2_joint",
        "arm_l3_joint",
        "arm_l4_joint",
    ]
    prismatic_arm_joints = all_arm_joints[1:]
    removed_arm_joints = all_arm_joints[1:-1]
    near_proximal_arm_joint = robot.joint_map[all_arm_joints[1]]
    distal_arm_joint = robot.joint_map[all_arm_joints[-1]]

    # Calculate aggregate joint characteristics
    xyz_total = np.array([0.0, 0.0, 0.0])
    limit_upper_total = 0.0
    for j in prismatic_arm_joints:
        joint = robot.joint_map[j]
        xyz_total = xyz_total + joint.origin[3, :3]
        if joint.limit is None:
            continue
        limit_upper_total = limit_upper_total + joint.limit.upper

    # Directly connect the proximal and distal parts of the arm
    distal_arm_joint.parent = near_proximal_arm_joint.parent

    # Make the distal prismatic joint act like the full arm
    distal_arm_joint.origin[3, :3] = xyz_total
    distal_arm_joint.limit.upper = limit_upper_total

    # Mark the eliminated joints as "fixed"
    for j in removed_arm_joints:
        joint = robot.joint_map[j]
        joint.type = "fixed"


def add_link_to_urdf(robot: ud.URDF, new_link: ud.Link):
    return ud.Robot(
        name=robot.robot.name,
        links=robot.robot.links + [new_link],
        joints=robot.robot.joints,
        materials=robot.robot.materials,
        transmission=robot.robot.transmission,
    )


def add_joint_to_urdf(robot: ud.URDF, new_joint: ud.Joint):
    return ud.Robot(
        name=robot.robot.name,
        links=robot.robot.links,
        joints=robot.robot.joints + [new_joint],
        materials=robot.robot.materials,
        transmission=robot.robot.transmission,
    )


def add_virtual_rotary_joint(robot: ud.URDF):
    """
    Add virtual rotary joint for mobile base.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF with mobile base rotation joint
    """
    virtual_base_rotary_link = ud.Link(name="virtual_base")
    limit_rotary = ud.Limit(effort=10, velocity=1, lower=-np.pi, upper=np.pi)
    mobile_base_rotation_joint = ud.Joint(
        name="mobile_base_rotation_joint",
        parent="virtual_base",
        child=robot.base_link,
        type="revolute",
        axis=np.array([0, 0, 1]),
        limit=limit_rotary,
        dynamics=None,
        safety_controller=None,
        calibration=None,
        mimic=None,
    )
    robot.robot = add_link_to_urdf(robot, virtual_base_rotary_link)
    robot.robot = add_joint_to_urdf(robot, mobile_base_rotation_joint)


def add_virtual_prismatic_joint(robot: ud.URDF):
    """
    Add virtual prismatic joint for mobile base.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF with mobile base as a prismatic joint
    """

    # Add a virtual base link
    virtual_base_prismatic_link = ud.Link(name="virtual_base")

    limit_prismatic = ud.Limit(effort=10, velocity=1, lower=-1.0, upper=1.0)

    mobile_base_translation_joint = ud.Joint(
        name="mobile_base_translation_joint",
        parent="virtual_base",
        child=robot.base_link,
        type="prismatic",
        axis=np.array([0, 1, 0]),
        limit=limit_prismatic,
        dynamics=None,
        safety_controller=None,
        calibration=None,
        mimic=None,
    )

    robot.robot = add_link_to_urdf(robot, virtual_base_prismatic_link)
    robot.robot = add_joint_to_urdf(robot, mobile_base_translation_joint)

def add_virtual_planar_joint(robot: ud.URDF):
    """
    Add virtual planar joint for mobile base.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF with mobile base as a planar joint
    """

    # Add a virtual base link
    virtual_base_planar_link = ud.Link(name="virtual_base")

    limit_planar = ud.Limit(effort=10, velocity=1, lower=-1.0, upper=1.0)

    mobile_base_planar_joint = ud.Joint(
        name="mobile_base_planar_joint",
        parent="virtual_base",
        child=robot.base_link,
        type="planar",
        axis=np.array([0, 0, 1]),
        limit=limit_planar,
        dynamics=None,
        safety_controller=None,
        calibration=None,
        mimic=None,
    )

    robot.robot = add_link_to_urdf(robot, virtual_base_planar_link)
    robot.robot = add_joint_to_urdf(robot, mobile_base_planar_joint)


def generate_robot_from_base_xacro():
    """
    Generates a `robot` URDF object from the model's base xacro.

    Returns
    -------
    urdf_parser_py.urdf.Robot
        the URDF robot object representing the robot model
    """

    urdf_tmp_file = get_urdf_from_robot_params(output_dir="/tmp")
    return ud.URDF.load(urdf_tmp_file)


def generate_urdf_from_robot(
    robot: ud.URDF, output_prefix: str, output_dir: str, description: str
):
    """
    Renders a `robot` URDF object out to a file in the output_dir folder.

    This enables you to safety generate URDFs on-the-fly
    to be used by your app. E.g. `generate_ik_urdfs()` uses
    this method to generate "calibrated" inverse kinematics
    URDFs, so each robot's unique backlash and skew parameters
    are baked into the IK calculations.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        the URDF representation to render out to a file
    output_prefix : str
        this gets prepended to the output filenames
    description : str
        description of the URDF, gets appended to the filename

    Returns
    -------
    str
        filepath of the generated URDF
    """

    filename = f"{output_dir}/{output_prefix}_{description}.urdf"
    robot.write_xml_file(filename)

    return filename


def generate_ik_urdfs(
    robot: ud.URDF,
    output_prefix: str,
    output_dir: str,
    rigid_wrist_urdf: bool = True,
    is_merge_arm: bool = True,
):
    """
    Generates URDFs for IK packages. The latest calibrated
    URDF is used as a starting point, then these modifications
    are applied:
      1. Clip joint limits
      2. Make non-IK joints rigid
      3. Merge arm joints
      4. Add virtual rotary base joint
      5. (optionally) Make wrist joints rigid

    Parameters
    ----------
    output_prefix : str
        this gets prepended to the output filenames
    rigid_wrist_urdf : bool or None
        whether to also generate a IK URDF with a fixed dex wrist

    Returns
    -------
    list(str)
        one or two filepaths, depending on `rigid_wrist_urdf`,
        to the generated URDFs. The first element will be the
        full IK version, and the second will be the rigid
        wrist version.
    """

    clip_joint_limits(robot)

    ignore_joints = [
        "lift_joint",
        "wrist_yaw_joint",
        "wrist_pitch_joint",
        "wrist_roll_joint",
    ]
    ignore_joints += [
        "arm_l1_joint",
        "arm_l2_joint",
        "arm_l3_joint",
        "arm_l4_joint",
    ]
    make_joints_rigid(robot, ignore_joints)

    if is_merge_arm:
        merge_arm(robot)

    robot_rotary = copy.copy(robot)
    robot_prismatic = copy.copy(robot)
    robot_planar = copy.copy(robot)

    add_virtual_rotary_joint(robot_rotary)
    add_virtual_prismatic_joint(robot_prismatic)
    add_virtual_planar_joint(robot_planar)

    ret = []
    fpath = generate_urdf_from_robot(
        robot_rotary, output_prefix, output_dir, "base_rotation_ik"
    )
    ret.append(fpath)

    fpath = generate_urdf_from_robot(
        robot_prismatic, output_prefix, output_dir, "base_translation_ik"
    )
    ret.append(fpath)

    fpath = generate_urdf_from_robot(
        robot_planar, output_prefix, output_dir, "base_planar_ik"
    )
    ret.append(fpath)

    if rigid_wrist_urdf:
        ignore_joints = [
            "mobile_base_translation_joint",
            "mobile_base_rotation_joint",
            "mobile_base_planar_joint",
            "lift_joint",
            "arm_l4_joint",
        ]
        make_joints_rigid(robot_rotary, ignore_joints)
        make_joints_rigid(robot_prismatic, ignore_joints)
        make_joints_rigid(robot_planar, ignore_joints)

        fpath = generate_urdf_from_robot(
            robot_rotary, output_prefix, output_dir, "base_rotation_ik_with_fixed_wrist"
        )
        ret.append(fpath)

        fpath = generate_urdf_from_robot(
            robot_prismatic,
            output_prefix,
            output_dir,
            "base_translation_ik_with_fixed_wrist",
        )
        ret.append(fpath)

        fpath = generate_urdf_from_robot(
            robot_planar,
            output_prefix,
            output_dir,
            "base_planar_ik_with_fixed_wrist",
        )
        ret.append(fpath)

    return ret


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate URDFs with virtual revolute and prismatic joints for IK packages"
    )

    file = tempfile.NamedTemporaryFile()

    parser.add_argument(
        "--prefix",
        type=str,
        default=file.name.split("/")[-1],
        help="Prefix for the output URDF files,e.g. stretch_dex_teleop_ii",
    )

    parser.add_argument(
        "--no-merge-arm",
        action="store_true",
        help="Do not merge the arm links and joints",
    )
    parser.add_argument(
        "--output-dir", type=str, default="/tmp", help="Folder to use for output"
    )

    args = parser.parse_args()

    no_merge_arm = args.no_merge_arm

    if no_merge_arm:
        print("Not merging arm links and joints")

    robot = generate_robot_from_base_xacro()
    print(generate_ik_urdfs(robot, args.prefix, args.output_dir))
