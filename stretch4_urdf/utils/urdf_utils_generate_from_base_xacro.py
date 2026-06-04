import argparse
import importlib.resources as importlib_resources
import io
import os
import xml.etree.ElementTree as ET

from stretch4_body.core.robot_params import RobotParams
from xacrodoc import XacroDoc
import yaml
from yourdfpy import URDF

_DESCRIPTION_UNMODIFIED = "unmodified"
_DESCRIPTION_CALIBRATED = "calibrated"


# =============================================================================
# Public Functions (Main Entry Points)
# =============================================================================

def get_available_tools(model_name: str) -> list[str]:
    """
    Get the list of available tools for a given model.

    Args:
        `model_name` (`str`): Name of the model.

    Returns:
        `list[str]`: A list of names of available tools.
    """
    tools_dir = os.path.join(importlib_resources.files("stretch4_urdf"), f"{model_name}_tools")
    available_tools = [d for d in os.listdir(tools_dir) if os.path.isdir(os.path.join(tools_dir, d))]
    if model_name in ["SE4"]: 
        available_tools += ['eoa_wrist_dw4_tool_nil']

    return available_tools


def get_robot_params() -> tuple[str, str, str]:
    """
    Get the model, batch, and tool name from stretch4_body.core.robot_params.

    This behavior relies on active robot hardware parameters configuration environments.

    Returns:
        `tuple[str, str, str]`: A tuple containing model_name, batch_name, and tool_name.
    """
    try:
        _, robot_params = RobotParams.get_params()
        model_name = robot_params["robot"]["model_name"]
        batch_name = robot_params["robot"]["batch_name"]
        tool_name = robot_params["robot"]["tool"]
        return model_name, batch_name, tool_name
    except Exception:
        raise Exception(
            "stretch4_body.core.robot_params not found. If you are not running this on a robot, "
            "explicitly pass model, batch, and tool arguments."
        )


def get_joint_limits(urdf_contents: str) -> dict[str, tuple[float | None, float | None]]:
    """
    Parses the URDF contents and extracts lower and upper bounds for all joints that have them.

    Args:
        `urdf_contents` (`str`): Raw URDF contents.

    Returns:
        `dict[str, tuple[float | None, float | None]]`: A dictionary mapping joint names to a tuple (lower, upper).
    """
    try:
        urdf = URDF.load(io.BytesIO(urdf_contents.encode('utf-8')))
        limits = {}
        for joint in urdf.robot.joints:
            if joint.limit:
                limits[joint.name] = (joint.limit.lower, joint.limit.upper)
        return limits
    except Exception as e:
        print(f"Warning: Failed to parse URDF for joint limits: {e}")
        return {}


def xacro2urdf_string(
    model_name: str, 
    batch_name: str, 
    tool_name: str, 
    do_add_file_prefix_to_absolute_paths: bool = True
) -> str:
    """
    Generates Robot URDF contents from the model xacro file.

    Args:
        `model_name` (`str`): Name of the model.
        `batch_name` (`str`): Name of the batch.
        `tool_name` (`str`): Name of the tool.
        `do_add_file_prefix_to_absolute_paths` (`bool`): Whether to add the file:// prefix to absolute mesh paths.

    Returns:
        `str`: Raw URDF XML contents.
    """
    urdf_pkg_path = str(importlib_resources.files("stretch4_urdf"))
    xacro_file = os.path.join(urdf_pkg_path, f"{model_name}.xacro")
    model_mesh_dir = os.path.join(urdf_pkg_path, f"{model_name}_{batch_name}/meshes")
    
    if not os.path.exists(model_mesh_dir):
        raise FileNotFoundError(
            f"Failed to resolve model mesh directory:\n\t{model_mesh_dir}\n"
            f"If paths are pointing to an old location, try re-installing stretch4_urdf to update paths."
        )

    if 'nil' in tool_name:
        tool_mesh_dir = None
    else:
        tool_mesh_dir = os.path.join(urdf_pkg_path, f"{model_name}_tools/{tool_name}/meshes")
        if not os.path.exists(tool_mesh_dir):
            raise FileNotFoundError(
                f"Failed to resolve tool mesh directory:\n\t{tool_mesh_dir}\n"
                f"If paths are pointing to an old location, try re-installing stretch4_urdf to update paths."
            )

    xacro_doc = XacroDoc.from_file(
        xacro_file, 
        subargs={
            "batch": batch_name, 
            "tool": tool_name, 
            "pkg_path": urdf_pkg_path,
            "model_mesh_dir": model_mesh_dir,
            "tool_mesh_dir": tool_mesh_dir
        }
    )

    return xacro_doc.to_urdf_string(use_protocols=do_add_file_prefix_to_absolute_paths)


def generate_urdf_string(
    model_name: str | None = None,
    batch_name: str | None = None,
    tool_name: str | None = None,
    *,
    use_calibration: bool = True,
    do_add_file_prefix_to_absolute_paths: bool = True
) -> str:
    """
    Main entry point to get raw URDF string outputs.

    Falls back to hardware configuration via stretch4_body if arguments are omitted.

    Args:
        `model_name` (`str | None`): Name of the model.
        `batch_name` (`str | None`): Name of the batch.
        `tool_name` (`str | None`): Name of the tool.
        `use_calibration` (`bool`): Whether to apply joint calibration values.
        `do_add_file_prefix_to_absolute_paths` (`bool`): Whether to add the file:// prefix to absolute paths.

    Returns:
        `str`: Generated URDF XML contents.
    """
    r_model, r_batch, r_tool = get_robot_params()
    model_name = model_name or r_model
    batch_name = batch_name or r_batch
    tool_name = tool_name or r_tool

    urdf_contents = _generate_urdf_string(model_name, batch_name, tool_name, do_add_file_prefix_to_absolute_paths)
    
    if use_calibration:
        urdf_contents = _apply_calibration_to_urdf_string(urdf_contents)
        
    return urdf_contents


def generate_urdf_file(
    model_name: str | None = None,
    batch_name: str | None = None,
    tool_name: str | None = None,
    output_dir: str = "temp",
    *,
    output_prefix: str | None = None,
    use_calibration: bool = True,
    do_add_file_prefix_to_absolute_paths: bool = True
) -> str:
    """
    Generates the URDF string content and writes it to an output file.

    Args:
        `output_dir` (`str`): Directory where the output URDF file will be saved.
        `model_name` (`str | None`): Name of the model.
        `batch_name` (`str | None`): Name of the batch.
        `tool_name` (`str | None`): Name of the tool.
        `output_prefix` (`str`): Optional filename prefix for the saved URDF file.
        `use_calibration` (`bool`): Whether to apply joint calibration values.
        `do_add_file_prefix_to_absolute_paths` (`bool`): Whether to add the file:// prefix to absolute paths.

    Returns:
        `str`: The resolved file path of the generated URDF file.
    """
    r_model, r_batch, r_tool = get_robot_params()
    model_name = model_name or r_model
    batch_name = batch_name or r_batch
    tool_name = tool_name or r_tool

    urdf_contents = generate_urdf_string(
        model_name=model_name,
        batch_name=batch_name,
        tool_name=tool_name,
        use_calibration=use_calibration,
        do_add_file_prefix_to_absolute_paths=do_add_file_prefix_to_absolute_paths
    )

    if output_prefix is None:
        output_prefix = f"{model_name}_{batch_name}_{tool_name}"

    description = _DESCRIPTION_CALIBRATED if use_calibration else _DESCRIPTION_UNMODIFIED
    filename = os.path.join(output_dir, f"{output_prefix}_{description}.urdf")
    
    os.makedirs(output_dir, exist_ok=True)
    with open(filename, "w") as f:
        f.write(urdf_contents)

    return filename


def generate_urdf_obj(
    model_name: str | None = None,
    batch_name: str | None = None,
    tool_name: str | None = None,
    *,
    use_calibration: bool = True,
    do_add_file_prefix_to_absolute_paths: bool = True
) -> URDF:
    """
    Generates a parsed yourdfpy URDF object directly out of the pipeline configurations.

    Args:
        `model_name` (`str | None`): Name of the model.
        `batch_name` (`str | None`): Name of the batch.
        `tool_name` (`str | None`): Name of the tool.
        `use_calibration` (`bool`): Whether to apply joint calibration values.
        `do_add_file_prefix_to_absolute_paths` (`bool`): Whether to add the file:// prefix to absolute paths.

    Returns:
        `URDF`: Parsed yourdfpy URDF robot object instance.
    """
    urdf_contents = generate_urdf_string(
        model_name=model_name,
        batch_name=batch_name,
        tool_name=tool_name,
        use_calibration=use_calibration,
        do_add_file_prefix_to_absolute_paths=do_add_file_prefix_to_absolute_paths
    )
    return URDF.load(io.BytesIO(urdf_contents.encode('utf-8')))


def generate_urdf_file_from_robot_params(
    apply_calibration: bool = True, 
    do_add_file_prefix_to_absolute_paths: bool = True, 
    output_dir: str = "temp",
    prefix: str|None = None,
) -> str:
    """
    Generates Robot URDF contents from the model's base xacro and saves it to a file.
    
    Args:
        `apply_calibration` (`bool`): Whether to apply joint calibration values.
        `do_add_file_prefix_to_absolute_paths` (`bool`): Whether to add the file:// prefix to absolute paths.
        `output_dir` (`str`): Directory to save the output URDF file.
        `prefix` (`str`): Optional filename prefix for the saved URDF file.
    
    Returns:
        `str`: The resolved file path of the generated URDF file.
    """ 

    model_name, batch_name, tool_name = get_robot_params()
    
    return generate_urdf_file(
        model_name=model_name, 
        batch_name=batch_name, 
        tool_name=tool_name, 
        output_dir=output_dir, 
        output_prefix=prefix,
        use_calibration=apply_calibration,
        do_add_file_prefix_to_absolute_paths=do_add_file_prefix_to_absolute_paths
    )


# =============================================================================
# Internal Functions
# =============================================================================

def _generate_urdf_string(
    model_name: str, 
    batch_name: str, 
    tool_name: str, 
    do_add_file_prefix_to_absolute_paths: bool = True
) -> str:
    """
    Internal helper to validate configurations and build the initial URDF string using model parameters.

    Args:
        `model_name` (`str`): Name of the model.
        `batch_name` (`str`): Name of the batch.
        `tool_name` (`str`): Name of the tool.
        `do_add_file_prefix_to_absolute_paths` (`bool`): Whether to add the file:// prefix to absolute paths.

    Returns:
        `str`: Raw URDF XML contents.
    """
    print(f"GETTING URDF FOR MODEL: {model_name}, BATCH: {batch_name}, TOOL: {tool_name}")
    available_tools = get_available_tools(model_name)
    if tool_name not in available_tools: 
        raise ValueError(
            f"Unexpected tool for model {model_name}: {tool_name}\nTools available for {model_name}:\n"
            + "\n".join([f"\t{tool}" for tool in available_tools])
            + f"\nCheck robot model and tool settings or add a new tool to stretch4_urdf/{model_name}_tools"
        )

    return xacro2urdf_string(model_name, batch_name, tool_name, do_add_file_prefix_to_absolute_paths)


def _apply_calibration_to_urdf_string(urdf_contents: str) -> str:
    """
    Internal helper that reads calibration files from environment parameters and injects values into the URDF.

    Args:
        `urdf_contents` (`str`): Raw URDF contents.

    Returns:
        `str`: Calibrated URDF XML contents.
    """
    fleet_path = os.environ.get("HELLO_FLEET_PATH")
    fleet_id = os.environ.get("HELLO_FLEET_ID")
    
    if fleet_path and fleet_id:
        calib_file = os.path.join(fleet_path, fleet_id, "stretch_calibration_values.yaml")
        if os.path.exists(calib_file):
            with open(calib_file, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            root = ET.fromstring(urdf_contents)
            
            if calib_data and "robot_calibration" in calib_data and "joints" in calib_data["robot_calibration"]:
                joints_calib = calib_data["robot_calibration"]["joints"]
                for joint in root.findall('joint'):
                    name = joint.get('name')
                    if name in joints_calib:
                        joint_data = joints_calib[name]
                        
                        parent_elem = joint.find('parent')
                        if parent_elem is not None and 'parent' in joint_data:
                            if parent_elem.get('link') != joint_data['parent']:
                                print(f"Warning: Parent link mismatch for joint '{name}'. Expected: {joint_data['parent']}, but found: {parent_elem.get('link')}. Skipping calibration.")
                                continue
                        
                        origin = joint.find('origin')
                        if origin is None:
                            origin = ET.SubElement(joint, 'origin')
                        
                        if 'xyz' in joint_data:
                            origin.set('xyz', str(joint_data['xyz']))
                        if 'rpy' in joint_data:
                            origin.set('rpy', str(joint_data['rpy']))
                
                return str(ET.tostring(root, encoding='unicode', xml_declaration=True))
        else:
            print(f"Warning: Calibration file not found at {calib_file}")
    else:
        print("Warning: HELLO_FLEET_PATH or HELLO_FLEET_ID not set. Cannot load calibration.")

    return urdf_contents


# =============================================================================
# Execution Main Hook
# =============================================================================

def main() -> str:
    """
    Command line tool execution entry point to generate Robot URDF from the base xacro file.

    Returns:
        `str`: Raw URDF XML contents or the path to the saved URDF file depending on CLI arguments.
    """
    parser = argparse.ArgumentParser(description="Generate Robot URDF from base xacro file pipelines.")

    parser.add_argument("--prefix", type=str, default=None, help="Prefix for the output URDF files")
    parser.add_argument("--output-dir", type=str, default=None, help="Folder to save the output files")
    parser.add_argument("--no-calibration", action="store_true", help="Omit processing hardware parameters")

    try:
        model_name_param, batch_name_param, tool_name_param = get_robot_params()
    except Exception:
        model_name_param, batch_name_param, tool_name_param = None, None, None

    parser.add_argument("--model", type=str, default=model_name_param, help="robot model name")
    parser.add_argument("--batch", type=str, default=batch_name_param, help="robot batch name")
    parser.add_argument("--tool", type=str, default=tool_name_param, help="robot tool name")

    args = parser.parse_args()
    use_calibration = not args.no_calibration

    if args.output_dir:
        return generate_urdf_file(
            output_dir=args.output_dir,
            output_prefix=args.prefix,
            model_name=args.model,
            batch_name=args.batch,
            tool_name=args.tool,
            use_calibration=use_calibration
        )
    
    return generate_urdf_string(
        model_name=args.model,
        batch_name=args.batch,
        tool_name=args.tool,
        use_calibration=use_calibration
    )


if __name__ == "__main__":
    print(main())