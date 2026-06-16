import argparse
import importlib.resources as importlib_resources
import io
import os
import xml.etree.ElementTree as ET

import yaml
try:
    from stretch4_body.core.robot_params import RobotParams
    import stretch4_body.core.hello_utils as hello_utils
except Exception:
    RobotParams = None
    hello_utils = None
from xacrodoc import XacroDoc
from yourdfpy import URDF
import logging


logger = logging.getLogger("urdf_utils")

def get_available_tools(model_name:str):

    tools_dir = os.path.join(importlib_resources.files("stretch4_urdf"), f"{model_name}_tools")
    # Only include directories, so we ignore .md files or random files
    available_tools = [d for d in os.listdir(tools_dir) if os.path.isdir(os.path.join(tools_dir, d))]
    if model_name in ["SE4"]: 
        available_tools += ['eoa_wrist_dw4_tool_nil']

    return available_tools

def generate_urdf_from_xacro(model_name:str, batch_name:str, tool_name:str, do_add_file_prefix_to_absolute_paths: bool = True) -> str:
    """
    Generates Robot URDF contents from the SE4 xacro.

    `model_name`: Name of the model.
    `batch_name`: Name of the batch.
    `tool_name`: Name of the tool.
    `do_add_file_prefix_to_absolute_paths`: Whether to add the file:// prefix to the absolute paths of the meshes.
    

    Returns:
        str: raw urdf contents
    """
    urdf_pkg_path = str(importlib_resources.files("stretch4_urdf"))
    xacro_file = os.path.join(urdf_pkg_path, f"{model_name}.xacro")
    model_mesh_dir = os.path.join(urdf_pkg_path, f"{model_name}_{batch_name}/meshes")
    if not os.path.exists(model_mesh_dir):
        raise FileNotFoundError(f"Failed to resolve model mesh directory:\n\t{model_mesh_dir}\nIf paths are pointing to an old location, trying re-installing stretch4_urdf to update the paths and re-sourcing your workspace.")

    if 'nil' in tool_name:
        tool_mesh_dir = None
    else:
        tool_mesh_dir = os.path.join(urdf_pkg_path, f"{model_name}_tools/{tool_name}/meshes")
        if not os.path.exists(tool_mesh_dir):
            raise FileNotFoundError(f"Failed to resolve tool mesh directory:\n\t{tool_mesh_dir}\nIf paths are pointing to an old location, trying re-installing stretch4_urdf to update the paths and re-sourcing your workspace.")

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

def get_robot_params():
    """
    Get the model, batch, and tool name from stretch4_body.core.robot_params. This only works if you're running on a robot.
    
    Returns:
        tuple[str, str, str | None]: model_name, batch_name, tool_name
    """
    if RobotParams is None:
        logger.warning("stretch4_body not found. Cannot automatically fetch robot parameters.")
        return None, None, None

    try:
        _, robot_params = RobotParams.get_params()
        model_name = robot_params["robot"]["model_name"]
        batch_name = robot_params["robot"]["batch_name"]
        tool_name = robot_params["robot"]["tool"]
        return model_name, batch_name, tool_name
    except Exception as e:
        logger.warning(f"Failed to fetch robot parameters from stretch4_body: {e}")
        return None, None, None

def generate_urdf_file(urdf_contents: str, output_prefix: str, output_dir: str, description: str):
    """
    Generate the Robot URDF file from the raw string.

    Parameters
    ----------
    urdf_contents : str
        raw urdf contents
    output_prefix : str
        prefix for the output URDF file
    output_dir : str
        directory to save the output URDF file
    description : str
        description of the output URDF file
    

    Returns:
        str: filepath
    """

    filename = f"{output_dir}/{output_prefix}_{description}.urdf"
    with open(filename, "w") as f:
        f.write(urdf_contents)

    return filename

def get_urdf_from_robot_params(apply_calibration: bool = True, do_add_file_prefix_to_absolute_paths: bool = True, output_dir: str|None = None, prefix: str|None = None,):
    """
    Generates Robot URDF contents from the model's base xacro, and optionally saves it to a file if a directory is provided.
    
    Parameters
    ----------
    output_dir : str
        directory to save the output URDF file
    prefix : str
        prefix for the output URDF file
    

    Returns:
        str: raw urdf contents
    """
    model_name, batch_name, tool_name = get_robot_params()
    if apply_calibration: 
        return get_urdf_calibrated(model_name, batch_name, tool_name, do_add_file_prefix_to_absolute_paths, output_dir=output_dir, prefix=prefix)
    else:
        return get_urdf(model_name, batch_name, tool_name, do_add_file_prefix_to_absolute_paths, output_dir=output_dir, prefix=prefix)

def get_urdf_calibrated(
    model_name:str,
    batch_name:str,
    tool_name:str,
    do_add_file_prefix_to_absolute_paths: bool = True,
    output_dir: str|None = None,
    prefix: str|None = None,
    description: str = "calibrated"
    ):
    """
    Generates Robot URDF contents from the base xacro, applies joint calibration values 
    from stretch_calibration_values.yaml if available, and optionally saves it.
    """

    urdf_contents = get_urdf(
        model_name, 
        batch_name, 
        tool_name, 
        do_add_file_prefix_to_absolute_paths=do_add_file_prefix_to_absolute_paths, 
        output_dir=None
    )

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
                    logger.info(f"Applying calibration to {name}")
                    if name in joints_calib:
                        joint_data = joints_calib[name]
                        
                        # Confirm that the joint in URDF has the same parent link as specified in the calibration
                        parent_elem = joint.find('parent')
                        if parent_elem is not None and 'parent' in joint_data:
                            if parent_elem.get('link') != joint_data['parent']:
                                logger.warning(f"Parent link mismatch for joint '{name}'. Expected: {joint_data['parent']}, but found: {parent_elem.get('link')}. Skipping calibration.")
                                continue
                        
                        #TODO: Threshold for calibration delta? 
                        
                        origin = joint.find('origin')
                        if origin is None:
                            origin = ET.SubElement(joint, 'origin')
                        
                        if 'xyz' in joint_data:
                            origin.set('xyz', str(joint_data['xyz']))
                        if 'rpy' in joint_data:
                            origin.set('rpy', str(joint_data['rpy']))
                
                urdf_contents = ET.tostring(root, encoding='unicode')
        else:
            logger.warning(f"Calibration file not found at {calib_file}")
    else:
        logger.warning("HELLO_FLEET_PATH or HELLO_FLEET_ID not set. Cannot load calibration.")

    if output_dir is not None:
        if prefix is None:
            prefix = f"{model_name}_{batch_name}_{tool_name}"
        return generate_urdf_file(urdf_contents, prefix, output_dir, description)

    return urdf_contents
    

def get_urdf(
    model_name:str,
    batch_name:str,
    tool_name:str,
    do_add_file_prefix_to_absolute_paths: bool = True,
    output_dir: str|None = None,
    prefix: str|None = None,
    description: str = "unmodified"
    ):
    """
    Generates Robot URDF contents from the base xacro, and optionally saves it to a file if a directory is provided.

    Parameters
    ----------
    model_name : str
        Name of the model (e.g. SE4).
    batch_name : str
        Name of the batch (e.g. eames).
    tool_name : str
        Name of the tool (e.g. eoa_wrist_dw4_tool_sg4).
    do_add_file_prefix_to_absolute_paths : bool
        Whether to add the file:// prefix to the absolute paths of the meshes.
    output_dir : str
        Directory to save the output URDF file.
    prefix : str
        Prefix for the output URDF file.
    description : str
        Description of the output URDF file.
    
    Returns
    -------
    str
        raw urdf contents or the filepath the contents were saved to if an output_dir is provided
    """
    logger.info(f"Loading robot model: {model_name}, batch: {batch_name}, tool: {tool_name}")

    available_tools = get_available_tools(model_name)
    if tool_name not in available_tools: 
        raise ValueError(f"Unexpected tool for model {model_name}: {tool_name}\nTools available for {model_name}:\n"
            + "\n".join([f"\t{tool}" for tool in available_tools])
            + f"\nCheck robot model and tool settings or add a new tool to stretch4_urdf/{model_name}_tools")

    urdf_contents = generate_urdf_from_xacro(model_name, batch_name, tool_name, do_add_file_prefix_to_absolute_paths)

    if output_dir is not None:
        if prefix is None:
            prefix = f"{model_name}_{batch_name}_{tool_name}"
        return generate_urdf_file(urdf_contents, prefix, output_dir, description)

    return urdf_contents


def get_joint_limits(urdf_contents: str):
    """
    Parses the URDF contents and extracts lower and upper bounds for all joints that have them.

    Parameters
    ----------
    urdf_contents : str
        raw urdf contents

    Returns
    -------
    dict
        A dictionary mapping joint names to a tuple (lower, upper).
    """
    try:
        urdf = URDF.load(io.StringIO(urdf_contents))
        limits = {}
        for joint in urdf.robot.joints:
            if joint.limit:
                limits[joint.name] = (joint.limit.lower, joint.limit.upper)
        return limits
    except Exception as e:
        logger.warning(f"Failed to parse URDF for joint limits: {e}")
        return {}


def setup_logging():
    if RobotParams is not None and hello_utils is not None:
        try:
            _, robot_params = RobotParams.get_params()
            logging_params = robot_params['logging'].copy()
            # Update filename to be specific to this tool
            if 'file_handler' in logging_params['handlers']:
                logging_params['handlers']['file_handler']['filename'] = hello_utils.get_stretch_directory('log/stretch_body_logger/') + 'stretch4_urdf.log'
            logging.config.dictConfig(logging_params)
            return
        except Exception as e:
            pass

    # Fallback to basic configuration if robot_params cannot be loaded or stretch4_body is missing
    logging.basicConfig(
        level=logging.INFO,
        format="[%(asctime)s] [%(name)s] [%(levelname)s]: %(message)s",
        datefmt="%m/%d/%Y %H:%M:%S"
    )

def main():
    setup_logging()
    parser = argparse.ArgumentParser(
        description="Generate Robot URDF from the base xacro file."
    )

    parser.add_argument(
        "--prefix",
        type=str,
        default=None,
        help="Prefix for the output URDF files",
    )
    parser.add_argument(
        "--output-dir", 
        type=str, 
        default=None, 
        help="Folder to save the output"
    )

    model_name_param, batch_name_param, tool_name_param = get_robot_params()

    parser.add_argument(
        "--model", type=str, default=model_name_param, required=model_name_param is None, help="robot model name"
    
    )

    parser.add_argument(
        "--batch", type=str, default=batch_name_param, required=batch_name_param is None, help="robot batch name"
    
    )

    parser.add_argument(
        "--tool", type=str, default=tool_name_param, required=tool_name_param is None, help="robot tool name"
    
    )

    args = parser.parse_args()

    return get_urdf(
                model_name=args.model, 
                batch_name=args.batch, 
                tool_name=args.tool, 
                output_dir=args.output_dir, 
                prefix=args.prefix
            )


if __name__ == "__main__":
    print(main())
