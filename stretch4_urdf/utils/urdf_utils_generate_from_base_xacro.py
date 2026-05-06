import argparse
import importlib.resources as importlib_resources
import os

from xacrodoc import XacroDoc


def get_available_tools(model_name:str):

    available_tools = os.listdir(os.path.join(importlib_resources.files("stretch4_urdf"), f"{model_name}_tools"))
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
    tool_mesh_dir = os.path.join(urdf_pkg_path, f"{model_name}_tools/{tool_name}/meshes")
    
    if not os.path.exists(tool_mesh_dir) or not os.path.exists(model_mesh_dir):
        raise FileNotFoundError(f"Failed to resolve mesh directories:\n\t{tool_mesh_dir}\n\t{model_mesh_dir}\nIf paths are pointing to an old location, trying re-installing stretch4_urdf to update the paths and re-sourcing your workspace.")

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
        tuple[str, str, str]: model_name, batch_name, tool_name
    """
    try:
        from stretch4_body.core.robot_params import RobotParams
        _, robot_params = RobotParams.get_params()
        model_name = robot_params["robot"]["model_name"]
        batch_name = robot_params["robot"]["batch_name"]
        tool_name = robot_params["robot"]["tool"]
        return model_name, batch_name, tool_name
    except:
        raise Exception(
            "stretch4_body.core.robot_params not found. If you are not running this on a robot, use the --filepath argument or use get_urdf() with the model, batch and tool name parameters."
        )

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

def get_urdf_from_robot_params(do_add_file_prefix_to_absolute_paths: bool = True, output_dir: str|None = None, prefix: str|None = None,):
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
    return get_urdf(model_name, batch_name, tool_name, do_add_file_prefix_to_absolute_paths, output_dir=output_dir, prefix=prefix)

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

    print(f"GETTING URDF FOR MODEL: {model_name}, BATCH: {batch_name}, TOOL: {tool_name}")
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


def main():

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
        "--model", type=str, default=model_name_param, help="robot model name"
    
    )

    parser.add_argument(
        "--batch", type=str, default=batch_name_param, help="robot batch name"
    
    )

    parser.add_argument(
        "--tool", type=str, default=tool_name_param, help="robot tool name"
    
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
