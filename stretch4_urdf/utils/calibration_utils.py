import os
import yaml
import xml.etree.ElementTree as ET
import logging
from datetime import datetime
CALIBRATION_FORMAT_VERSION = '2.0'

def get_calibration_values_filepath(fleet_id=None):
    """
    Get the default path to the stretch_calibration_values.yaml for a given fleet ID.
    If no fleet_id is provided, checks the HELLO_FLEET_ID environment variable.
    """
    if fleet_id is None:
        fleet_id = os.environ.get('HELLO_FLEET_ID', 'unknown_robot')
    return os.path.join(os.path.expanduser('~/stretch_user'), fleet_id, 'stretch_calibration_values.yaml')

def create_calibration_values_file(filepath=None, fleet_id=None):
    """
    Creates or reinitializes a stretch_calibration_values.yaml file with the given version.
    """
    if filepath is None:
        filepath = get_calibration_values_filepath(fleet_id)
        
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    
    data = {
        'version': CALIBRATION_FORMAT_VERSION,
        'robot_calibration': {}
    }
    with open(filepath, 'w') as f:
        yaml.dump(data, f, sort_keys=False)
        
    return filepath

def add_calibration_joint(joint_name, xyz, rpy, parent, child, robot_id, timestamp=None, extra=None, filepath=None, fleet_id=None):
    """
    Adds a joint entry to the calibration file. Creates the file and necessary structure if it doesn't exist.
    """
    if filepath is None:
        filepath = get_calibration_values_filepath(fleet_id)
        
    if timestamp is None:
        timestamp = datetime.now().isoformat()
        
    urdf_data = {'version': CALIBRATION_FORMAT_VERSION, 'robot_calibration': {}}
    
    if os.path.exists(filepath):
        try:
            with open(filepath, 'r') as f:
                loaded_urdf = yaml.safe_load(f)
                if loaded_urdf:
                    if 'robot_calibration' in loaded_urdf:
                        urdf_data['robot_calibration'] = loaded_urdf['robot_calibration']
                    if 'version' in loaded_urdf:
                        urdf_data['version'] = loaded_urdf['version']
        except Exception as e:
            print(f"Warning: Failed to load existing URDF calibration values: {e}")
            
    joints = urdf_data['robot_calibration']
    
    joint_entry = {
        'data': {
            'xyz': xyz,
            'rpy': rpy,
            'parent': parent,
            'child': child
        },
        'robot_id': robot_id,
        'timestamp': timestamp
    }
    
    if extra:
        joint_entry.update(extra)
        
    joints[joint_name] = joint_entry
    
    os.makedirs(os.path.dirname(filepath), exist_ok=True)
    with open(filepath, 'w') as f:
        yaml.dump(urdf_data, f, sort_keys=False)

def apply_calibration_to_urdf_v1(urdf_contents, calib_data, logger=None):
    if logger is None:
        logger = logging.getLogger(__name__)
        
    root = ET.fromstring(urdf_contents)
    
    if calib_data and "robot_calibration" in calib_data and "joints" in calib_data["robot_calibration"]:
        joints_calib = calib_data["robot_calibration"]["joints"]
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name in joints_calib:
                logger.debug(f"Applying calibration to {name}")
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
        
    return urdf_contents

def apply_calibration_to_urdf_v2(urdf_contents, calib_data, logger=None):
    if logger is None:
        logger = logging.getLogger(__name__)
        
    root = ET.fromstring(urdf_contents)
    
    if calib_data and "robot_calibration" in calib_data:
        joints_calib = calib_data["robot_calibration"]
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name in joints_calib:
                logger.debug(f"Applying calibration to {name}")
                joint_entry = joints_calib[name]
                
                if 'data' not in joint_entry:
                    logger.warning(f"Missing 'data' block for joint '{name}' in calibration. Skipping.")
                    continue
                    
                joint_data = joint_entry['data']
                
                # Confirm that the joint in URDF has the same parent link as specified in the calibration
                parent_elem = joint.find('parent')
                if parent_elem is not None and 'parent' in joint_data:
                    if parent_elem.get('link') != joint_data['parent']:
                        logger.warning(f"Parent link mismatch for joint '{name}'. Expected: {joint_data['parent']}, but found: {parent_elem.get('link')}. Skipping calibration.")
                        continue
                
                origin = joint.find('origin')
                if origin is None:
                    origin = ET.SubElement(joint, 'origin')
                
                if 'xyz' in joint_data:
                    origin.set('xyz', str(joint_data['xyz']))
                if 'rpy' in joint_data:
                    origin.set('rpy', str(joint_data['rpy']))
        
        urdf_contents = ET.tostring(root, encoding='unicode')
        
    return urdf_contents

def apply_calibration_to_urdf(urdf_contents, logger=None):
    if logger is None:
        logger = logging.getLogger(__name__)
        
    fleet_path = os.environ.get("HELLO_FLEET_PATH")
    fleet_id = os.environ.get("HELLO_FLEET_ID")
    
    if fleet_path and fleet_id:
        calib_file = os.path.join(fleet_path, fleet_id, "stretch_calibration_values.yaml")
        if os.path.exists(calib_file):
            with open(calib_file, 'r') as f:
                calib_data = yaml.safe_load(f)
                
            version = None
            if calib_data is not None:
                if "version" in calib_data:
                    version = str(calib_data["version"])
                elif "robot_calibration" in calib_data and isinstance(calib_data["robot_calibration"], dict) and "metadata" in calib_data["robot_calibration"] and "version" in calib_data["robot_calibration"]["metadata"]:
                    version = str(calib_data["robot_calibration"]["metadata"]["version"])
                    
            if version is None:
                logger.warning(f"Calibration file missing version information: {calib_file}. Using nominal URDF values.")
            elif version.startswith("2"):
                return apply_calibration_to_urdf_v2(urdf_contents, calib_data, logger)
            elif version.startswith("1"):
                return apply_calibration_to_urdf_v1(urdf_contents, calib_data, logger)
            else:
                logger.warning(f"Unsupported calibration version: {version} in {calib_file}. Using nominal URDF values.")
        else:
            logger.debug(f"Calibration file not found at {calib_file}. Using nominal URDF values.")
    else:
        logger.debug("HELLO_FLEET_PATH or HELLO_FLEET_ID not set. Using nominal URDF values.")
        
    return urdf_contents
