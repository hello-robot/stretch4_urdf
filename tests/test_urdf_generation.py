import glob
import importlib.resources as importlib_resources
import os
import tempfile
import unittest
import io 

from yourdfpy import urdf as ud
from stretch4_urdf.utils.urdf_utils_generate_from_base_xacro import (
    get_available_tools, get_urdf, get_urdf_calibrated)
from stretch4_urdf.utils.urdf_utils_generate_ik_urdfs import generate_ik_urdfs

class TestUrdfGeneration(unittest.TestCase):
    def _get_all_combinations(self):
        urdf_pkg_path = str(importlib_resources.files("stretch4_urdf"))
        
        # Find models from .xacro files in the root of the package
        xacro_files = glob.glob(os.path.join(urdf_pkg_path, '*.xacro'))
        models = [os.path.basename(f).replace('.xacro', '') for f in xacro_files]
        self.assertTrue(len(models) > 0, "Expected to find at least one model .xacro file.")

        combinations = []
        for model in models:
            # Find batches for the current model
            batch_dirs = glob.glob(os.path.join(urdf_pkg_path, f'{model}_*'))
            batches = []
            for d in batch_dirs:
                if os.path.isdir(d):
                    basename = os.path.basename(d)
                    # Skip the tools directory since it's not a batch
                    if basename == f"{model}_tools":
                        continue
                    batch = basename.replace(f'{model}_', '')
                    batches.append(batch)
            
            self.assertTrue(len(batches) > 0, f"Expected to find at least one batch for {model}.")
            
            tools = get_available_tools(model)
            self.assertTrue(len(tools) > 0, f"Expected to find at least one tool for {model}.")

            for batch in batches:
                for tool in tools:
                    combinations.append((model, batch, tool))
                    
        return combinations

    def test_get_urdf_for_all_combinations(self):
        """
        Generates a temp urdf for all possible model, batch, and tool combinations,
        confirms the generated file is valid.
        """
        combinations = self._get_all_combinations()
        self.assertGreater(len(combinations), 0, "Expected to find at least one valid combination")

        with tempfile.TemporaryDirectory() as temp_dir:
            generated_count = 0
            for model, batch, tool in combinations:
                with self.subTest(model=model, batch=batch, tool=tool):
                    filepath = get_urdf(
                        model_name=model,
                        batch_name=batch,
                        tool_name=tool,
                        output_dir=temp_dir
                    )
                    # Verify that the generated output exists
                    self.assertTrue(os.path.exists(filepath), f"URDF could not be generated for {model}_{batch}_{tool}")
                    
                    # Verify file contains basic valid XML/URDF properties
                    with open(filepath, 'r') as f:
                        contents = f.read()
                        self.assertIn('<robot', contents, "File does not contain a <robot> tag")
                        self.assertIn('</robot>', contents, "File does not close the </robot> tag")
                    
                    robot = ud.URDF.load(filepath)
                    self.assertIsNotNone(robot, "Failed to load generated URDF file with yourdfpy.")
                    generated_count += 1
                    
            self.assertGreater(generated_count, 0, "No URDFs were generated during the test.")

    def test_generate_ik_urdfs_for_all_combinations(self):
        """
        Tests the generate_ik_urdfs function for all valid combinations.
        """
        combinations = self._get_all_combinations()
        self.assertGreater(len(combinations), 0, "Expected to find at least one valid combination")

        with tempfile.TemporaryDirectory() as temp_dir:
            generated_count = 0
            for model, batch, tool in combinations:
                with self.subTest(model=model, batch=batch, tool=tool):
                    filepath = get_urdf(
                        model_name=model,
                        batch_name=batch,
                        tool_name=tool,
                        output_dir=temp_dir
                    )
                    robot = ud.URDF.load(filepath)

                    output_prefix = f"ik_test_{model}_{batch}_{tool}"
                    ik_filepaths = generate_ik_urdfs(
                        robot=robot, 
                        output_prefix=output_prefix, 
                        output_dir=temp_dir
                    )
                    
                    self.assertGreater(len(ik_filepaths), 0, f"Could not generate IK URDFs for {model}_{batch}_{tool}")
                    for ik_path in ik_filepaths:
                        self.assertTrue(os.path.exists(ik_path), f"IK URDF not generated for {ik_path}")
                        
                    generated_count += 1
            
            self.assertGreater(generated_count, 0, "No IK URDFs were generated during the test.")

    def test_get_urdf_calibrated_file_output(self):
        """
        Tests retrieving calibrated URDF and writing it to a file directly.
        """
        with tempfile.TemporaryDirectory() as temp_dir:
            filepath_calibrated = get_urdf_calibrated(
                model_name="SE4",
                batch_name="francis",
                tool_name="eoa_wrist_dw4_tool_sg4",
                output_dir=temp_dir
            )
            self.assertTrue(os.path.exists(filepath_calibrated), "Calibrated URDF could not be generated for SE4_francis_eoa_wrist_dw4_tool_sg4")
        
            with open(filepath_calibrated, 'r') as f:
                filepath_calibrated_contents = f.read()
                self.assertIn('<robot', filepath_calibrated_contents, "Calibrated file does not contain a <robot> tag")
                self.assertIn('</robot>', filepath_calibrated_contents, "Calibrated file does not close the </robot> tag")
            
            robot = ud.URDF.load(filepath_calibrated)
            self.assertTrue(isinstance(robot, ud.URDF), "Calibrated URDF file is not a valid URDF")

    def test_yourdf_loading_functionality(self):
        """
        Tests retrieving exactly the calibrated URDF string without writing to file.
        """
        urdf_calibrated_contents = get_urdf_calibrated(
            model_name="SE4",
            batch_name="francis",
            tool_name="eoa_wrist_dw4_tool_sg4",
        )
        self.assertTrue(type(urdf_calibrated_contents) == str)
        self.assertIn('<robot', urdf_calibrated_contents, "Calibrated urdf string does not contain a <robot> tag")
        self.assertIn('</robot>', urdf_calibrated_contents, "Calibrated urdf string does not close the </robot> tag")

        urdf_model = ud.URDF.load(io.StringIO(urdf_calibrated_contents))
        self.assertTrue(isinstance(urdf_model, ud.URDF), "Calibrated urdf string is not a valid URDF")

if __name__ == '__main__':
    unittest.main()
