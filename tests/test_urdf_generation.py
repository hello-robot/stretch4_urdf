import glob
import importlib.resources as importlib_resources
import os
import tempfile
import unittest

from stretch4_urdf.utils.urdf_utils_generate_from_base_xacro import (
    get_available_tools, get_urdf)


class TestUrdfGeneration(unittest.TestCase):
    def test_generate_all_combinations(self):
        """
        Generates a temp urdf for all possible model, batch, and tool combinations,
        confirms the generated file is valid, then cleans up.
        """

        urdf_pkg_path = str(importlib_resources.files("stretch4_urdf"))
        
        # Find models from .xacro files in the root of the package
        xacro_files = glob.glob(os.path.join(urdf_pkg_path, '*.xacro'))
        models = [os.path.basename(f).replace('.xacro', '') for f in xacro_files]
        self.assertTrue(len(models) > 0, "Expected to find at least one model .xacro file.")

        with tempfile.TemporaryDirectory() as temp_dir:
            generated_count = 0
            
            for model in models:
                # Find batches for the current model
                # Batches are folders formatted as {model}_{batch} inside the pkg
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
                                
                            generated_count += 1
                            
            self.assertGreater(generated_count, 0, "No URDFs were generated during the test.")

if __name__ == '__main__':
    unittest.main()
