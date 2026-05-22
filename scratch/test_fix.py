import xml.etree.ElementTree as ET
import os

# Mock data
root_xml = """
<robot name="stretch">
  <link name="link_wrist" />
  <link name="link_wrist_pitch" />
  <joint name="joint_wrist">
    <parent link="link_arm" />
    <child link="link_wrist" />
  </joint>
  <joint name="joint_wrist_pitch">
    <parent link="link_wrist" />
    <child link="link_wrist_pitch" />
  </joint>
  <link name="link_wrist_pitch">
    <visual>
      <geometry>
        <mesh filename="package://meshes/link_wrist_pitch.STL" />
      </geometry>
    </visual>
  </link>
</robot>
"""

root = ET.fromstring(root_xml)

def rename_everywhere(old_str, new_str):
    print(f"Renaming '{old_str}' to '{new_str}'")
    if not old_str or not new_str or old_str == new_str: return
    # XML
    for tag in root.iter():
        for attr in ['name', 'link']:
            val = tag.get(attr)
            if val == old_str:
                tag.set(attr, new_str)
                print(f"  Changed {tag.tag}.{attr} to {new_str}")
        
        # For filename, use substring replacement
        val = tag.get('filename')
        if val and old_str in val:
            new_val = val.replace(old_str, new_str)
            tag.set('filename', new_val)
            print(f"  Changed {tag.tag}.filename to {new_val}")

renames = {
    "link_wrist": "wrist_link",
    "link_wrist_pitch": "wrist_pitch_link",
    "joint_wrist": "wrist_joint",
    "joint_wrist_pitch": "wrist_pitch_joint"
}

# Sort by length descending
for old_name in sorted(renames.keys(), key=len, reverse=True):
    rename_everywhere(old_name, renames[old_name])

print("\nFinal XML:")
print(ET.tostring(root, encoding='unicode'))
