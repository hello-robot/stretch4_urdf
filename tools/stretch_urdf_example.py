#! /usr/bin/env python3


import yourdfpy as urdf_loader
import time
try:
    import stretch4_body.robot.robot
except Exception:
    stretch4_body = None
try:
    # works on ubuntu 22.04
    import importlib.resources as importlib_resources
    if stretch4_body is not None:
        str(importlib_resources.files("stretch4_body"))
except (AttributeError, ModuleNotFoundError) as e:
    # works on ubuntu 20.04
    import importlib_resources
    if stretch4_body is not None:
        try:
            str(importlib_resources.files("stretch4_body"))
        except ModuleNotFoundError:
            pass


def get_configuration(robot):
    s = robot.get_status()
    configuration = {
        'wheel_0_joint': 0.0,
        'wheel_1_joint': 0.0,
        'wheel_2_joint': 0.0,
        'lift_joint': s['lift']['pos'],}
        # 'arm_l0_joint': s['arm']['pos'] / 4.0,
        # 'arm_l1_joint': s['arm']['pos'] / 4.0,
        # 'arm_l2_joint': s['arm']['pos'] / 4.0,
        # 'arm_l3_joint': s['arm']['pos'] / 4.0,
        # 'wrist_yaw_joint': s['end_of_arm']['wrist_yaw']['pos'],
        # 'head_pan_joint': s['head']['head_pan']['pos'],
        # 'head_tilt_joint': s['head']['head_tilt']['pos']}
    return configuration

def main():
    if stretch4_body is None:
        print("Error: stretch4_body not found. This example requires a robot.")
        return
    r = stretch4_body.robot.robot.Robot()
    r.startup()

    pkg = str(importlib_resources.files("stretch4_urdf"))  # eg .local/lib/python3.10/site-packages/stretch_urdf)
    model_name = r.params['model_name']
    tool_name = r.params['tool']
    batch_name = r.params['batch_name']
    urdf_name = pkg + '/%s_%s/stretch_description_%s_%s.urdf' % (model_name,batch_name, model_name, tool_name)
    urdf = urdf_loader.URDF.load(urdf_name)
    links = ['base_link','mast_link']

    try:
        while True:
            lfk = urdf.fk_link(cfg=get_configuration(r), links=links, use_names=True)
            print('###############################3')
            for ll in links:
                print('----------- %s ---------'%ll.upper())
                print(lfk[ll])
            time.sleep(0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    r.stop()

if __name__ == "__main__":
    main()
