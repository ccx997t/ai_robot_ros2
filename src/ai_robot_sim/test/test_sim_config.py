from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]


def test_world_is_valid_sdf_with_required_systems():
    root = ET.parse(PACKAGE_DIR / 'worlds' / 'm2_test.sdf').getroot()
    world = root.find('world')
    filenames = {plugin.attrib['filename'] for plugin in world.findall('plugin')}
    assert 'ignition-gazebo-physics-system' in filenames
    assert 'ignition-gazebo-user-commands-system' in filenames
    assert world.find("model[@name='ground_plane']") is not None


def test_diff_drive_controller_contract():
    config = yaml.safe_load((PACKAGE_DIR / 'config' / 'controllers.yaml').read_text())
    params = config['base_controller']['ros__parameters']
    assert params['left_wheel_names'] == ['left_wheel_joint']
    assert params['right_wheel_names'] == ['right_wheel_joint']
    assert params['odom_frame_id'] == 'odom'
    assert params['base_frame_id'] == 'base_link'
    assert params['enable_odom_tf'] is True
    assert params['publish_rate'] >= 20.0
    assert params['cmd_vel_timeout'] == 1000.0
    assert params['linear.x.max_velocity'] == 0.30
    assert params['angular.z.max_velocity'] == 0.80
