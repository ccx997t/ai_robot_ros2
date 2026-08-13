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
    assert 'ignition-gazebo-sensors-system' in filenames
    assert 'ignition-gazebo-imu-system' in filenames
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


def test_sensor_default_parameters():
    config = yaml.safe_load((PACKAGE_DIR / 'config' / 'sensors.yaml').read_text())
    assert config['lidar']['topic'] == '/scan'
    assert config['lidar']['frame_id'] == 'laser_link'
    assert config['lidar']['update_rate'] >= 5.0
    assert config['camera']['image_topic'] == '/camera/image_raw'
    assert config['camera']['camera_info_topic'] == '/camera/camera_info'
    assert config['camera']['frame_id'] == 'camera_optical_link'
    assert config['camera']['width'] == 320 and config['camera']['height'] == 240
    assert config['imu']['topic'] == '/imu/data'
    assert config['imu']['frame_id'] == 'imu_link'
    assert config['imu']['update_rate'] >= 50.0


def test_encoder_default_parameters():
    config = yaml.safe_load((PACKAGE_DIR / 'config' / 'encoders.yaml').read_text())
    encoder = config['encoder']
    assert encoder['topic'] == '/joint_states'
    assert encoder['message_type'] == 'sensor_msgs/msg/JointState'
    assert encoder['left_joint'] == 'left_wheel_joint'
    assert encoder['right_joint'] == 'right_wheel_joint'
    assert encoder['position_unit'] == 'rad'
    assert encoder['velocity_unit'] == 'rad/s'
    assert encoder['update_rate'] == 100.0
    assert encoder['ticks_per_revolution'] > 0
    assert encoder['radians_per_tick'] > 0.0
    assert encoder['simulated_continuous_position'] is True


def test_encoder_has_independent_launch_entry():
    launch_text = (PACKAGE_DIR / 'launch' / 'encoder.launch.py').read_text()
    assert 'sim_base.launch.py' in launch_text
    assert "'sensor_type': 'joint_state'" in launch_text
    assert "'diagnostic_name': 'encoder'" in launch_text
    assert "'monitor_only': True" in launch_text
    assert "'enable_lidar': 'false'" in launch_text
    assert "'enable_camera': 'false'" in launch_text
    assert "'enable_imu': 'false'" in launch_text
