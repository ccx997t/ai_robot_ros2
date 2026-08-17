from pathlib import Path
import math
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).parents[1]


def _box_contract(model):
    pose = [float(value) for value in model.find('pose').text.split()]
    size = [float(value) for value in model.find(
        'link/collision/geometry/box/size').text.split()]
    return pose, size


def _point_clear_of_box(x, y, pose, size, clearance=0.0):
    return (abs(x - pose[0]) > size[0] / 2.0 + clearance or
            abs(y - pose[1]) > size[1] / 2.0 + clearance)


def test_world_is_valid_sdf_with_required_systems():
    root = ET.parse(PACKAGE_DIR / 'worlds' / 'm2_test.sdf').getroot()
    world = root.find('world')
    filenames = {plugin.attrib['filename'] for plugin in world.findall('plugin')}
    assert 'ignition-gazebo-physics-system' in filenames
    assert 'ignition-gazebo-user-commands-system' in filenames
    assert 'ignition-gazebo-sensors-system' in filenames
    assert 'ignition-gazebo-imu-system' in filenames
    assert world.find("model[@name='ground_plane']") is not None
    target = world.find("model[@name='lidar_target']")
    assert target is not None
    assert target.find('pose').text.split()[:3] == ['2', '0', '0.5']
    target_size = target.find(
        "link/collision/geometry/box/size").text.split()
    assert target_size == ['0.5', '0.5', '1.0']


def test_m5_navigation_world_matches_scenario_contract():
    contract = yaml.safe_load(
        (PACKAGE_DIR / 'config' / 'm5_scenario.yaml').read_text())['scenario']
    root = ET.parse(PACKAGE_DIR / 'worlds' / contract['world_file']).getroot()
    world = root.find('world')
    assert world.attrib['name'] == contract['name'] == 'm5_navigation'
    assert contract['frame_id'] == 'map'

    filenames = {plugin.attrib['filename'] for plugin in world.findall('plugin')}
    assert {
        'ignition-gazebo-physics-system',
        'ignition-gazebo-user-commands-system',
        'ignition-gazebo-scene-broadcaster-system',
        'ignition-gazebo-sensors-system',
        'ignition-gazebo-imu-system',
    }.issubset(filenames)
    assert world.find("model[@name='ground_plane']") is not None

    static_boxes = {}
    for name, expected in contract['static_models'].items():
        model = world.find(f"model[@name='{name}']")
        assert model is not None, name
        assert model.find('static').text == 'true'
        pose, size = _box_contract(model)
        assert pose[:3] == expected['pose'][:3]
        assert math.isclose(pose[5], expected['pose'][3], abs_tol=1e-12)
        assert size == expected['size']
        static_boxes[name] = (pose, size)

    footprint = contract['robot']['footprint']
    assert footprint == [
        [-0.26, -0.20], [-0.26, 0.20],
        [0.26, 0.20], [0.26, -0.20],
    ]
    boundary = contract['boundary']
    points = [contract['robot']['spawn'], *contract['goals'].values()]
    obstacle_pose = contract['temporary_obstacle']['pose']
    points.append(obstacle_pose)
    for point in points:
        assert boundary['x_min'] < point['x'] < boundary['x_max']
        assert boundary['y_min'] < point['y'] < boundary['y_max']

    clearance = contract['robot']['minimum_static_clearance']
    clear_points = [contract['robot']['spawn']]
    clear_points.extend(goal for goal in contract['goals'].values()
                        if goal['reachable'])
    clear_points.append(obstacle_pose)
    interior_boxes = {
        name: box for name, box in static_boxes.items()
        if not name.startswith('boundary_')
    }
    for point in clear_points:
        assert all(_point_clear_of_box(
            point['x'], point['y'], pose, size, clearance)
            for pose, size in interior_boxes.values())

    pocket = contract['goals']['enclosed_pocket']
    assert pocket['reachable'] is False
    assert -4.0 < pocket['x'] < -2.4
    assert -3.2 < pocket['y'] < -1.8
    assert all(contract['goals'][name]['reachable'] for name in (
        'east_room', 'north_room', 'west_bay'))

    temporary = contract['temporary_obstacle']
    assert temporary['initially_present'] is False
    assert temporary['size'] == [0.7, 0.7, 0.8]
    assert world.find(f"model[@name='{temporary['name']}']") is None


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
