from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def load_params(node):
    content = yaml.safe_load((ROOT / 'config' / 'nav2_m5.yaml').read_text())
    return content[node]['ros__parameters']


def test_planner_controller_and_safety_limits():
    planner = load_params('planner_server')
    controller = load_params('controller_server')
    follow = controller['FollowPath']
    assert planner['planner_plugins'] == ['GridBased']
    assert planner['GridBased']['plugin'].endswith('NavfnPlanner')
    assert planner['GridBased']['allow_unknown'] is False
    assert controller['controller_plugins'] == ['FollowPath']
    assert follow['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert -0.30 <= follow['min_vel_x'] <= 0.0
    assert 0.0 < follow['max_vel_x'] <= 0.30
    assert 0.0 < follow['max_vel_theta'] <= 0.80
    assert follow['acc_lim_x'] <= 0.60
    assert follow['acc_lim_theta'] <= 1.60


def test_global_and_local_costmap_contract():
    content = yaml.safe_load((ROOT / 'config' / 'nav2_m5.yaml').read_text())
    local = content['local_costmap']['local_costmap']['ros__parameters']
    global_map = content['global_costmap']['global_costmap'][
        'ros__parameters']
    expected_footprint = (
        '[[-0.26, -0.20], [-0.26, 0.20], '
        '[0.26, 0.20], [0.26, -0.20]]')
    assert local['global_frame'] == 'odom'
    assert local['rolling_window'] is True
    assert local['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert global_map['global_frame'] == 'map'
    assert global_map['plugins'] == [
        'static_layer', 'obstacle_layer', 'inflation_layer']
    assert local['footprint'] == expected_footprint
    assert global_map['footprint'] == expected_footprint
    assert local['obstacle_layer']['scan']['topic'] == '/scan'
    assert global_map['obstacle_layer']['scan']['topic'] == '/scan'
    assert local['inflation_layer']['inflation_radius'] >= 0.40
    assert global_map['inflation_layer']['inflation_radius'] >= 0.40


def test_navigation_entry_and_lifecycle_contract():
    source = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "choices=['sim', 'real']" in source
    assert "'localization.launch.py'" in source
    assert "('nav2_planner', 'planner_server'" in source
    assert "('nav2_controller', 'controller_server'" in source
    assert "('nav2_behaviors', 'behavior_server'" in source
    assert "('nav2_bt_navigator', 'bt_navigator'" in source
    assert 'lifecycle_manager_navigation' in source
    assert 'slam_toolbox' not in source


def test_navigation_fault_recovery_entry_is_isolated_and_recoverable():
    source = (
        ROOT / 'launch' / 'navigation_fault_recovery.launch.py').read_text()
    navigation = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "choices=['sim', 'real']" in source
    assert "'scan_output_topic': scan_source" in source
    assert "'fault_mode': 'drop'" in source
    assert "'input_topic': '/scan'" in source
    assert "'diagnostic_name': 'navigation_lidar'" in source
    assert "'monitor_only': True" in source
    assert "DeclareLaunchArgument('scan_output_topic'" in navigation
