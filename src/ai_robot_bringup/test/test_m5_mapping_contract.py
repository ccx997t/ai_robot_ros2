from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).parents[1]
MAPPING_LAUNCH = PACKAGE_DIR / 'launch' / 'mapping.launch.py'


def test_mapping_entry_is_mode_explicit_and_uses_m5_world():
    source = MAPPING_LAUNCH.read_text(encoding='utf-8')
    assert "'mode', default_value='sim', choices=['sim', 'real']" in source
    assert "'worlds' / 'm5_navigation.sdf'" in source
    assert "'launch' / 'm4_bringup.launch.py'" in source
    assert "condition=sim_condition" in source
    assert 'starts neither simulation nor SLAM' in source


def test_slam_is_the_mapping_tf_authority_and_uses_project_parameters():
    source = MAPPING_LAUNCH.read_text(encoding='utf-8')
    assert "package='slam_toolbox'" in source
    assert "executable='async_slam_toolbox_node'" in source
    assert "name='slam_toolbox'" in source
    assert "'slam_toolbox_m5.yaml'" in source
    assert "executable='map_saver_server'" in source
    assert "'node_names': ['map_saver']" in source

    config = yaml.safe_load(
        (PACKAGE_DIR / 'config' / 'slam_toolbox_m5.yaml').read_text())
    params = config['slam_toolbox']['ros__parameters']
    assert params['mode'] == 'mapping'
    assert params['map_frame'] == 'map'
    assert params['odom_frame'] == 'odom'
    assert params['base_frame'] == 'base_link'
    assert params['scan_topic'] == '/scan'
    assert params['use_sim_time'] is True
    assert params['resolution'] == 0.05
    assert params['max_laser_range'] <= 12.0
    assert 0.0 < params['transform_publish_period'] <= 0.1
    assert params['map_update_interval'] <= 2.0
    assert params['use_map_saver'] is True
    assert params['enable_interactive_mode'] is False


def test_world_override_is_forwarded_through_existing_bringup_chain():
    m4_source = (PACKAGE_DIR / 'launch' / 'm4_bringup.launch.py').read_text()
    sensors_source = (PACKAGE_DIR.parents[0] / 'ai_robot_sim' / 'launch' /
                      'sensors.launch.py').read_text()
    base_source = (PACKAGE_DIR.parents[0] / 'ai_robot_sim' / 'launch' /
                   'sim_base.launch.py').read_text()
    for source in (m4_source, sensors_source, base_source):
        assert "'world_file'" in source
    assert "LaunchConfiguration('world_file')" in base_source
