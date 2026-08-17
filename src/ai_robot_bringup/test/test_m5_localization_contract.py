from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_amcl_frame_input_and_particle_contract():
    params = yaml.safe_load(
        (ROOT / 'config' / 'amcl_m5.yaml').read_text())['amcl'][
            'ros__parameters']
    assert params['global_frame_id'] == 'map'
    assert params['odom_frame_id'] == 'odom'
    assert params['base_frame_id'] == 'base_link'
    assert params['scan_topic'] == '/scan'
    assert params['tf_broadcast'] is True
    assert params['robot_model_type'].endswith('DifferentialMotionModel')
    assert params['min_particles'] >= 500
    assert params['max_particles'] >= params['min_particles']
    assert params['always_reset_initial_pose'] is True


def test_localization_entry_separates_slam_and_amcl_authority():
    source = (ROOT / 'launch' / 'localization.launch.py').read_text()
    assert "choices=['sim', 'real']" in source
    assert "package='nav2_amcl'" in source
    assert "package='nav2_map_server'" in source
    assert "node_names': ['m5_map_server', 'amcl']" in source
    assert 'slam_toolbox' not in source
    assert "'world_file'" in source
