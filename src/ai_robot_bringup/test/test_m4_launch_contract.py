from pathlib import Path


LAUNCH_FILE = Path(__file__).parents[1] / 'launch' / 'm4_bringup.launch.py'


def test_m4_entry_declares_mode_contract_and_full_simulation():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('mode', default_value='sim', choices=['sim', 'real'])" in source
    assert "'launch' / 'sensors.launch.py'" in source
    assert "condition=sim_condition" in source
    assert "PythonExpression([\"'\", mode, \"' == 'sim'\"])" in source


def test_safety_layer_is_declared_before_selected_implementation():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    expected_nodes = (
        "executable='base_status_node'",
        "executable='cmd_vel_safety_node'",
        "executable='health_reporter'",
    )
    implementation_index = source.index('simulation = IncludeLaunchDescription')
    for node in expected_nodes:
        assert source.index(node) < implementation_index


def test_simulation_uses_fused_odometry_as_the_only_tf_authority():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert "'controllers_m4.yaml'" in source
    assert "'output_topic': '/wheel/odom'" in source
    assert "executable='ekf_node'" in source
    assert "('/odometry/filtered', '/odom')" in source


def test_real_mode_fail_safe_contract_is_documented():
    source = LAUNCH_FILE.read_text(encoding='utf-8')

    assert 'Real mode remains fail-safe' in source
    assert 'starts no simulator' in source
