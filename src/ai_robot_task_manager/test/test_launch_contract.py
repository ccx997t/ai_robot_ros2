from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_launch_has_explicit_modes_and_safe_unintegrated_default():
    launch = (ROOT / 'launch' / 'task_manager.launch.py').read_text()
    assert "DeclareLaunchArgument('mode', default_value='sim')" in launch
    assert "mode not in ('sim', 'real')" in launch
    assert "'navigation_enabled': False" in launch


def test_node_exposes_public_action_without_bottom_command_output():
    node = (
        ROOT / 'ai_robot_task_manager' / 'task_manager_node.py').read_text()
    assert "'/execute_robot_task'" in node
    assert 'ActionServer(' in node
    assert '/cmd_vel' not in node
    assert '/base_controller/cmd_vel_unstamped' not in node
