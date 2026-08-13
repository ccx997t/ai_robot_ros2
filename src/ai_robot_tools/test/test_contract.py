import ast
from pathlib import Path
import unittest


class WorkspaceContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        launch_file = (
            Path(__file__).resolve().parents[2]
            / 'ai_robot_bringup'
            / 'launch'
            / 'sim_bringup.launch.py'
        )
        cls.launch_tree = ast.parse(launch_file.read_text(encoding='utf-8'))

    def test_explicit_modes(self):
        declarations = [
            node for node in ast.walk(self.launch_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'DeclareLaunchArgument'
            and node.args
            and isinstance(node.args[0], ast.Constant)
            and node.args[0].value == 'mode'
        ]
        self.assertEqual(1, len(declarations))
        choices = next(
            keyword.value for keyword in declarations[0].keywords
            if keyword.arg == 'choices'
        )
        self.assertEqual(
            ['sim', 'real'],
            [element.value for element in choices.elts],
        )

    def test_foundation_nodes_receive_mode(self):
        nodes = [
            node for node in ast.walk(self.launch_tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'Node'
        ]
        packages = {
            next(
                keyword.value.value for keyword in node.keywords
                if keyword.arg == 'package'
            )
            for node in nodes
        }
        self.assertEqual({'ai_robot_base', 'ai_robot_tools'}, packages)
        for node in nodes:
            executable = next(
                keyword.value.value for keyword in node.keywords
                if keyword.arg == 'executable'
            )
            parameters = next(
                keyword.value for keyword in node.keywords
                if keyword.arg == 'parameters'
            )
            if executable not in {'cmd_vel_safety_node', 'odom_contract_relay'}:
                self.assertEqual('[parameters]', ast.unparse(parameters))

    def test_sim_time_is_derived_from_mode(self):
        source = ast.unparse(self.launch_tree)
        self.assertIn("'mode': mode", source)
        self.assertIn("'use_sim_time': use_sim_time", source)
        self.assertIn("value_type=bool", source)


if __name__ == '__main__':
    unittest.main()
