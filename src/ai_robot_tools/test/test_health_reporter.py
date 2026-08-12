import unittest

from diagnostic_msgs.msg import DiagnosticStatus

from ai_robot_tools.health_reporter import build_status


class HealthReporterContractTest(unittest.TestCase):
    def test_status_is_structured_and_safe(self):
        status = build_status('sim')

        self.assertEqual(DiagnosticStatus.OK, status.level)
        self.assertEqual('ai_robot_tools/health_reporter', status.name)
        self.assertEqual('foundation', status.hardware_id)
        self.assertIn('hardware control is disabled', status.message)
        self.assertEqual(
            {
                'mode': 'sim',
                'hardware_control': 'disabled',
            },
            {value.key: value.value for value in status.values},
        )


if __name__ == '__main__':
    unittest.main()
