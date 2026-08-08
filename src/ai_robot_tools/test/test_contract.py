import unittest
class WorkspaceContractTest(unittest.TestCase):
    def test_explicit_modes(self):
        self.assertEqual({'sim', 'real'}, {'sim', 'real'})
if __name__ == '__main__':
    unittest.main()
