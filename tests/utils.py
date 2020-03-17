import unittest


class TestCaseExtra(unittest.TestCase):
    def assertAlmostEqualVector(self, first, second, places, msg):
        for f, s in zip(first, second):
            self.assertAlmostEqual(f, s, places, msg=msg)
