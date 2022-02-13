import unittest

from state_representation import Shape, Ellipsoid

SHAPE_METHOD_EXPECTS = [
    'get_center_state',
    'get_center_pose',
    'get_center_position',
    'get_center_orientation',
    'get_center_twist',
    'set_center_state',
    'set_center_pose',
    'set_center_position',
    'set_center_orientation'
]

ELLIPSOID_METHOD_EXPECTS = [
    'get_axis_lengths',
    'get_axis_length',
    'set_axis_lengths',
    'get_rotation_angle',
    'set_rotation_angle',
    'get_rotation',
    'sample_from_parameterization',
    'from_algebraic_equation',
    'fit',
    'to_std_vector'
]


class TestState(unittest.TestCase):

    def test_callable_methods(self):
        methods = [m for m in dir(Shape) if callable(getattr(Shape, m))]
        for expected in SHAPE_METHOD_EXPECTS:
            self.assertIn(expected, methods)

        methods = [m for m in dir(Ellipsoid) if callable(getattr(Ellipsoid, m))]
        for expected in ELLIPSOID_METHOD_EXPECTS:
            self.assertIn(expected, methods)

    def test_sampling(self):
        ellipse = Ellipsoid("test")
        points = ellipse.sample_from_parameterization(100)
        for point in points:
            x = point.get_position()[0]
            y = point.get_position()[1]
            d = x * x + y * y - 1.0
            self.assertTrue(abs(d) < 1e-3)

        ellipse.set_axis_lengths([3., 0.5, 0.])
        points = ellipse.sample_from_parameterization(100)
        for point in points:
            x = point.get_position()[0]
            y = point.get_position()[1]
            d = x * x / 9.0 + y * y / 0.25 - 1.0
            self.assertTrue(abs(d) < 1e-3)


if __name__ == '__main__':
    unittest.main()
