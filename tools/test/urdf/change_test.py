import unittest

from tools.robot.change import set_position


class RobotChanges(unittest.TestCase):


    def test_set_new_robot_position(self):

        robot = {
            'id': 0,
            'position': (1.0, 1.0, 1.0),
            'rotation': (2.0, 2.0, 2.0),
            'part': {}
        }

        set_position(robot,
                     lambda id: ((2.0, 2.0, 2.0), (3.0, 3.0, 3.0)),
                     lambda x: x)

        self.assertEqual((2.0, 2.0, 2.0), robot['position'])
        self.assertEqual((3.0, 3.0, 3.0), robot['rotation'])


    def test_set_new_joint_position(self):

        robot = {
            'id': 0,
            'position': (1.0, 1.0, 1.0),
            'rotation': (2.0, 2.0, 2.0),
            'part': {
                'connects_to': [
                    {
                        'id': 1,
                        'position': (3.0, 3.0, 3.0),
                        'rotation': (4.0, 4.0, 4.0),
                        'part': {}
                    }
                ]
            }
        }

        set_position(robot,
                     lambda x: (None,None),
                     lambda entity_id, conn: ((5.0, 5.0, 5.0), (6.0, 6.0, 6.0)))

        self.assertEqual((5.0, 5.0, 5.0), robot['part']['connects_to'][0]['position'])
        self.assertEqual((6.0, 6.0, 6.0), robot['part']['connects_to'][0]['rotation'])