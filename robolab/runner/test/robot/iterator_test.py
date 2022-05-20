import unittest

from runner.robot.iterator import iterate_robot


class TestRobotIterator(unittest.TestCase):


    def test_iterate(self):

        robot1 = {
            'part': {
                'connects_to': [
                    {
                        'part': {

                        }
                    }
                ]
            }
        }

        def on_limb(limb, color, color2):
            pass

        def on_joint(joint, color, color2):
            # print(color)
            # print(color2)
            pass

        iterate_robot(robot1,
                      on_limb,
                      on_joint,
                      'red',
                      'green')