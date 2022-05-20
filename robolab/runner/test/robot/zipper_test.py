import unittest

from runner.robot.zipper import zip_robot


class RobotChanges(unittest.TestCase):


    def test_set_new_robot_position(self):

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

        robot2 = {
            'part': {
                'connects_to': [
                    {
                        'part': {
                            'display_as': 'colored'
                        }
                    }
                ]
            }
        }

        zip_robot(robot1, robot2)

        self.assertEqual(robot1['part']['connects_to'][0]['part']['display_as'], 'colored')