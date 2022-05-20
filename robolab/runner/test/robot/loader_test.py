import unittest

from runner.robot.helper import generate_urdf
from runner.robot.loader import read_urdf


class TestObjLoader(unittest.TestCase):


    def test_read_scale_and_mesh_from_collision_geometry(self):

        URDF = '''<?xml version="1.0"?>
        <robot name="test">
          <link name="base_link">
              <collision>
                <geometry>
                  <mesh filename="FILENAME" scale="2 2 2"/>
                </geometry>
              </collision>
            </link>
        </robot>
        '''

        generate_urdf('runner/resources/', 'testrobot.obj', URDF)
        robot = read_urdf('runner/resources/.urdf.swp')
        self.assertEqual(robot['part']['model'], 'testrobot.obj')
        self.assertEqual(robot['part']['scaling'], (2., 2., 2.))


    def test_prefer_visual_over_collision_geometry(self):

        URDF = '''<?xml version="1.0"?>
        <robot name="test">
          <link name="base_link">
              <collision>
                <geometry>
                  <mesh filename="FILENAME" scale="2 2 2"/>
                </geometry>
              </collision>
              <visual>
                <geometry>
                  <mesh filename="visual.obj" scale="4 4 4"/>
                </geometry>
              </visual>
            </link>
        </robot>
        '''

        generate_urdf('runner/resources/', 'testrobot.obj', URDF)
        robot = read_urdf('runner/resources/.urdf.swp')
        self.assertEqual(robot['part']['model'], 'visual.obj')
        self.assertEqual(robot['part']['scaling'], (4., 4., 4.))


    def test_raise_if_no_geometry(self):

        URDF = '''<?xml version="1.0"?>
        <robot name="test">
          <link name="base_link">
          </link>
        </robot>
        '''

        with self.assertRaises(Exception):
            generate_urdf('runner/resources/', 'testrobot.obj', URDF)
            robot = read_urdf('runner/resources/.urdf.swp')
