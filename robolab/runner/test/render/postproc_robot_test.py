import unittest

from runner.render.postproc_robot import postproc_robot


class TestURDFPostProc(unittest.TestCase):


    def test_texture_name_into_group(self):

        robot = {
            'part': {
                'model': 'mesh.obj'
            }
        }

        def load_object(name):
            return {
                'triangles': [()],
                'quads': [],
                'texture_names': ['abc.jpg']
            }

        postproc_robot(load_object, [0], robot)
        self.assertEqual(robot['part']['drawable']['groups'][0]['texture_name'], 'abc.jpg')


    def test_assign_random_color_if_no_texture_found(self):

        robot = {
            'part': {
                'model': 'mesh.obj'
            }
        }


        def load_object(name):
            return {
                'triangles': [()],
                'quads': [],
                'texture_names': []
            }

        postproc_robot(load_object, [0], robot)
        self.assertTrue('color' in robot['part']['drawable']['groups'][0], 'abc.jpg')


    def test_prefer_color_over_texture_if_defined_via_urdf_and_not_display_as_colored(self):

        robot = {
            'part': {
                'model': 'mesh.obj',
                'color': (0., 0., 0., 1.)
            }
        }

        def load_object(name):
            return {
                'triangles': [()],
                'quads': [],
                'texture_names': ['abc.jpg']
            }

        postproc_robot(load_object, [0], robot)
        self.assertTrue('texture_name' not in robot['part']['drawable']['groups'][0])
        self.assertEqual(robot['part']['drawable']['groups'][0]['color'], (0., 0., 0., 1.))


    def test_recurse_into_nested_structure(self):

        robot = {
            'part': {
                'model': 'box',
                'connects_to': [
                    {
                        'part': {
                            'model': 'mesh.obj',
                            'connects_to': [
                                {
                                    'part': {
                                        'model': 'cylinder'
                                    }
                                }
                            ]
                        },
                    }
                ]
            }
        }

        def load_object(name):
            return {
                'vertices': [],
                'triangles': [],
                'quads': [],
                'texture_names': ['abc.jpg']
            }

        postproc_robot(load_object, [0], robot)
        self.assertEqual(robot['part']['connects_to'][0]['part']['drawable']['groups'][0]['texture_name'], 'abc.jpg')
