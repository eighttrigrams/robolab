import unittest

from moderngl_render.postproc_robot import postproc_robot


class TestPostprocRobot(unittest.TestCase):

    def test_make_drawables_from_model(self): # for now, one level will suffice

        robot = {
            'part': {
                'model': 'abc.obj'
            }
        }

        postproc_robot(lambda model: {'vao': 'vao_ref'}, robot)

        self.assertFalse('model' in robot['part'])
        self.assertTrue('drawable' in robot['part'])
        self.assertEqual('vao_ref', robot['part']['drawable']['vao'])

