import unittest

from runner.render.obj_loader import load_object


class TestObjLoader(unittest.TestCase):

    def test_basics(self):
        obj = load_object('runner/resources/', 'quad_plane_untextured.obj')
        self.assertTrue(len(obj['quads']) > 0)
        self.assertTrue(len(obj['triangles']) == 0)
