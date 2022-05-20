import moderngl
from pyrr import Matrix44
from runner.scene_updater import update_pos_orns
import pybullet
from typing import Any, Tuple


class ModernGLRender:

    def __init__(self, prog = None, wnd = None, ctx= None, scn = None, **kwargs):
        super().__init__(**kwargs)

        self.aspect_ratio = 16/9
        self.wnd = wnd
        self.ctx = wnd.ctx
        self.prog = prog
        self.mvp = self.prog['Mvp']
        self.scn = scn


    def render(self, time, frame_time):
        pybullet.stepSimulation()
        self.ctx.clear(1.0, 1.0, 1.0)
        self.ctx.enable(moderngl.DEPTH_TEST)

        proj = Matrix44.perspective_projection(45.0, self.aspect_ratio, 0.1, 1000.0)
        lookat = Matrix44.look_at(
            (1.0,1.0,10.0),
            (0.0, 0.0, 0.0),
            (0.0, 1.0, 0.0))


        update_pos_orns(self.scn)

        for scene_item in self.scn:

            matrix = Matrix44.identity()
            matrix = matrix * Matrix44.from_translation(scene_item['position'])
            self.mvp.write((proj * lookat * matrix).astype('f4').tobytes())
            scene_item['part']['drawable']['vao'].render()


    def resize(self, width: int, height: int):
        pass

    def key_event(self, key: Any, action: Any):
        print("pressed", key)
        pass

    def mouse_position_event(self, x: int, y: int):
        pass


    def mouse_press_event(self, x: int, y: int, button: int):
        pass


    def mouse_release_event(self, x: int, y: int, button: int):
        pass