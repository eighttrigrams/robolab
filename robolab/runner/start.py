from runner.scene_loader import *
from runner.config import *
from runner.render.postproc_robot import postproc_robot as render_postproc_robot, upload_base_textures
from runner.render.obj_loader import load_object
from runner.render2.postproc_robot import postproc_robot as moderngl_render_postproc_robot
from runner.render.texture_repository import upload_textures_for_drawable

from objloader import Obj
import runner.resources
import sys
from functools import partial
from runner.render2.run import ModernGLRender
from runner.render2.create_context import create_context
from runner.render2.run_example import run_example

# sets up the physics system and graphics/runner environments

resources_base_dir = 'runner/resources/' # must contain ending slash


if gui != 'pybullet':
    pybullet.connect(pybullet.DIRECT)
else:
    pybullet.connect(pybullet.GUI)


pybullet.setGravity(0, 0, -9.81)


if gui == 'robolab':
    pygame.init()
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)
    init_draw(*display)


sc_id = sys.argv[1] if len(sys.argv) > 1 else scene_id
scr_id = sys.argv[2] if len(sys.argv) > 2 else scene_script_id

if gui == 'robolab':
    upload_base_textures(resources_base_dir)

    def add_drawable_for_part(resources_base_dir, filename):
            drawable = load_object(resources_base_dir, filename)
            upload_textures_for_drawable(resources_base_dir, drawable)
            return drawable

    color = [0]
    postproc_robot = partial(render_postproc_robot, partial(add_drawable_for_part, resources_base_dir), color)

if gui == 'pybullet':
    postproc_robot = lambda x,y: None

if gui == 'robolab2':

    window, prog = create_context(display)

    def add_drawable(model):
        obj = Obj.open(runner.resources.find(model))
        vbo = window.ctx.buffer(obj.pack('vx vy vz nx ny nz'))
        vao = window.ctx.simple_vertex_array(prog, vbo, 'in_vert', 'in_norm')
        return {'vao': vao}

    postproc_robot = partial(moderngl_render_postproc_robot, add_drawable)


scene, (init_scene, apply_changes), props = setup_scene(sc_id, scr_id, resources_base_dir, postproc_robot)

print(scene)

if gui != 'robolab2':
    loop(pygame.time.Clock(), scene, init_scene, apply_changes, props, gui == 'pybullet')

if gui == 'robolab2':
    run_example(ModernGLRender, ctx=window.ctx, wnd=window, prog=prog, scn=scene)


pybullet.disconnect()