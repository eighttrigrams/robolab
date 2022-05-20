import numpy as np

from runner.utils import lambdaize
from .draw_body import draw
from .shaders import *
from .texture_repository import textures, upload_texture
from .helper import *
from functools import partial


ID = 'id'


shaders = {}

fog_color = (0.7, 0.7, 0.7, 0.0)
# 1.0 was used for 1 in clear color before and 0 in shader. now both have same value 0. seems to make no difference though



def init_draw(x, y):
    glViewport(0, 0, x, y)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (x/y), 0.1, 50.0)
    glMatrixMode(GL_MODELVIEW)
    global shaders
    shaders = get_builder().make(fog_color)
    glUseProgram(shaders['basic']['program'])
    glEnable(GL_DEPTH_TEST)
    glClearColor(*fog_color)


def draw_scene(scene, props):

    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

    glLoadIdentity()
    _set_light(props.light_position)
    _set_camera(props)
    light_pos = props.light_position.as_tuple()
    translate(light_pos)
    rotate((0.875, 0.875, 0.0))
    scale((0.2, 0.2, 0.2))
    # draw(ImmutableRecord(drawables['box']), shaders, textures) todo fix


    for entity in scene:

        glLoadIdentity()
        _set_light(props.light_position)
        _set_camera(props)

        glPushMatrix()
        _draw_connection(ImmutableRecord(entity), [], _draw)
        glPopMatrix()


def _draw_connection(connection, fs, draw):

    glPopMatrix()
    glPushMatrix()

    f = lambdaize(transform)(*(connection.position, connection.rotation))

    _draw_part(
        ImmutableRecord(connection.part),
        fs + [f],
        draw)


def _draw(drawable):
    draw(ImmutableRecord(drawable), shaders, textures),



def _draw_part(part, fs, draw):

    for f in fs: f()
    transform(part.position, part.rotation, part.scaling)

    if 'drawable' in part: draw(part.drawable)
    else: print("(WARN) drawable not found " + part['model'])

    if 'connects_to' in part:
        for connection in part.connects_to:
            _draw_connection(ImmutableRecord(connection), fs, draw)


def _set_light(light_position):
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, light_position.as_tuple())


def _set_camera(props):
    player_position = props.player_position.as_tuple()
    looking_direction = props.gaze_normal.as_tuple()
    looking_at = tuple(np.array(player_position) + np.array(looking_direction))

    rotate((props.player_rotation.x, 0., 0.))
    look_at(player_position, looking_at, (0., 0., 1.))







