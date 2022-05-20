import pygame
import pybullet
from pygame import *
from immutable.immutable import assoc_in, update_in, transact
from runner.config import GAZE_NORMAL, PLAYER_POSITION, X, Y, PLAYER_ROTATION, Z, LIGHT_POSITION
from runner.utils import identity, plus, minus
from runner.render.draw import *
from runner.scene_updater import update_pos_orns


def _player_position_updates(props, keys):
    fx = identity
    fy = identity

    gaze_normal = props.gaze_normal
    if keys[K_w]:
        fx = plus(0.2 * gaze_normal.x)
        fy = plus(0.2 * gaze_normal.y)
    if keys[K_s]:
        fx = minus(0.2 * gaze_normal.x)
        fy = minus(0.2 * gaze_normal.y)
    if keys[K_a]:
        fx = plus(0.2 * -gaze_normal.y)
        fy = plus(0.2 * gaze_normal.x)
    if keys[K_d]:
        fx = minus(0.2 * -gaze_normal.y)
        fy = minus(0.2 * gaze_normal.x)

    return [
        (update_in, [PLAYER_POSITION, X], fx),
        (update_in, [PLAYER_POSITION, Y], fy),
        (update_in, [PLAYER_POSITION, Z], (plus(0.1) if keys[K_r] else minus(0.1))
            if keys[K_r] or keys[K_f] else identity)]


def handle_keys(props):
    keys=pygame.key.get_pressed()

    actions = []

    if keys[K_LEFT] or keys[K_RIGHT]:
        actions.append((update_in, [PLAYER_ROTATION, Z], minus(0.1) if keys[K_LEFT] else plus(0.1)))
        actions.append((assoc_in, [GAZE_NORMAL, X], math.sin(props.player_rotation.z)))
        actions.append((assoc_in, [GAZE_NORMAL, Y], math.cos(props.player_rotation.z)))

    if keys[K_UP] or keys[K_DOWN]:
        actions.append((update_in, [PLAYER_ROTATION, X], minus(0.03) if keys[K_UP] else plus(0.03)))

    if keys[K_j]:
        actions.append((assoc_in, [LIGHT_POSITION, X], props.light_position.x - .5))
    if keys[K_l]:
        actions.append((assoc_in, [LIGHT_POSITION, X], props.light_position.x + .5))
    if keys[K_i]:
        actions.append((assoc_in, [LIGHT_POSITION, Y], props.light_position.y + .5))
    if keys[K_k]:
        actions.append((assoc_in, [LIGHT_POSITION, Y], props.light_position.y - .5))
    if keys[K_z]:
        actions.append((assoc_in, [LIGHT_POSITION, Z], props.light_position.z + .5))
    if keys[K_h]:
        actions.append((assoc_in, [LIGHT_POSITION, Z], props.light_position.z - .5))

    return transact(props, *(actions + _player_position_updates(props, keys)))


def handle_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            print("Quit")
            pygame.quit()
            quit()





def loop(clock, scene, init_scene, apply_changes, props, rendering_disabled = False):

    sess, state = init_scene(scene)

    i=0
    while True:
        if not rendering_disabled:
            handle_quit()
            props = handle_keys(props)

            update_pos_orns(scene)

            draw_scene(scene, props)
            pygame.display.flip()

        state = apply_changes(scene, i, sess, state)
        clock.tick(1240)

        pybullet.stepSimulation()
        pybullet.stepSimulation()
        pybullet.stepSimulation()
        pybullet.stepSimulation()
        pybullet.stepSimulation()
        pybullet.stepSimulation()
        i = i + 1
