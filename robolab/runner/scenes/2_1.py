from runner.render.draw import *
from runner.utils import ImmutableRecord, XYZ, findByIdentifier
import pybullet

c = 0.02

dct = {
    "beg": 20,
    "en": 40,
    "jointForce": 0.,
    "jointForce2": 0.,
}

PC = pybullet.POSITION_CONTROL

def init_scene(scene):
    return 1, 1

def apply_changes(scene, i, sess, state):

    boxId = (findByIdentifier('hunchback')(scene))['id']
    control = partial(pybullet.setJointMotorControl2, boxId)

    if i > dct['beg']:
        dct['jointForce2'] = dct['jointForce2'] - c

        control(1, PC, dct['jointForce2'], force = 15)

        control(3, PC, dct['jointForce2'], force = 100)

    if i > dct['en']:
        dct['jointForce'] = dct['jointForce'] - c
        control(0, PC, dct['jointForce'], force = 5)
        control(2, PC, -dct['jointForce'], force = 500)

    return 1
