from runner.render.draw import *
from runner.utils import XYZ, findByIdentifier
import pybullet


dct = {
    "beg": 70,
    "mid": 100,
    "en": 120,
    "jointForce": 0.0
}

force = 25

PC = pybullet.POSITION_CONTROL

def init_scene(scene):
    return 1, 1

def apply_changes(scene, i, sess, state):
    beg = dct['beg']
    mid = dct['mid']
    en = dct['en']
    jointForce = dct['jointForce']


    if i > beg and i < mid:
        dct['jointForce'] = jointForce + 0.08
        jointForce = dct['jointForce']
    boxId = (findByIdentifier('gripper')(scene))['id']
    control = partial(pybullet.setJointMotorControl2, boxId)
                      
                      
    if (i > beg): control(0, PC, jointForce)
    if (i > beg): control(1, PC, jointForce)
    if (i > beg): control(2, PC, jointForce)
    if (i > beg): control(3, PC, jointForce)
    if (i > beg): control(4, PC, -jointForce)
    if (i > beg): control(5, PC, -jointForce)
    if (i > beg): control(6, PC, -jointForce)
    if (i > beg): control(7, PC, -jointForce)

    # if i > mid:
        # dct['jointForce'] = 0.0
        # jointForce = dct['jointForce']
        # pybullet.applyExternalForce(
        #     scene[3][ID], -1, [0, 22, 0], [0, 0, 0], pybullet.LINK_FRAME)

    if i > en:
        dct['jointForce'] = jointForce + 0.01
        jointForce = dct['jointForce']
    if (i > en): control(0, PC, -jointForce, force = force)
    if (i > en): control(1, PC, -jointForce, force = force)
    if (i > en): control(2, PC, -jointForce, force = force)
    if (i > en): control(3, PC, -jointForce, force = force)
    if (i > en): control(4, PC, jointForce, force = force)
    if (i > en): control(5, PC, jointForce, force = force)
    if (i > en): control(6, PC, jointForce, force = force)
    if (i > en): control(7, PC, jointForce, force = force)

    return 1