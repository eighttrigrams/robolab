import pybullet
from tools.robot.change import set_position


def get_pos_orn_joint(entity_id, connection):
    orn = pybullet.getJointState(entity_id, connection['id'])[0]
    orn = [i * orn for i in connection['axis']]
    return connection['position'], orn


def get_pos_orn_robot(entity_id):
    (pos, orn) = pybullet.getBasePositionAndOrientation(entity_id)
    return pos, pybullet.getEulerFromQuaternion(orn)


def update_pos_orns(scene):
    for entity in scene:
        set_position(entity, get_pos_orn_robot, get_pos_orn_joint)