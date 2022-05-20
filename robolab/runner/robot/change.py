from .iterator import iterate_robot
from runner.utils import nop

def set_robot_pos_orn(robot, get_pos_orn_robot, get_pos_orn_joint):

    robot['position'], robot['rotation'] = get_pos_orn_robot(robot['id'])

    if 'connects_to' not in robot['part']: return

    def update_joint(joint):
        joint['position'], joint['rotation'] = get_pos_orn_joint(robot['id'], joint)

    for connection in robot['part']['connects_to']:
        iterate_robot(connection, nop, update_joint)
