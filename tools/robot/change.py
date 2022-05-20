def set_position(robot, get_pos_orn_robot, get_pos_orn_joint):
    robot['position'], robot['rotation'] = get_pos_orn_robot(robot['id'])
    _set_position_part(robot['id'], robot['part'], get_pos_orn_joint)


def _set_position_part(robot_id, part, get_pos_orn):
    if 'connects_to' in part:
        for connection in part['connects_to']:
            _set_position_connection(robot_id,connection, get_pos_orn)


def _set_position_connection(robot_id, connection, get_pos_orn):
    connection['position'], connection['rotation'] = get_pos_orn(robot_id, connection)
    _set_position_part(robot_id, connection['part'], get_pos_orn)