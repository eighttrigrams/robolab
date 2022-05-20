def zip_robot(robot1, robot2):
    _zip_connection(robot1, robot2)


def _zip_connection(connection1, connection2):

    if 'part' in connection1:
        connection2_part = connection2['part'] if 'part' in connection2 else {}
        _zip_part(connection1['part'], connection2_part)


def _zip_part(part1, part2):

    for key in part2.keys():
        if key == 'connects_to': continue
        part1[key] = part2[key]

    if 'connects_to' in part1:

        part2_connections = []
        if not 'connects_to' in part2:
            for _ in part1['connects_to']:
                part2_connections.append({'part': {}})
        else:
            part2_connections = part2['connects_to']
            if len(part2_connections) != len(part1['connects_to']):
                raise Exception("Part and insertion connections not equal")

        for connection in zip(part1['connects_to'], part2_connections):
            _zip_connection(connection[0], connection[1])



