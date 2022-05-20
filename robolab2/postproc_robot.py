

def postproc_robot(insert_drawable, robot, insertions = None):
    """
    Abc
    :param insert_drawable
    :return:
    """
    robot['part']['drawable'] = insert_drawable(robot['part']['model'])
    del robot['part']['model']




