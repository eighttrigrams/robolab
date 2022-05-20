def iterate_robot(robot, on_limb, on_joint, *args):
    _iterate_joint(robot, on_limb, on_joint, *args)


def _iterate_joint(joint, on_limb, on_joint, *args):
    on_joint(joint, *args)
    if 'part' in joint:
        limb = joint['part']
        _iterate_limb(limb, on_limb, on_joint, *args)


def _iterate_limb(limb, on_limb, on_joint, *args):
    on_limb(limb, *args)
    if 'connects_to' in limb:
        joints = limb['connects_to']
        for joint in joints:
            _iterate_joint(joint, on_limb, on_joint, *args)