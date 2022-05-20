from importlib import import_module
from immutable.immutable import assoc
from runner.loop import *
from runner.robot.helper import generate_urdf
from runner.robot.loader import read_urdf
from runner.robot.zipper import zip_robot
from .config import props


ID = 'id'

SCENES_MODULE = 'runner.scenes.'


def _set_base_position_and_orientation(robot):
    if not "rotation" in robot:
        robot['rotation'] = (0., 0., 0.)
    else:
        robot['rotation'] = (
            math.radians(float(robot['rotation'][0])),
            math.radians(float(robot['rotation'][1])),
            math.radians(float(robot['rotation'][2])))
    if not 'position' in robot:
        robot['position'] = (0., 0., 0.)
    else:
        robot['position'] = (float(robot['position'][0]), float(robot['position'][1]), float(robot['position'][2]))


def _import_scene(scene_id):
    scene_module_dict = import_module(SCENES_MODULE + scene_id).__dict__
    props_from_config = props
    if 'props' in scene_module_dict:
        props_ = scene_module_dict['props']
        for k, v in props_from_config.items(): # todo find a less verbose way to do this
            if not k in props_: props_ = assoc(props_, k, v)
    else:
        props_ = props_from_config
    return props_, scene_module_dict['scene']


def _import_scene_script(scene_id, scene_script_id):

    prefix = '0_'
    if scene_id != '0': prefix = scene_id + '_'
    scene_script_module_name = SCENES_MODULE + prefix + scene_script_id
    try:
        scene_script_module = import_module(scene_script_module_name)
        scene_script_module_dict = scene_script_module.__dict__
    except Exception:
        scene_script_module_name = SCENES_MODULE + '0_0'
        scene_script_module = import_module(scene_script_module_name)
        scene_script_module_dict = scene_script_module.__dict__

    return scene_script_module_dict['init_scene'], scene_script_module_dict['apply_changes']


def _load_urdf_into_physics_and_postprocess_for_graphics(insertions, resources_base_dir, filename, postproc_robot):
    _set_base_position_and_orientation(insertions)
    id = pybullet.loadURDF(resources_base_dir + filename, list(insertions['position']),
                           pybullet.getQuaternionFromEuler(list(insertions['rotation'])))
    robot = read_urdf(resources_base_dir + filename)

    zip_robot(robot, insertions)
    postproc_robot(robot)

    robot[ID] = id
    if 'identifier' in insertions: robot['identifier'] = insertions['identifier']
    return robot


def setup_scene(scene_id, scene_script_id, resources_base_dir, postproc_robot):

    props, scene = _import_scene(scene_id)

    scene_ = []
    for entity in scene:
        robot_filename = entity['robot']
        if robot_filename.endswith('.urdf'):
            robot = _load_urdf_into_physics_and_postprocess_for_graphics(entity, resources_base_dir, robot_filename, postproc_robot)
            scene_.append(robot)
        if robot_filename.endswith('.obj'):
            generate_urdf(resources_base_dir, robot_filename)
            robot = _load_urdf_into_physics_and_postprocess_for_graphics(entity, resources_base_dir, '.urdf.swp', postproc_robot)
            scene_.append(robot)

    return scene_, _import_scene_script(scene_id, scene_script_id), props