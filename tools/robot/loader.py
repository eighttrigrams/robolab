import xml.etree.ElementTree as ET


def read_urdf(path_to_urdf_file):

    item = {}

    tree = ET.parse(path_to_urdf_file)
    root = tree.getroot()

    materials = {}
    for material in root.findall('material'):
        colors = []
        for color in material.findall('color'):
            colors.append(color.attrib['rgba'])
        materials[material.attrib['name']] = colors[0]

    link_names = []
    for link in root.findall('link'):
        link_names.append(link.attrib['name'])

    children = []
    for joint in root.findall('joint'):
        child = joint.find('child').attrib['link']
        children.append(child)

    trunks = [i for i in children + link_names if i not in children or i not in link_names]
    if len(trunks) != 1: raise Exception("E")
    trunk = trunks[0]

    joints = []
    _calc_joint_numbers(joints, trunk, root, [0])

    if not 'part' in item: item['part'] = {}
    _create_part(item['part'], trunk, root, joints, materials)

    return item


def _calc_joint_numbers(joints, source_el_name, root, i):
    for joint in root.findall('joint'):
        if joint.find('parent').attrib['link'] == source_el_name:
            name_of_child = joint.find('child').attrib['link']

            xyz = joint.find('origin').attrib['xyz'].split()
            xyz = (float(xyz[0]), float(xyz[1]), float(xyz[2]))

            axis_xyz = (1.0, 0.0, 0.0)
            axes = joint.findall('axis')
            if len(axes) > 0:
                axis = axes[0]
                axis_xyz = axis.attrib['xyz'].split()
                axis_xyz = (float(axis_xyz[0]), float(axis_xyz[1]), float(axis_xyz[2]))

            joints.append((i[0], name_of_child, xyz, axis_xyz))
            i[0] = i[0] + 1
            _calc_joint_numbers(joints, name_of_child, root, i)



def _create_part(part, source_el_name, root, joints, materials):
    link = root.find('.//link[@name="' + source_el_name + '"]')

    material_refs = link.findall('.//material')
    if len(material_refs) > 0:
        color = materials[material_refs[0].attrib['name']]
        part['display_as'] = 'colored'
        part['color'] = tuple(map(float, color.split()))

    collision_geometries = link.findall('.//collision')
    visual_geometries = link.findall('.//visual')
    model_geometries = visual_geometries if len(visual_geometries) > 0 else collision_geometries
    if not len(model_geometries) > 0: raise Exception('no geometry found at all for part')
    model_geometry = model_geometries[0]

    origins = model_geometry.findall('.//origin')
    if len(origins) > 0:
        origin = origins[0]
        xyz = origin.attrib['xyz'].split()
        xyz = (float(xyz[0]), float(xyz[1]), float(xyz[2]))
        part['position'] = xyz

        if 'rpy' in origin.attrib:
            rpy = origin.attrib['rpy'].split()
            rpy = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
            part['rotation'] = rpy

    box = model_geometry.findall('.//box')
    if len(box) > 0:
        size = box[0].attrib['size'].split()
        size = (float(size[0]), float(size[1]), float(size[2]))
        part['scaling'] = size
        part['model'] = 'box'

    cylinder = model_geometry.findall('.//cylinder')
    if len(cylinder) > 0:
        length = cylinder[0].attrib['length']
        radius = cylinder[0].attrib['radius']
        scaling = (float(radius) * 2, float(radius) * 2, float(length))
        part['scaling'] = scaling
        part['model'] = 'cylinder'

    mesh = model_geometry.findall('.//mesh')
    if len(mesh) > 0:
        scale = mesh[0].attrib['scale'].split()
        scale = (float(scale[0]), float(scale[1]), float(scale[2]))
        part['scaling'] = scale
        filename = mesh[0].attrib['filename']
        part['model'] = filename

    if not 'scaling' in part:
        part['scaling'] = (1., 1., 1.)
    if not 'rotation' in part:
        part['rotation'] = (0., 0., 0.)
    if not 'position' in part:
        part['position'] = (0., 0., 0.)

    children = []
    for joint in root.findall('joint'):
        child = joint.find('parent')
        if child.attrib['link'] == source_el_name:
            children.append(joint.find('child').attrib['link'])

    if children and not 'connects_to' in part:
        part['connects_to'] = []

    i = 0
    for child in children:

        if (len(part['connects_to'])) <= i:
            part['connects_to'].append({})

        if not 'part' in part['connects_to'][i]:
            part['connects_to'][i]['part'] = {}

        for joint in joints:
            if joint[1] == child:
                if not 'id' in part['connects_to'][i]:
                    part['connects_to'][i]['id'] = joint[0]
                if not 'position' in part['connects_to'][i]:
                    part['connects_to'][i]['position'] = joint[2]
                if not 'axis' in part['connects_to'][i]:
                    part['connects_to'][i]['axis'] = joint[3]

        _create_part(part['connects_to'][i]['part'], child, root, joints, materials)
        i = i + 1