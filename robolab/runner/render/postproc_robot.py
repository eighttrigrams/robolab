from runner.render.texture_repository import upload_texture
from runner.robot.iterator import iterate_robot


def upload_base_textures(resources_base_dir):

    upload_texture(resources_base_dir + "marble.jpg")
    # upload_texture(resources_base_dir + "200_200_st.png")     # for stone
    upload_texture(resources_base_dir + "ground.png")


def postproc_robot(add_drawable, color, robot):

    iterate_robot(
        robot,
        _calculate_drawable,
        lambda *args: None,
        add_drawable,
        color)


def _calculate_drawable(part, add_drawable, color):

    drawable = -1
    if 'model' in part:
        model = part['model']
        if model.endswith('.obj'):
            drawable = add_drawable(model)
        elif model == 'cylinder':
            drawable = add_drawable('cylinder.obj')
        else:
            drawable = add_drawable('box.obj')

    if drawable == -1: raise Exception('(ERROR) no drawable found for part')


    triangles = drawable['triangles']
    quads = drawable['quads']

    if 'display_as' in part:

        if part['display_as'] not in ['harlekin', 'striped']:
            part['drawable'] = { 'groups' : [{'triangles': triangles, 'quads': quads}] }
        if  part['display_as'] in ['ground', 'marble']:
            for triangle in triangles:
                triangle[0] = (triangle[0][0], triangle[0][1], (4, 2))
                triangle[1] = (triangle[1][0], triangle[1][1], (0, 4))
                triangle[2] = (triangle[2][0], triangle[2][1], (4, 0))

        if part['display_as'] == 'ground':

            part['drawable']['groups'][0]['texture_name'] = 'ground.png'

        elif part['display_as'] == 'marble':

            part['drawable']['groups'][0]['texture_name'] = 'marble.jpg'

        elif part['display_as'] == 'harlekin':

            groups = [{'triangles': [], 'quads': [], 'color': (0.8, 0.8, 0.8, 1.0) },
                      {'triangles': [], 'quads': [], 'color': (0.7, 0.7, 0.7, 1.0)},
                      {'triangles': [], 'quads': [], 'color': (0.9, 0.9, 0.9)},
                      {'triangles': [], 'quads': [], 'color': (0.6, 0.6, 0.6)}]

            part['drawable'] = { 'groups' : groups }

            i = 0
            for triangle in triangles:
                groups[i]['triangles'].append(triangle)
                i = (i + 1) % 4

            i = 0
            for quad in quads:
                groups[i]['quads'].append(quad)
                i = (i + 1) % 4

        elif part['display_as'] == 'striped':

            groups = [{'triangles': [], 'quads': [], 'color': (1.0, 1.0, 1.0, 1.0) },
                      {'triangles': [], 'quads': [], 'color': (0.5, 0.5, 0.5, 1.0)}]

            part['drawable'] = { 'groups' : groups }

            i = 0
            for triangle in triangles:
                groups[i]['triangles'].append(triangle)
                i = (i + 1) % 2

            i = 0
            for quad in quads:
                groups[i]['quads'].append(quad)
                i = (i + 1) % 2

        elif part['display_as'] == 'colored':

            if 'color' in part:
                part['drawable']['groups'][0]['color'] = part['color']
            elif 'color' in part['drawable']['groups'][0]:
                part['drawable']['groups'][0]['color'] = part['color']
                del part['drawable']['color']
            else:
                raise Exception('(ERROR) requested display_as colored, but no color was found')

    else:
        part['drawable'] = { 'groups' : [{'triangles': triangles, 'quads': quads}] }

        if 'color' in part:
            part['drawable']['groups'][0]['color'] = part['color']
            del part['color']
        elif len(drawable['texture_names']) > 0:
            part['drawable']['groups'][0]['texture_name'] = drawable['texture_names'][0]
        else:
            if color[0] == 0: part['drawable']['groups'][0]['color'] = (1.0, 0.0, 0.0, 1.0)
            if color[0] == 1: part['drawable']['groups'][0]['color'] = (1.0, 1.0, 0.0, 1.0)
            if color[0] == 2: part['drawable']['groups'][0]['color'] = (0.0, 1.0, 0.0, 1.0)
            if color[0] == 3: part['drawable']['groups'][0]['color'] = (0.0, 0.0, 1.0, 1.0)
            color[0] = (color[0] + 1) % 4