from immutable.immutable import ImmutableRecord


def _read_obj_file(path, filename):
    triangles = []
    quads = []
    vertices = []
    vertex_normals = []
    vertex_textures = []

    texture_names = []

    try:
        f = open(path + filename)

        for line in f:
            if line.startswith('mtllib'):
                mtllib_name = line.split()[1]
                try:
                    m = open(path + mtllib_name)
                    for li in m:
                        if li.startswith('map_Kd'):
                            tex = li.split()[1]
                            texture_names.append(tex)
                    m.close()
                except:
                    print('(WARN) material lib not found in resources dir', path + mtllib_name)


            if line[:2] == 'vn':
                items = line.split()
                items.remove('vn')
                vertex_normal = []
                for item in items:
                    vertex_normal.append(round(float(item), 2))
                vertex_normals.append(tuple(vertex_normal))

            if line[:2] == 'vt':
                items = line.split()
                items.remove('vt')
                vertex_texture = []
                for item in items:
                    vertex_texture.append(round(float(item), 2))
                vertex_textures.append(tuple(vertex_texture))

            if line[:2] == 'v ':

                items = line.split()
                items.remove('v')
                vertex = []
                for item in items:
                    vertex.append(round(float(item), 2))

                vertices.append(tuple(vertex))

            elif line[0] == 'f':

                face_element = line.split()
                face_element.remove('f')
                face = []

                i = 0
                for face_element in face_element:
                    subelement = face_element.split('/')

                    if not len(subelement) == 3: raise Exception('no normals found in obj file ' + filename)
                    face.append(ImmutableRecord({
                        'v': int(subelement[0]) - 1,
                        't': int(subelement[1]) - 1 if subelement[1] != '' else -1,
                        'n': int(subelement[2]) - 1 if len(subelement) == 3 else -1
                    }))
                    i = i + 1

                if i == 3:
                    triangles.append(tuple(face))
                elif i == 4:
                    quads.append(tuple(face))
                else:
                    print('(WARN) only tris and quads allowed')

        f.close()
        return {
            'triangles': triangles,
            'quads': quads,
            'vertices': vertices,
            'vertex_normals': vertex_normals,
            'vertex_textures': vertex_textures,
            'texture_names': texture_names
        }
    except IOError:
        raise IOError("could not load object file " + path + filename)


def load_object(path, filename):
    result = _read_obj_file(path, filename)

    triangles_resolved = []
    quads_resolved = []

    for quad in result['quads']:
        quad_resolved = []
        for vertex in quad:
            quad_resolved.append((
                result['vertices'][vertex.v],
                result['vertex_normals'][vertex.n],
                result['vertex_textures'][vertex.t] if vertex.t != -1 else -1
            ))
        quads_resolved.append(quad_resolved)


    for triangle in result['triangles']:
        quad_resolved = []
        for vertex in triangle:
            quad_resolved.append((
                result['vertices'][vertex.v],
                result['vertex_normals'][vertex.n],
                result['vertex_textures'][vertex.t] if vertex.t != -1 else -1
            ))
        triangles_resolved.append(quad_resolved)

    result['quads'] = quads_resolved
    result['triangles'] = triangles_resolved
    return result



