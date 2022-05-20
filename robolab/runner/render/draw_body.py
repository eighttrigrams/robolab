from OpenGL.GL import *


def draw(drawable, shaders, textures):

    for group in drawable['groups']:
        if 'texture_name' in group:
            _draw_textured(group, textures[group['texture_name']])
        elif 'color' in group:
            glUniform1f(shaders['basic']['params']['textures_disabled'], True)
            _draw_untextured(group)
            glUniform1f(shaders['basic']['params']['textures_disabled'], False)


def _draw_textured(group, textureID):

    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_TEXTURE_COORD_ARRAY)
    glEnableClientState(GL_NORMAL_ARRAY)

    glEnable(GL_TEXTURE_2D)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL)
    glBindTexture(GL_TEXTURE_2D, textureID)

    glBegin(GL_TRIANGLES)
    for face in group['triangles']:
        _draw_vertex_and_texture_and_normal(face)
    glEnd()

    glBegin(GL_QUADS)
    for face in group['quads']:
        _draw_vertex_and_texture_and_normal(face, 4)
    glEnd()

    glDisableClientState(GL_VERTEX_ARRAY)
    glDisableClientState(GL_TEXTURE_COORD_ARRAY)
    glDisableClientState(GL_NORMAL_ARRAY)
    glDisable(GL_TEXTURE_2D)


def _draw_untextured(group):

    glColor3f(group['color'][0], group['color'][1], group['color'][2])

    glEnableClientState(GL_VERTEX_ARRAY)
    glEnableClientState(GL_NORMAL_ARRAY)

    glBegin(GL_TRIANGLES)
    for face in group['triangles']:
        _draw_vertex_and_normal(face)
    glEnd()

    glBegin(GL_QUADS)
    for face in group['quads']:
        _draw_vertex_and_normal(face, 4)
    glEnd()

    glDisableClientState(GL_VERTEX_ARRAY)
    glDisableClientState(GL_NORMAL_ARRAY)


def _draw_vertex_and_normal(face, n = 3):
    for i in range(0, n):
        glNormal3fv(face[i][1])
        glVertex3fv(face[i][0])


def _draw_vertex_and_texture_and_normal(face, n = 3):
    for i in range(0, n):
        glTexCoord2fv(face[i][2])
        glNormal3fv(face[i][1])
        glVertex3fv(face[i][0])
