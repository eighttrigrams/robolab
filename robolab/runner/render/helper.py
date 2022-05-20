import math
from OpenGL.GL import *
from OpenGL.GLU import *



def transform(pos, rot, sc = None):
    translate(pos)
    rotate(rot)
    if sc != None: scale(sc)


def translate(triple):
    glTranslatef(triple[0], triple[1], triple[2])


def rotate(triple_rad):
    glRotatef(math.degrees(triple_rad[2]), 0.0, 0.0, 1.0)
    glRotatef(math.degrees(triple_rad[1]), 0.0, 1.0, 0.0)
    glRotatef(math.degrees(triple_rad[0]), 1.0, 0.0, 0.0)


def scale(scaling):
    glScale(scaling[0], scaling[1], scaling[2])


def look_at(t0, t1, t2):
    gluLookAt(t0[0], t0[1], t0[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2])
