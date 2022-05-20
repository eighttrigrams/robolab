from OpenGL.GL import *
from PIL import Image
import ntpath

textures = {}


def upload_textures_for_drawable(resources_base_dir, drawable):
    for tex in drawable['texture_names']:
        upload_texture(resources_base_dir + tex)


def upload_texture(path_to_file):
    im = Image.open(path_to_file)
    c = im.convert('RGB')

    data = c.getdata()
    flattened = [item for sublist in data for item in sublist]

    ix, iy = im.size

    textureID = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, textureID)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy,
                 0, GL_RGB, GL_UNSIGNED_BYTE, flattened)

    im.close()
    c.close()

    global textures
    textures[ntpath.basename(path_to_file)] = textureID
