from moderngl_render.window.window import Window

vs = '''
    #version 330
    
    uniform mat4 Mvp;

    in vec3 in_vert;
    in vec3 in_norm;

    out vec3 v_vert;
    out vec3 v_norm;


    void main() {
        gl_Position = Mvp * vec4(in_vert, 1.0);
        v_vert = in_vert;
        v_norm = in_norm;
    }
    '''

fs = '''
    #version 330

    uniform vec3 Light;
    uniform sampler2D Texture;

    in vec3 v_vert;
    in vec3 v_norm;

    out vec4 f_color;

    void main() {
        float lum = clamp(dot(normalize(Light - v_vert), normalize(v_norm)), 0.0, 1.0) * 0.8 + 0.2;
        f_color = vec4(vec3(1.0,0.0,1.0) * lum, 1.0);
        //f_color = vec4(1.0,0.0,0.0,1.0);
    }
    '''

# todo

def create_context(display):
    window = Window(size=display)
    prog = window.ctx.program(vertex_shader=vs, fragment_shader=fs)
    return window, prog