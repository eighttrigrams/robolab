from OpenGL.GL import *
from OpenGL.GL import shaders

from immutable.immutable import ImmutableRecord


def get_builder():
    def _basic():

        vertex_shader = shaders.compileShader('''
        
            varying float fog; 
            float endFog = 18.5;
            uniform vec4 fogColor;
            uniform bool texturesDisabled;
            vec4 Global_ambient;
            vec4 Light_ambient;
            vec4 Light_diffuse;
            uniform float lightX;
            vec3 Light_location;
            vec4 Material_ambient;
            vec4 Material_diffuse;
            
            varying vec4 baseColor;
    
            void main(){
            
                gl_Position = ftransform(); 
                gl_TexCoord[0] = gl_MultiTexCoord0; 
                Global_ambient = vec4(0.3, 0.3, 0.3 , 0.1 );
                Light_ambient = vec4(0.2,0.2,0.2, 1.0 );
                Light_diffuse = vec4(1.0,1.0,1.0,1.0 );
                Material_ambient = vec4(0.2, 0.2, 0.2, 0.3 );
                Material_diffuse = vec4(0.7,0.7, 0.7, 0.2 );
                
                vec3 normal = gl_NormalMatrix * gl_Normal; 
                vec3 normalized_normal = normalize(normal);
                
                // vec3 EC_Light_location = gl_NormalMatrix * gl_LightSource[0].position.xyz;              // not working
                // float diffuse_weight = max(0.0, dot(normalized_normal, normalize(EC_Light_location)));  // properly
                
                // 2 working versions, the latter is something i tried based on the other versions, which seems to work better
                // float diffuse_weight = dot(vec3(gl_LightSource[0].position), normalized_normal);                         
                float diffuse_weight = dot(normalize(gl_LightSource[0].position.xyz), normalized_normal);                  
                                  
                baseColor = clamp(
                    (
                    (Global_ambient * Material_ambient)
                    +(Light_ambient * Material_ambient)
                    +(Light_diffuse * Material_diffuse * diffuse_weight)
                    ), 0.0, 1.0);    
                    
                /*
                if (diffuse_weight > 0.95)
                    baseColor = vec4(0.5,1.0,0.5,1.0);
                else if (diffuse_weight > 0.5)
                    baseColor = vec4(0.3,0.6,0.3,1.0);
                else if (diffuse_weight > 0.25)
                    baseColor = vec4(0.2,0.4,0.2,1.0);
                else
                    baseColor = vec4(0.1,0.2,0.1,1.0);   
                */
                
                
                float fogCoord; 
                fogCoord = abs(gl_Position.z);
                fogCoord = clamp( fogCoord, 0.0, endFog);
                fog = (endFog - fogCoord)/endFog;
                fog = clamp( fog, 0.0, 1.0);
                
                if (texturesDisabled) {
                    baseColor = mix(fogColor, mix(gl_Color, baseColor, fog), fog);
                }
                // else baseColor = mix(fogColor, baseColor, fog);
                
            }
            ''', GL_VERTEX_SHADER)


        fragment_shader = shaders.compileShader('''
        
            uniform vec4 fogColor;
            varying float fog; 
            varying vec4 baseColor;
            uniform sampler2D sampler;
            uniform bool texturesDisabled;
            
            void main() {
                vec4 cool = texture2D( sampler, gl_TexCoord[0].st );
                if (texturesDisabled) gl_FragColor = baseColor;
                else {
                    gl_FragColor = mix(fogColor, mix(baseColor, cool, fog), fog);
                    // gl_FragColor = mix(baseColor, cool, 1.0);
                }  
            }
            ''', GL_FRAGMENT_SHADER)

        return (vertex_shader, fragment_shader)


    def _compile(which):

        shader = -1
        try:
            shader_pair = which()
            shader = shaders.compileProgram(shader_pair[0], shader_pair[1])
        except Exception as err:
            info = glGetShaderInfoLog(shader)
            print('(ERROR) ' + str(err))
            print('(ERROR) ' + str(info))
            import sys
            sys.exit()
        return shader


    def make(fog_color):

        b = _compile(_basic)
        glUseProgram(b)

        fc_loc = glGetUniformLocation(b, 'fogColor')
        glUniform4f( fc_loc, *fog_color)

        td_loc = glGetUniformLocation( b, 'texturesDisabled' )
        glUniform1f(td_loc, False)

        return {
            'basic': {
                'program': b,
                'params': {
                    'textures_disabled': td_loc
                }
            }
        }

    return ImmutableRecord({'make': make})