URDF = '''<?xml version="1.0"?>
<robot name="generated">
  <link name="base_link">
      <collision>
        <geometry>
          <mesh filename="FILENAME" scale="1 1 1"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0"/>
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
    </link>
</robot>
'''


def generate_urdf(resources_base_dir, obj_filename, urdf = URDF):
    f = open(resources_base_dir + '.urdf.swp', 'w')
    f.write(urdf.replace('FILENAME', obj_filename))
    f.close()