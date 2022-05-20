from immutable.immutable import ImmutableRecord
from runner.utils import XYZ

# rendering

gui = 'robolab' # 'robolab'|'robolab2'|'pybullet'
display = (1280, 720)

# scenes

scene_id = '2'
scene_script_id = '1'


# default  values for various scene properties. can be overriden as needed

PLAYER_POSITION = 'player_position'
GAZE_NORMAL = 'gaze_normal'
PLAYER_ROTATION = 'player_rotation'
LIGHT_POSITION = 'light_position'

X = 'x'
Y = 'y'
Z = 'z'

props = ImmutableRecord({
    PLAYER_POSITION: XYZ({X: 0., Y: -2., Z: 1.}),
    GAZE_NORMAL: XYZ({X: 0., Y: 1., Z: 0.}),
    PLAYER_ROTATION: ImmutableRecord({Z: 0., X: 0.}),
    LIGHT_POSITION: XYZ({X: 0., Y: 2.5, Z: 2.})
})
