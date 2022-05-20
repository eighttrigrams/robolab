from immutable.immutable import ImmutableRecord
from runner.config import PLAYER_ROTATION, GAZE_NORMAL, PLAYER_POSITION, Z, X, Y
from runner.utils import XYZ


def leg():
    return { "part": {
        "display_as": "harlekin",
    }}


props = ImmutableRecord({
    PLAYER_POSITION: XYZ({X: 4.0, Y: -7.0, Z: 3.8}),
    GAZE_NORMAL: XYZ({X: -1.0, Y: 2., Z: 0.}),
    PLAYER_ROTATION: ImmutableRecord({Z: 0., X: 0.2})
})


scene=[{
    "robot": "box.urdf",
    "position": (-0.50, 0, .6)
},{
    "robot": "ground.urdf",
    "position": (0, 0, 0),
    "part": {
    }
},{
    "robot": "box.urdf",
    "position": (0.6, 0, .6)
},{
    "robot": "box.urdf",
    "position": (-0.6, 0, 1.6)
},{
    "identifier": "gripper",
    "robot": "gripper.urdf",
    "position": (-1.4, 0.1, 3.65),
    "rotation": (0, 0, 90),
    "part": {
        "connects_to": [leg(), leg(), leg(), leg(), {"part": {"display_as": "striped"}}]
    }
}]