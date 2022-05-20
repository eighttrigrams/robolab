from immutable.immutable import ImmutableRecord
from runner.utils import XYZ

props = ImmutableRecord({
    'player_position': XYZ({'x': 0., 'y': -10., 'z': 1.}),
    'gaze_normal': XYZ({'x': 0., 'y': 1., 'z': 0.}),
})


scene=[
    {
        "robot": "box.urdf",
        "position": (-4, 0, 1)
    },
    {
        "robot": "box.urdf",
        "position": (-4, 5, 1),
        "part": {
            "color": (0.0, 1.0, 1.0, 1.0),
            "display_as": "colored",
        }
    },
    {
        "robot": "box.urdf",
        "position": (6, 0, 1),
        "part": {
            "display_as": "marble",
        }
    },
    {
        "robot": "box.urdf",
        "position": (5, 5, 1)
    },


    {
        "robot": "ground.urdf",
        "position": (5, -5, -0.7),
        "part": {
            "display_as": "ground",
        }
    },
    {
        "robot": "ground.urdf",
        "position": (5, 5, -0.7),
        "part": {
            "display_as": "ground",
        }
    },
    {
        "robot": "ground.urdf",
        "position": (-5, 5, -0.7),
        "part": {
            "display_as": "ground",
        }
    },

    {
        "robot": "ground.urdf",
        "position": (-5, -5, -0.7),
        "part": {
            "display_as": "ground",
        }
    },
    {
        "identifier": "hunchback",
        "rotation": (0,0,180+15),
        "robot": "hunchback.urdf",
        "part": {
            "display_as": "harlekin",
            "connects_to": [{"part": {"display_as": "striped"}}, {"part": {"display_as": "striped"}}]
        }
    }
]