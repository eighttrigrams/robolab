from immutable.immutable import ImmutableRecord
from runner.utils import XYZ

props = ImmutableRecord({
    'player_position': XYZ({'x': 1.5, 'y': -6., 'z': 4.}),
    'gaze_normal': XYZ({'x': -1., 'y': 1., 'z': -1.}),
})


scene=[

    {
        "robot": "rail.urdf",
        "position": (-1.2, 0, 1)
    },
    {
        "robot": "rail.urdf",
        "position": (1.2, 0, 1)
    },


    {
        "robot": "ground.urdf",
        "position": (5, -5, 0)
    },
    {
        "robot": "ground.urdf",
        "position": (5, 5, 0)
    },
    {
        "robot": "ground.urdf",
        "position": (-5, 5, 0)
    },

    {
        "identifier": "ground",
        "robot": "ground.urdf",
        "position": (-5, -5, 0)
    },
    {
        "identifier": "hunchback",
        "position": (0., 0., 2),
        "rotation": (0,0,180),
        "robot": "railbot.urdf",
        "part": {
            "display_as": "harlekin",
            "connects_to": [{"part": {"display_as": "striped"}}, {"part": {"display_as": "striped"}}, {"part": {"display_as": "striped"}}, {"part": {"display_as": "striped"}}]
        }
    }
]