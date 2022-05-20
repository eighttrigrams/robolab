from immutable.immutable import ImmutableRecord
from runner.utils import XYZ

props = ImmutableRecord({
    'player_position': XYZ({'x': 1.5, 'y': -6., 'z': 4.}),
    'gaze_normal': XYZ({'x': -1., 'y': 1., 'z': -1.}),
})


scene=[

    {
        "robot": "box.obj",
        "position": (-1.2, 0, 1)
    },
    {
        "robot": "box.obj",
        "position": (1.2, 0, 1)
    }
]