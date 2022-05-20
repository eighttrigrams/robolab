# Robolab | Robolab

## Notes

As of May 2020, the installation only works for MacOS. pygame could not by installed via pip, tensorflow 1.5 is no longer available, though I did not investigate anything further after the pygame error.

## Prerequisites

Link the following repos with symlinks into your local environment

```
cd robolab/robolab
ln -s ../tools tools
ln -s ../immutable immutable
```

* OpenGL2
* Python3

If you prefer to install python libs locally, 
use `virtualenv env; . env.sh` to set up a virtualenv. 

* `pip3 install PyOpenGL PyOpenGL_accelerate pygame pybullet Pillow numpy tensorflow`
* `pip3 install tensorflow==1.5` (Ubuntu)

## Start

`python3 -m runner.start`

This load the scene configured in [config.py](runner/config.py). To load another scene
and scenario one can run

`python3 -m runner.start {scene_id} [{scene_script_id}]`

where the scene_id and scene_script_id refer to files in runner/scenes. A valid
call would be

`python3 -m runner.start 4 4_1`

the first part of scene_script_id must match scene_id.

## Development 

For working with virtualenv, there is `. env.sh`, which 
sets up the current terminal for working with the env at ./env (created with `virtualenv env`)
This also provides shortcuts to work with the project

For `python3 -m runner.start 2 1` one can now write `g 2 1`

### Testing

To execute all unit tests of the project, run

`python3 -m  unittest discover -s ./ -p "*_test.py"`

Single tests can get executed with

* python3 -m unittest src/render/test/*.py
* python3 -m unittest src/*/**/*.py
* python3 -m  unittest immutable.immutable_test.TestImmutable.test_assoc
* python3 -m  unittest immutable.immutable_test
* python3 -m unittest discover -s runner/test/render -p "*_test.py"
* python3 -m unittest discover -s ./ -p "*_test.py"
* python3 -m unittest discover -s . -p "*_test.py"
* test cases must start with test_
* -p "*_test.py" apparently pattern cannot be *.test.py
