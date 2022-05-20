
# Window Examples

ModernGL can only create contexts in headless mode and relies on other libraries
to create a context displaying a visible window. ModernGL will then load OpenGL
functions from the existing contexts instead.

This package contains examples using ModernGL with various libraries
opening a visible window.

**Most example windows should work on Windows, Linux and OS X.
Some require additional setup such as installing dynamic libraries.**

## PyQt5

PyQt5 is a comprehensive set of Python bindings for Qt v5.
This package requires no external dependencies and works pretty much
out of the box on most platforms.

Homepage: https://www.riverbankcomputing.com/software/pyqt/intro

## pyglet

Pyglet is a cross-platform windowing and multimedia library for Python.
No external dependencies or installation requirements.
As is January 2019, pyglet is not able to open core OpenGL context causing
issues when using OpenGL versions higher than 2.1 on some platforms.
A version solving this issue is currently under development.

Homepage: https://bitbucket.org/pyglet/pyglet/wiki/Home

## py-sdl2

PySDL2 is a wrapper around the SDL2 library. It has no licensing restrictions.
In addition to the python package you need to install the shared library
from https://www.libsdl.org/download-2.0.php.

Homepage: https://github.com/marcusva/py-sdl2

## pyGLFW

Python bindings for GLFW (on GitHub: glfw/glfw). It is a ctypes wrapper which
keeps very close to the original GLFW API.
You will need to install the GLFW shared library from https://www.glfw.org/download.html.

Homepage: https://github.com/FlorianRhiem/pyGLFW


# Window Features

Each window type support the folloing features:

* Runtime resizing and viewport update
* Fullscreen mode
* Multisample antialiasing (MSAA) default at 4
* Keyboard events
* Mouse position and button events
* Vsync toggle
* Toggle mouse cursor on or off

Events are passed from the window to an example class in a
platform and framework-indenpedent way.


## Example Options
```bash
$> python.exe 00_empty_window.py -h
usage: 00_empty_window.py [-h] [-w {glfw,pyglet,pyqt5,sdl2}] [-fs] [-vs VSYNC]
                          [-s SAMPLES] [-c CURSOR]

optional arguments:
  -h, --help            show this help message and exit
  -w {glfw,pyglet,pyqt5,sdl2}, --window {glfw,pyglet,pyqt5,sdl2}
                        Name for the window type to use
  -fs, --fullscreen     Open the window in fullscreen mode
  -vs VSYNC, --vsync VSYNC
                        Enable or disable vsync
  -s SAMPLES, --samples SAMPLES
                        Specify the desired number of samples to use for
                        multisampling
  -c CURSOR, --cursor CURSOR
                        Enable or disable displaying the mouse cursor
```

The default window type is PyQt5 as this is for the most part work out of the
box on most platforms. It may not be the most performant window type
(also depending on platform) as I've seen `sdl2` and `glfw` yield much
smoother and higher frame rates.


# Creating Examples

Creating an example is fairly simple. We simply need to implement the `Example`
class and pass its reference into `run_example`. The example below is similar
to `00_empty_window.py` and works as a decent template with enough comments
to explain

```py
from window import Example, run_example


class MyExample(Example):
    # The minimum required OpenGL version (default 3.3)
    gl_version = (3, 3)
    # Title of the window
    title = "Empty Window"
    # Size of the window to open
    window_size = (1280, 720)
    # Lock aspect ratio letting the window class automatically
    # calculate the right viewport regardless of window size.
    # This can add black borders because the viewport is centered
    # vertically and horizontally.
    aspect_ratio = 16 / 9
    # Should users be able to resize the window?
    resizable = True

    def __init__(self, **kwargs):
        # ctx and wnd are mainly passed in there
        super().__init__(**kwargs)

        # The ModernGL context is avaialble
        # self.ctx.something()

        # A window reference is also availabel
        # self.wnd.size

    def render(self, time: float, frame_time: float):
        """
        Called every frame

        Params:
            time (float): Number of seconds elapsed since rendering started
            frame_time (float): Seconds since last frame
        """
        self.ctx.clear(0.2, 0.4, 0.7)
        # Render stuff here

    # ------------ OPTIONAL METHODS --------------

    def resize(self, width: int, heigh: int):
        """
        Pick window resizes in case we need to update internal states.
        """
        print("Window resized to", width, heigh)

    def key_event(self, key, action):
        """
        Handle key events in a generic way supporting all window types.
        wnd.keys references key constant for the selected window type.
        """
        if action == self.wnd.keys.ACTION_PRESS:
            if key == self.wnd.keys.SPACE:
                print("Space was pressed")

        if action == self.wnd.keys.ACTION_RELEASE:
            if key == self.wnd.keys.SPACE:
                print("Space was released")

    def mouse_position_event(self, x, y):
        """
        Mouse position reported in pixel position
        with the upper left corner as the origin
        """
        print("Mouse pos", x, y)

    def mouse_press_event(self, x, y, button):
        """Reports left and right mouse button presses + position"""
        if button == 1:
            print("Left mouse button pressed @", x, y)
        if button == 2:
            print("Right mouse button pressed @", x, y)

    def mouse_release_event(self, x, y, button):
        """Reports left and right mouse button releases + position"""
        if button == 1:
            print("Left mouse button released @", x, y)
        if button == 2:
            print("Right mouse button released @", x, y)


# Make the module importable so this example
# can be used by other examples!
if __name__ == '__main__':
    run_example(MyExample)
```