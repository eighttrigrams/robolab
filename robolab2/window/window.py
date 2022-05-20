import sys
import moderngl
from PyQt5 import QtCore, QtOpenGL, QtWidgets
from typing import Tuple

from moderngl_render.window.keys import Keys


class Window:
    """
    A basic window implementation using PyQt5 with the goal of
    creating an OpenGL context and handle keyboard and mouse input.

    This window bypasses Qt's own event loop to make things as flexible as possible.

    If you need to use the event loop and are using other features
    in Qt as well, this example can still be useful as a reference
    when creating your own window.
    """
    keys = Keys

    def __init__(self, title="Example", gl_version=(4,5), size=(1280, 720), resizable=True,
                 fullscreen=False, vsync=True, aspect_ratio=16/9, samples=4, cursor=True, **kwargs):

        """
        Args:
            title (str): The window title
            gl_version (tuple): Major and minor version of the opengl context to create
            size (tuple): Window size x, y
            resizable (bool): Should the window be resizable?
            fullscreen (bool): Open window in fullsceeen mode
            vsync (bool): Enable/disable vsync
            aspect_ratio (float): The desired aspect ratio. Can be set to None.
            samples (int): Number of MSAA samples for the default framebuffer
            cursor (bool): Enable/disable displaying the cursor inside the window
        """

        self.title = title
        self.gl_version = gl_version
        self.width, self.height = size
        self.resizable = resizable
        self.buffer_width, self.buffer_height = size
        self.fullscreen = fullscreen
        self.vsync = vsync
        self.aspect_ratio = aspect_ratio
        self.samples = samples
        self.cursor = cursor

        self.ctx = None  # type: moderngl.Context
        self.example = None
        self.frames = 0  # Frame counter
        self._close = False

        # Specify OpenGL context parameters
        gl = QtOpenGL.QGLFormat()
        gl.setVersion(self.gl_version[0], self.gl_version[1])
        gl.setProfile(QtOpenGL.QGLFormat.CoreProfile)
        gl.setDepthBufferSize(24)
        gl.setDoubleBuffer(True)
        gl.setSwapInterval(1 if self.vsync else 0)

        # Configure multisampling if needed
        if self.samples > 1:
            gl.setSampleBuffers(True)
            gl.setSamples(self.samples)

        # We need an application object, but we are bypassing the library's
        # internal event loop to avoid unnecessary work
        self.app = QtWidgets.QApplication([])

        # Create the OpenGL widget
        self.widget = QtOpenGL.QGLWidget(gl)
        self.widget.setWindowTitle(self.title)

        # If fullscreen we change the window to match the desktop on the primary screen
        if self.fullscreen:
            rect = QtWidgets.QDesktopWidget().screenGeometry()
            self.width = rect.width()
            self.height = rect.height()
            self.buffer_width = rect.width() * self.widget.devicePixelRatio()
            self.buffer_height = rect.height() * self.widget.devicePixelRatio()

        if self.resizable:
            # Ensure a valid resize policy when window is resizable
            size_policy = QtWidgets.QSizePolicy(
                QtWidgets.QSizePolicy.Expanding,
                QtWidgets.QSizePolicy.Expanding,
            )
            self.widget.setSizePolicy(size_policy)
            self.widget.resize(self.width, self.height)
        else:
            self.widget.setFixedSize(self.width, self.height)

        # Center the window on the screen if in window mode
        if not self.fullscreen:
            self.widget.move(QtWidgets.QDesktopWidget().rect().center() - self.widget.rect().center())

        # Needs to be set before show()
        self.widget.resizeGL = self.resize

        if not self.cursor:
            self.widget.setCursor(QtCore.Qt.BlankCursor)

        if self.fullscreen:
            self.widget.showFullScreen()
        else:
            self.widget.show()

        # We want mouse position events
        self.widget.setMouseTracking(True)

        # Override event functions
        self.widget.keyPressEvent = self.key_pressed_event
        self.widget.keyReleaseEvent = self.key_release_event
        self.widget.mouseMoveEvent = self.mouse_move_event
        self.widget.mousePressEvent = self.mouse_press_event
        self.widget.mouseReleaseEvent = self.mouse_release_event
        self.widget.closeEvent = self.close_event

        # Attach to the context
        self.ctx = moderngl.create_context(require=self.gl_version_code)

        # Ensure retina and 4k displays get the right viewport
        self.buffer_width = self.width * self.widget.devicePixelRatio()
        self.buffer_height = self.height * self.widget.devicePixelRatio()

        self.set_default_viewport()
        self.print_context_info()


    @property
    def gl_version_code(self) -> int:
        """
        Generates the version code integer for the selected OpenGL version.
        Example: gl_version (4, 1) returns 410
        """
        return self.gl_version[0] * 100 +  self.gl_version[1] * 10

    @property
    def is_closing(self) -> bool:
        """
        Returns: (bool) Is the window about to close?
        """
        return self._close

    def close(self):
        """
        Signal for the window to close
        """
        self._close = True

    def render(self, time: float, frame_time: float):
        """
        Renders the assigned example

        Args:
            time (float): Current time in seconds
            frame_time (float): Delta time from last frame in seconds
        """
        self.example.render(time, frame_time)

    def set_default_viewport(self):
        """
        Calculates the viewport based on the configured aspect ratio.
        Will add black borders and center the viewport if the window
        do not match the configured viewport.

        If aspect ratio is None the viewport will be scaled
        to the entire window size regardless of size.
        """
        if self.aspect_ratio:
            expected_width = int(self.buffer_height * self.aspect_ratio)
            expected_height = int(expected_width / self.aspect_ratio)

            if expected_width > self.buffer_width:
                expected_width = self.buffer_width
                expected_height =  int(expected_width / self.aspect_ratio)

            blank_space_x = self.buffer_width - expected_width
            blank_space_y = self.buffer_height - expected_height

            self.ctx.viewport = (
                blank_space_x // 2,
                blank_space_y // 2,
                expected_width,
                expected_height,
            )
        else:
            self.ctx.viewport = (0, 0, self.buffer_width, self.buffer_height)

    def print_context_info(self):
        """
        Prints moderngl context info.
        """
        print("Context Version:")
        print('ModernGL:', moderngl.__version__)
        print('vendor:', self.ctx.info['GL_VENDOR'])
        print('renderer:', self.ctx.info['GL_RENDERER'])
        print('version:', self.ctx.info['GL_VERSION'])
        print('python:', sys.version)
        print('platform:', sys.platform)
        print('code:', self.ctx.version_code)

    def swap_buffers(self):
        """
        Swap buffers, set viewport, trigger events and increment frame counter
        """
        self.widget.swapBuffers()
        self.set_default_viewport()
        self.app.processEvents()
        self.frames += 1

    def resize(self, width: int,  height: int):
        """
        Replacement for Qt's resizeGL method.
        """
        self.width = width // self.widget.devicePixelRatio()
        self.height = height // self.widget.devicePixelRatio()
        self.buffer_width = width
        self.buffer_height = height

        if self.ctx:
            self.set_default_viewport()

        if self.example:
            self.example.resize(self.buffer_width, self.buffer_height)


    def key_pressed_event(self, event):
        """
        Process Qt key press events forwarding them to the example
        """
        if event.key() == self.keys.ESCAPE:
            self.close()

        self.example.key_event(event.key(), self.keys.ACTION_PRESS)

    def key_release_event(self, event):
        """
        Process Qt key release events forwarding them to the example
        """
        self.example.key_event(event.key(), self.keys.ACTION_RELEASE)

    def mouse_move_event(self, event):
        """
        Forward mouse cursor position events to the example
        """
        self.example.mouse_position_event(event.x(), event.y())

    def mouse_press_event(self, event):
        """
        Forward mouse press events to the example
        """
        # Support left and right mouse button for now
        if event.button() not in [1, 2]:
            return

        self.example.mouse_press_event(event.x(), event.y(), event.button())

    def mouse_release_event(self, event):
        """
        Forward mouse release events to the example
        """
        # Support left and right mouse button for now
        if event.button() not in [1, 2]:
            return

        self.example.mouse_release_event(event.x(), event.y(), event.button())

    def close_event(self, event):
        """
        Detect the standard PyQt close events to make users happy
        """
        self.close()

    def destroy(self):
        """
        Quit the Qt application to exit the window gracefully
        """
        QtCore.QCoreApplication.instance().quit()

    @property
    def size(self) -> Tuple[int, int]:
        """
        Returns: (width, height) tuple with the current window size
        """
        return self.width, self.height

    @property
    def buffer_size(self) -> Tuple[int, int]:
        """
        Returns: (with, heigh) tuple with the current window buffer size
        """
        return self.buffer_width, self.buffer_height
