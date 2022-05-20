import time


def run_example(example_cls, ctx = None, wnd = None, prog = None, scn=None):

    wnd.example = example_cls(ctx=ctx, wnd=wnd, prog=prog, scn=scn)

    start_time = time.time()
    current_time = start_time
    prev_time = start_time
    frame_time = 0

    while not wnd.is_closing:
        current_time, prev_time = time.time(), current_time
        frame_time = max(current_time - prev_time, 1 / 1000)

        wnd.render(current_time - start_time, frame_time)
        wnd.swap_buffers()

    duration = time.time() - start_time
    wnd.destroy()
    print("Duration: {0:.2f}s @ {1:.2f} FPS".format(duration, wnd.frames / duration))