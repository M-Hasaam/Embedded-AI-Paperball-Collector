"""Run by test_path_zoom.py in a process of its own: hundreds of gesture messages while Tk's main loop runs, then close."""
import ctypes
import sys
import tempfile
import tkinter as tk
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from helpers.path_settings import PathSettings  # noqa: E402

root = tk.Tk()
root.withdraw()
app = types.SimpleNamespace(root=root, tracker=dashboard.PathTracker(), path_settings=PathSettings(), path_window=None,
                            runner=types.SimpleNamespace(status="Idle", active=False))
app.path_return = app.path_abort = app.path_clear = app.path_test_drive = lambda: None
app.path_run_plan = app.apply_path_settings = lambda *a, **k: None
app._button = dashboard.Dashboard._button.__get__(app)
user32 = ctypes.WinDLL("user32")
user32.PostMessageW.argtypes = user32.SendMessageW.argtypes = [ctypes.c_void_p, ctypes.c_uint, ctypes.c_size_t, ctypes.c_ssize_t]
handled = 0
ROUND = 40                                  # messages of each kind in one round


def one_round(round_number):
    global handled
    window = dashboard.PathWindow(app, touch_log=Path(tempfile.gettempdir()) / "touch_check_log.txt")
    window.win.geometry("660x760+-4000+-4000")
    original = window._gesture

    def counted(*args):
        global handled
        handled += 1
        original(*args)

    window._gesture = counted

    def send(n):
        if n == ROUND:
            window.win.update()                     # let the last posted messages arrive
            window._poll_touch()
            window.close()
            if round_number < 2:
                root.after(20, lambda: one_round(round_number + 1))
            else:
                root.after(20, root.quit)
            return
        hwnd = window.canvas.winfo_id()
        for message, flags in ((0x020A, 0), (0x020E, 0)):
            wparam = ((((-1) ** n * 30) & 0xFFFF) << 16) | flags
            # one posted (as input arrives) and, every tenth time, one sent from inside a Tk callback
            (user32.SendMessageW if n % 10 == 0 else user32.PostMessageW)(hwnd, message, wparam, 0)
        root.after(5, lambda: send(n + 1))

    root.after(100, lambda: send(0))


one_round(0)
root.after(45000, root.quit)
root.mainloop()
last = dashboard.PathWindow(app, touch_log=Path(tempfile.gettempdir()) / "touch_check_log.txt")
last.win.update()
root.destroy()                               # without close(): the window gives itself back as it goes
print(f"handled {handled} gestures")
