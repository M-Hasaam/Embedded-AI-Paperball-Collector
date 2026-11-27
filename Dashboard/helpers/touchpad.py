"""Two-finger touchpad gestures on one widget, read from the Windows messages.

Tk passes a program only the vertical wheel: it never delivers sideways scrolling, and a pinch looks like any other scroll to
it. Windows itself sends a precision touchpad's gestures to a program as
    two fingers moving together    WM_MOUSEWHEEL (up/down) and WM_MOUSEHWHEEL (sideways)
    two fingers apart / together   WM_MOUSEWHEEL marked as with Ctrl held
so this reads those messages from the widget's window.

The callback runs inside Tk's own event loop, so it only notes what arrived; the window asks for it afterwards (take()).
Calling Tk from inside it could crash Python.
"""
import ctypes
import sys
from collections import deque
from ctypes import wintypes

WM_MOUSEWHEEL, WM_MOUSEHWHEEL, WM_NCDESTROY = 0x020A, 0x020E, 0x0082
MK_CONTROL, VK_CONTROL, GWLP_WNDPROC = 0x0008, 0x11, -4
AVAILABLE = sys.platform == "win32"


class TouchHook:
    """Takes over the wheel messages of the window `hwnd` (a widget's winfo_id()). take() hands out what arrived as
    (sideways, ctrl, delta, screen x, screen y); detach() gives the window back. Raises OSError if it cannot be done."""

    def __init__(self, hwnd):
        if not AVAILABLE:
            raise OSError("touchpad gestures are read from Windows messages")
        user32 = ctypes.WinDLL("user32", use_last_error=True)       # a private handle: its argument types are ours alone
        procedure = ctypes.WINFUNCTYPE(ctypes.c_ssize_t, wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM)
        self._set = user32.SetWindowLongPtrW
        self._set.argtypes, self._set.restype = [wintypes.HWND, ctypes.c_int, ctypes.c_void_p], ctypes.c_void_p
        self._pass_on = user32.CallWindowProcW
        self._pass_on.argtypes = [ctypes.c_void_p, wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
        self._pass_on.restype = ctypes.c_ssize_t
        self._key = user32.GetKeyState
        self._key.argtypes, self._key.restype = [ctypes.c_int], ctypes.c_short
        self.hwnd = hwnd
        self._events = deque(maxlen=500)
        self._procedure = procedure(self._on_message)          # kept for as long as the window may call it
        self._old = self._set(hwnd, GWLP_WNDPROC, ctypes.cast(self._procedure, ctypes.c_void_p))
        if not self._old:
            raise OSError(ctypes.get_last_error(), "could not read the window's messages")

    def _on_message(self, hwnd, message, wparam, lparam):
        old = self._old
        if message in (WM_MOUSEWHEEL, WM_MOUSEHWHEEL):
            delta = ctypes.c_short((wparam >> 16) & 0xFFFF).value
            ctrl = bool(wparam & MK_CONTROL) or self._key(VK_CONTROL) < 0
            x = ctypes.c_short(lparam & 0xFFFF).value
            y = ctypes.c_short((lparam >> 16) & 0xFFFF).value
            self._events.append((message == WM_MOUSEHWHEEL, ctrl, delta, x, y))
            return 0                                           # handled: Tk does not get it as well
        if message == WM_NCDESTROY:
            self.detach()
        return self._pass_on(old, hwnd, message, wparam, lparam)

    def take(self):
        taken = []
        while self._events:
            taken.append(self._events.popleft())
        return taken

    def detach(self):
        if self._old:
            self._set(self.hwnd, GWLP_WNDPROC, self._old)
            self._old = None
