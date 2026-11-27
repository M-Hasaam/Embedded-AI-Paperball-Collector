"""Telling a live motion sensor from a stuck one.

The car keeps sending the last reading when its MPU-6050 stops answering, so on screen the old values look live. A working
sensor never sends exactly the same line for long: its readings carry noise (in captured output a line repeated twice at most).
"""

FROZEN_S = 2.0        # the same line for this long: the sensor is stuck
SILENT_S = 1.5        # no line at all for this long: nothing arrives


class SensorWatch:
    def __init__(self, frozen_s=FROZEN_S, silent_s=SILENT_S):
        self.frozen_s, self.silent_s = frozen_s, silent_s
        self.reset()

    def reset(self):
        self.text = None                      # the newest line
        self._changed = self._arrived = 0.0   # when it last differed, and when it last arrived

    def reading(self, text, now):
        if text != self.text:
            self.text, self._changed = text, now
        self._arrived = now

    def check(self, now):
        """(None, 0.0) while all is well, ("frozen", seconds unchanged) or ("silent", seconds since the last line)."""
        if self.text is None:
            return None, 0.0
        if now - self._arrived >= self.silent_s:
            return "silent", now - self._arrived
        if self._arrived - self._changed >= self.frozen_s:
            return "frozen", self._arrived - self._changed
        return None, 0.0
