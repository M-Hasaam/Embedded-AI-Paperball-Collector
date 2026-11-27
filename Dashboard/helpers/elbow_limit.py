"""Where the elbow may go at each shoulder angle, for the red zones on the dashboard's elbow slider.

The car enforces the limit itself (AI_Car.ino, elbowWindowAt); this is a copy of its table, kept the same by
tests/test_elbow_zones.py. Measured on the arm and plotted in docs/arm-limits.
"""

SHOULDER = (40, 60, 70, 80, 90, 100, 110, 115, 120, 130, 140, 150, 160, 170)
LOWEST = (135, 120, 105, 100, 100, 100, 100, 100, 95, 95, 95, 90, 100, 120)
HIGHEST = (180, 180, 180, 180, 180, 180, 180, 180, 180, 170, 170, 160, 150, 145)
MARGIN = 0                       # extra degrees kept inside the table's limits (LIMIT_MARGIN in the car)
SERVO_RANGE = (80, 180)          # the elbow servo's own range


def elbow_window(shoulder):
    """(lowest, highest) elbow angle allowed with the shoulder at `shoulder`. Between two rows of the table the tighter one
    counts, and exactly on a row both its neighbours do."""
    low, high = SERVO_RANGE
    shoulder = min(max(shoulder, SHOULDER[0]), SHOULDER[-1])
    for k in range(len(SHOULDER) - 1):
        if SHOULDER[k] <= shoulder <= SHOULDER[k + 1]:
            low = max(low, LOWEST[k], LOWEST[k + 1])
            high = min(high, HIGHEST[k], HIGHEST[k + 1])
    if low > SERVO_RANGE[0]:
        low += MARGIN
    if high < SERVO_RANGE[1]:
        high -= MARGIN
    if low > high:
        low = high = (low + high) // 2
    return low, high
