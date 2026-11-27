"""Ready-made arenas. The bin is at the left edge; the car starts in front of it facing out, as the real one would."""
import math
import random

from sim.world import Ball, Obstacle, World

BIN_XY = (40.0, 150.0)
START_XY = (80.0, 150.0)          # 40 cm in front of the bin, facing +x
START_HEADING = 0.0


def _place_balls(rng, world, n, margin=25.0):
    """n balls at random free spots (not near walls, the bin, the start or any obstacle that ever exists)."""
    balls, tries = [], 0
    while len(balls) < n and tries < 2000:
        tries += 1
        x, y = rng.uniform(margin, world.width - margin), rng.uniform(margin, world.height - margin)
        if math.hypot(x - BIN_XY[0], y - BIN_XY[1]) < 70 or math.hypot(x - START_XY[0], y - START_XY[1]) < 40:
            continue
        if any(o.x1 - margin < x < o.x2 + margin and o.y1 - margin < y < o.y2 + margin for o in world.obstacles):
            continue
        if any(math.hypot(x - b.x, y - b.y) < 30 for b in balls):
            continue
        balls.append(Ball(x, y))
    return balls


def empty(seed=0, balls=4):
    rng = random.Random(1000 + seed)
    w = World(400, 300, BIN_XY)
    w.balls = _place_balls(rng, w, balls)
    return w


def clutter(seed=0, balls=5):
    """A central block and a corner box (permanent), four chair legs (thin), and two boxes that are put there and taken away."""
    rng = random.Random(1000 + seed)
    w = World(400, 300, BIN_XY, obstacles=[
        Obstacle(180, 100, 220, 200, label="Table"),                    # permanent
        Obstacle(300, 20, 380, 60, label="Box"),                        # permanent: a box in the corner
        Obstacle(120, 60, 123, 63, thin=True, label="Chair leg"), Obstacle(150, 60, 153, 63, thin=True, label="Chair leg"),     # permanent
        Obstacle(120, 90, 123, 93, thin=True, label="Chair leg"), Obstacle(150, 90, 153, 93, thin=True, label="Chair leg"),
        Obstacle(100, 190, 140, 230, appear=60, vanish=240, label="Bag"),             # temporary: a bag left for a while
        Obstacle(260, 200, 300, 240, appear=150, vanish=500, label="Box"),           # temporary: a box that arrives later
    ])
    w.balls = _place_balls(rng, w, balls)
    return w


SCENARIOS = {"empty": empty, "clutter": clutter}
