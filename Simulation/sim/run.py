"""Run the roaming algorithm in the simulator.

  (from the Simulation folder)
  py -3.12 -m sim.run --scenario clutter --seed 3 --png data/sim/run.png        one run, with a picture
  py -3.12 -m sim.run --scenario clutter --batch 20                              20 runs and the averages
  add --perfect for a car with no errors, --max-time 900 for the time limit (seconds)
"""
import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim.brain import Brain  # noqa: E402
from sim.car import CarAPI, Noise, SimCar  # noqa: E402
from sim.scenarios import SCENARIOS, START_HEADING, START_XY  # noqa: E402


@dataclass
class Result:
    scenario: str
    seed: int
    balls: int
    delivered: int
    seconds: float
    bumps: int
    driven_cm: float
    return_efficiency: float     # true path length / straight line, per delivery, averaged (1.0 = perfectly straight)
    drift_cm: float              # at the end: how far the brain's position guess is from the truth
    strategy: str = "map"        # which search the car used
    car: object = None
    brain: object = None
    world: object = None


def run_episode(scenario="clutter", seed=0, max_time=900.0, noise=None, balls=None, strategy="map", rechecks=1):
    world = SCENARIOS[scenario](seed) if balls is None else SCENARIOS[scenario](seed, balls)
    car = SimCar(world, START_XY, START_HEADING, noise or Noise(), seed=seed)
    bx, by = world.bin_xy
    brain = Brain(CarAPI(car), start_est=(START_XY[0] - bx, START_XY[1] - by), seed=seed, strategy=strategy,
                  bounds=(-bx, -by, world.width - bx, world.height - by), rechecks=rechecks)
    brain.run(max_time)
    eff = [d[0] / max(d[1], 1.0) for k, _, _, _, d in car.events if k == "deliver" and d]
    ex, ey = car.x - world.bin_xy[0], car.y - world.bin_xy[1]
    return Result(scenario, seed, len(world.balls) - sum(1 for k, *_ in car.events if k == "drop"), car.delivered, car.t, car.bumps,
                  car.driven_cm, statistics.mean(eff) if eff else float("nan"), math.hypot(brain.x - ex, brain.y - ey),
                  strategy, car, brain, world)


def summarise(r):
    return (f"{r.scenario} seed {r.seed}: delivered {r.delivered}/{r.balls}, {r.seconds:.0f} s, driven {r.driven_cm:.0f} cm, "
            f"bumps {r.bumps}, return path x{r.return_efficiency:.2f} of straight, position guess off by {r.drift_cm:.0f} cm")


def summarise_batch(results):
    got, total = sum(r.delivered for r in results), sum(r.balls for r in results)
    effs = [r.return_efficiency for r in results if r.return_efficiency == r.return_efficiency]
    return (f"ALL: delivered {got}/{total} ({100 * got / max(total, 1):.0f}%), mean time {statistics.mean(r.seconds for r in results):.0f} s, "
            f"bumps {sum(r.bumps for r in results)} in {len(results)} runs, return path x{statistics.mean(effs) if effs else float('nan'):.2f}, "
            f"position guess off by {statistics.mean(r.drift_cm for r in results):.0f} cm on average")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="clutter", choices=sorted(SCENARIOS))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--batch", type=int, default=0)
    ap.add_argument("--max-time", type=float, default=900.0)
    ap.add_argument("--perfect", action="store_true")
    ap.add_argument("--strategy", default="map", choices=["map", "lanes"], help="the search: map (stop and look) or lanes (lawnmower)")
    ap.add_argument("--png", default="")
    a = ap.parse_args(argv)
    noise = Noise.perfect() if a.perfect else None
    if a.batch:
        results = [run_episode(a.scenario, s, a.max_time, noise, strategy=a.strategy) for s in range(a.batch)]
        for r in results:
            print(summarise(r))
        print("\n" + summarise_batch(results))
        return results
    r = run_episode(a.scenario, a.seed, a.max_time, noise, strategy=a.strategy)
    print(summarise(r))
    if a.png:
        from sim.render import render
        render(r, a.png)
        print("picture:", a.png)
    return r


if __name__ == "__main__":
    main()
