"""Where things live under Dashboard/.

    dashboard.py      the program you run
    helpers/          the modules dashboard.py imports
    data/             everything the program writes: the car's address, the servo memory, the runner log, the distance
                      calibration, the taught grab spot (not kept in git)
    moves/            the recorded moves (JSON), made in the Moves window
    tests/            the automatic tests
"""
from pathlib import Path

DASHBOARD_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = DASHBOARD_DIR.parent
DATA_DIR = DASHBOARD_DIR / "data"
MOVES_DIR = DASHBOARD_DIR / "moves"
MODEL_PATH = REPO_DIR / "Training" / "paper_ball_detector" / "roboflow-paperball.onnx"


def data_file(name):
    """A path inside data/, making the folder first if it is not there yet."""
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    return DATA_DIR / name
