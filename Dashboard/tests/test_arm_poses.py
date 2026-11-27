import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402
from helpers.elbow_limit import elbow_window  # noqa: E402


class ArmPoseButtonsTest(unittest.TestCase):
    def app(self):
        self.sent = []
        app = types.SimpleNamespace(set_to={})
        app.servo_scales = [types.SimpleNamespace(set=lambda angle, i=i: app.set_to.__setitem__(i, angle)) for i in range(4)]
        app._send_servo = self.sent.append
        app._set_pose = lambda angles: dashboard.Dashboard._set_pose(app, angles)
        return app

    def test_all_closed_moves_every_servo_to_the_closed_pose(self):
        app = self.app()
        dashboard.Dashboard._set_closed_servos(app)
        self.assertEqual(app.set_to, {0: 100, 1: 135, 2: 40, 3: 90})          # gripper, elbow, shoulder, servo 4
        self.assertEqual(self.sent, [0, 1, 2, 3])

    def test_all_default_still_does_what_it_did(self):
        app = self.app()
        dashboard.Dashboard._set_default_servos(app)
        self.assertEqual(app.set_to, dict(enumerate(dashboard.SERVO_DEFAULT_ANGLES)))
        self.assertEqual(self.sent, [0, 1, 2, 3])

    def test_the_closed_pose_is_one_the_car_allows(self):
        angles = dashboard.SERVO_CLOSED_ANGLES
        for i, angle in enumerate(angles):
            self.assertTrue(dashboard.SERVO_MIN_ANGLES[i] <= angle <= dashboard.SERVO_MAX_ANGLES[i], i)
            self.assertTrue(dashboard.SERVO_RANGE_DEFAULT[i][0] <= angle <= dashboard.SERVO_RANGE_DEFAULT[i][1], i)
        low, high = elbow_window(angles[2])
        self.assertTrue(low <= angles[1] <= high)

    def test_the_elbow_is_allowed_wherever_the_shoulder_starts_from(self):
        # the shoulder swings to its closed angle while the elbow goes to 135: the limit must never hold the elbow back
        for shoulder in range(40, 171):
            low, high = elbow_window(shoulder)
            self.assertTrue(low <= dashboard.SERVO_CLOSED_ANGLES[1] <= high, shoulder)


if __name__ == "__main__":
    unittest.main()
