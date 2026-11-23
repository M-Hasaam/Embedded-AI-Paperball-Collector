import sys
import types
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import dashboard  # noqa: E402


class BoolVar:
    def __init__(self, value=False):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class ElbowBypassTest(unittest.TestCase):
    def setUp(self):
        self.asked = []
        self.answer = True
        self.original = dashboard.messagebox.askokcancel

        def ask(title, message, **kw):
            self.asked.append(title)
            return self.answer

        dashboard.messagebox.askokcancel = ask
        self.addCleanup(lambda: setattr(dashboard.messagebox, "askokcancel", self.original))
        self.sent = []
        self.app = types.SimpleNamespace(elbow_bypass_var=BoolVar(), _send=self.sent.append, root=None)

    def test_turning_it_on_asks_first_and_sends_b1(self):
        self.app.elbow_bypass_var.set(True)
        dashboard.Dashboard._elbow_bypass_toggled(self.app)
        self.assertEqual(len(self.asked), 1)
        self.assertEqual(self.sent, ["B1"])
        self.assertTrue(self.app.elbow_bypass_var.get())

    def test_declining_leaves_the_limit_on(self):
        self.answer = False
        self.app.elbow_bypass_var.set(True)
        dashboard.Dashboard._elbow_bypass_toggled(self.app)
        self.assertEqual(self.sent, [])
        self.assertFalse(self.app.elbow_bypass_var.get())

    def test_turning_it_off_sends_b0_without_asking(self):
        self.app.elbow_bypass_var.set(False)
        dashboard.Dashboard._elbow_bypass_toggled(self.app)
        self.assertEqual(self.sent, ["B0"])
        self.assertEqual(self.asked, [])

    def test_reconnecting_puts_the_limit_back_on(self):
        self.app.elbow_bypass_var.set(True)
        dashboard.Dashboard._reset_elbow_bypass(self.app)
        self.assertFalse(self.app.elbow_bypass_var.get())
        self.assertEqual(self.sent, ["B0"])


if __name__ == "__main__":
    unittest.main()
