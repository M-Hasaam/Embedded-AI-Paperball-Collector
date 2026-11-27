"""A window for the simulator: pick the arena and how wrong the car is, press Run, and watch the run play back like a video (play, pause,
speed, a slider through the time), or look at the whole route at once. Click a line of "what happened" to jump to that moment.

    py -3.12 -m sim.gui
"""
import queue
import sys
import threading
import time
import tkinter as tk
from collections import namedtuple
from pathlib import Path
from tkinter import filedialog, ttk

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PIL import ImageTk  # noqa: E402

from sim import render as R  # noqa: E402
from sim.car import Noise  # noqa: E402
from sim.run import run_episode  # noqa: E402
from sim.scenarios import SCENARIOS  # noqa: E402
from sim.timeline import clock, events_of, plain_batch, plain_summary, state_at  # noqa: E402

BG, PANEL, TEXT, MUTED, ACCENT = "#0f141a", "#1a212a", "#e2e8f0", "#94a3b8", "#3b82f6"
SPEEDS = ("1x", "5x", "20x", "60x", "200x")
ARM_SPEED = 2.0                 # playback never goes faster than this while the arm is moving (a pick takes 6 s: three to watch)


def _hex(rgb):
    return "#%02x%02x%02x" % tuple(rgb[:3])


# the colours in the picture, explained
LEGEND = ((R.CAR, "Car (the nose points where it drives)"), (R.ROUTE_COLOUR, "Planned lane route (lanes search)"), (R.CAMERA_VIEW, "Camera view (36 deg wide, 2.2 m)"),
          (R.CAR, "Distance sensor beam (drawn short)"), (R.GOOD, "Floor already looked at"),
          (R.BAD, "Obstacles and walls it has found"), (R.PENDING_BLUE, "Ball seen, to come back for"), (R.BALL, "Ball"), (R.BIN, "Bin"), (R.OBSTACLE_EDGE, "Fixed obstacle"),
          (R.TEMP_EDGE, "Obstacle that comes and goes"), (R.LEG, "Chair leg"), (R.TRAIL, "Where it just drove"))

STRATEGY_LABELS = {"map": "Map-guided scanning", "lanes": "Lanes (lawnmower)"}
STRATEGY_HINTS = {"map": "Stops where the most floor has never been looked at, spins to look, plans a path to the next stop. Finds balls quickly.",
                  "lanes": "Drives straight lanes across the arena, 70 cm apart, looking ahead at every step. Hardly spins; takes longer to finish a pass."}
ARENAS = {"empty": "Empty arena: just the bin and the balls",
          "clutter": "Cluttered: a table, a box, chair legs, and two boxes that come and go"}

Slider = namedtuple("Slider", "key label lo hi default unit field convert")
IDENT = lambda v: v                                    # noqa: E731

# Each slider is one way the real car is wrong. `convert` turns what is on screen into the Noise field.
SLIDERS = (
    Slider("gyro", "Gyro drifts", 0.0, 0.3, 0.05, "deg/s", "gyro_bias_dps", IDENT),
    Slider("turn", "Turns are off by", 0.0, 5.0, 1.0, "deg", "turn_resid_deg", IDENT),
    Slider("speed", "Speed differs from expected by", 0, 20, 8, "%", "speed_err", lambda v: v / 100.0),
    Slider("veer", "Drives crooked (to the left)", 0.0, 0.15, 0.03, "deg/cm", "veer_deg_per_cm", IDENT),
    Slider("thin", "Ultrasonic misses chair legs", 0, 100, 50, "%", "thin_hit", lambda v: 1.0 - v / 100.0),
    Slider("usdrop", "Ultrasonic gives no echo", 0, 20, 3, "%", "us_dropout", lambda v: v / 100.0),
    Slider("cammiss", "Camera misses a ball", 0, 50, 8, "%", "cam_detect", lambda v: 1.0 - v / 100.0),
    Slider("camfalse", "Camera sees a ball that is not there", 0, 10, 2, "%", "cam_false_pos", lambda v: v / 100.0),
)


def build_noise(values, perfect):
    """The Noise for the slider values ({key: number}); a perfect car ignores them."""
    if perfect:
        return Noise.perfect()
    noise = Noise()
    for s in SLIDERS:
        setattr(noise, s.field, s.convert(values[s.key]))
    return noise


def run_job(arena, seed, balls, minutes, noise, batch, strategy="map", rechecks=1):
    """Everything one press of Run does (off the screen's thread): one run, or `batch` runs with consecutive seeds."""
    seeds = range(seed, seed + batch) if batch else [seed]
    return [run_episode(arena, s, minutes * 60.0, noise, balls, strategy=strategy, rechecks=rechecks) for s in seeds]


class SimWindow:
    def __init__(self, root):
        self.root = root
        root.title("Paper-ball collector simulator")
        root.configure(bg=BG)
        root.minsize(1000, 640)
        self.busy = False
        self.results, self.current = [], None
        self.t, self.playing, self.overview = 0.0, False, False
        self.event_times, self.events = [], []
        self.shown_image = None
        self.queue = queue.Queue()
        self._after = None
        self._last_tick = 0.0
        self._seeking = False
        self._style()

        self.arena_var = tk.StringVar(value="clutter")
        self.seed_var = tk.IntVar(value=8)
        self.balls_var = tk.IntVar(value=5)
        self.strategy_var = tk.StringVar(value=STRATEGY_LABELS["map"])
        self.rechecks_var = tk.IntVar(value=1)          # one extra round: measured to recover most of the balls the camera missed
        self.minutes_var = tk.IntVar(value=15)
        self.batch_var = tk.IntVar(value=10)
        self.perfect_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="Ready: press Run once")
        self.time_var = tk.StringVar(value="0:00 / 0:00")
        self.speed_var = tk.StringVar(value="20x")
        self.overview_var = tk.BooleanVar(value=False)
        self.guess_var = tk.BooleanVar(value=False)
        self.slow_arm_var = tk.BooleanVar(value=True)
        self.camera_var = tk.BooleanVar(value=True)
        self.route_var = tk.BooleanVar(value=True)
        self.seen_var = tk.BooleanVar(value=True)
        self.pick_var = tk.StringVar()
        self.slider_vars, self.slider_labels, self.slider_scales = {}, {}, {}

        left = ttk.Frame(root, style="Panel.TFrame", padding=12)
        left.grid(row=0, column=0, sticky="ns", padx=(10, 5), pady=10)
        right = ttk.Frame(root, style="TFrame")
        right.grid(row=0, column=1, sticky="nsew", padx=(5, 10), pady=10)
        root.columnconfigure(1, weight=1)
        root.rowconfigure(0, weight=1)
        self._build_left(left)
        self._build_right(right)
        self._perfect_changed()
        root.bind("<Destroy>", self._on_destroy, add="+")

    # -- look --
    def _style(self):
        st = ttk.Style(self.root)
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        st.configure("TFrame", background=BG)
        st.configure("Panel.TFrame", background=PANEL)
        st.configure("TLabel", background=PANEL, foreground=TEXT)
        st.configure("Muted.TLabel", background=PANEL, foreground=MUTED)
        st.configure("Head.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 11, "bold"))
        st.configure("Status.TLabel", background=BG, foreground=MUTED)
        st.configure("Time.TLabel", background=BG, foreground=TEXT, font=("Consolas", 10))
        st.configure("TCheckbutton", background=PANEL, foreground=TEXT)
        st.configure("Bar.TCheckbutton", background=BG, foreground=TEXT)
        st.configure("TButton", padding=6)
        st.configure("Run.TButton", background=ACCENT, foreground="white", font=("Segoe UI", 10, "bold"))
        st.map("Run.TButton", background=[("disabled", "#30363d"), ("active", "#60a5fa")])
        st.configure("Play.TButton", background=ACCENT, foreground="white", font=("Segoe UI", 10, "bold"), padding=(10, 4))
        st.map("Play.TButton", background=[("disabled", "#30363d"), ("active", "#60a5fa")])
        st.configure("Horizontal.TScale", background=ACCENT, troughcolor="#0d1117", bordercolor=PANEL, lightcolor=ACCENT, darkcolor=ACCENT)
        st.map("Horizontal.TScale", background=[("disabled", "#30363d")], lightcolor=[("disabled", "#30363d")], darkcolor=[("disabled", "#30363d")])
        st.configure("TSpinbox", fieldbackground="#0d1117", foreground=TEXT, arrowcolor=TEXT)
        st.configure("TCombobox", fieldbackground="#0d1117", foreground=TEXT, arrowcolor=TEXT, background=PANEL)
        st.map("TCombobox", fieldbackground=[("readonly", "#0d1117")], foreground=[("readonly", TEXT)],
               selectbackground=[("readonly", "#0d1117")], selectforeground=[("readonly", TEXT)])
        self.root.option_add("*TCombobox*Listbox.background", "#0d1117")
        self.root.option_add("*TCombobox*Listbox.foreground", TEXT)

    def _row(self, parent, label, widget_factory):
        row = ttk.Frame(parent, style="Panel.TFrame")
        row.pack(fill="x", pady=2)
        ttk.Label(row, text=label).pack(side="left")
        w = widget_factory(row)
        w.pack(side="right")
        return w

    def _build_left(self, p):
        ttk.Label(p, text="Arena", style="Head.TLabel").pack(anchor="w")
        combo = self._row(p, "Layout", lambda r: ttk.Combobox(r, textvariable=self.arena_var, values=sorted(SCENARIOS), state="readonly", width=12))
        self.arena_hint = ttk.Label(p, text=ARENAS["clutter"], style="Muted.TLabel", wraplength=270, justify="left")
        self.arena_hint.pack(anchor="w", pady=(0, 4))
        combo.bind("<<ComboboxSelected>>", lambda e: self.arena_hint.configure(text=ARENAS.get(self.arena_var.get(), "")))
        self._row(p, "Random layout (seed)", lambda r: ttk.Spinbox(r, from_=0, to=9999, textvariable=self.seed_var, width=7))
        self._row(p, "Balls on the floor", lambda r: ttk.Spinbox(r, from_=1, to=12, textvariable=self.balls_var, width=7))
        self._row(p, "Time limit (minutes)", lambda r: ttk.Spinbox(r, from_=1, to=60, textvariable=self.minutes_var, width=7))
        self.strategy_combo = self._row(p, "Search", lambda r: ttk.Combobox(r, textvariable=self.strategy_var, values=list(STRATEGY_LABELS.values()),
                                                                           state="readonly", width=19))
        self.strategy_hint = ttk.Label(p, text=STRATEGY_HINTS["map"], style="Muted.TLabel", wraplength=270, justify="left")
        self.strategy_hint.pack(anchor="w", pady=(0, 4))
        self.strategy_combo.bind("<<ComboboxSelected>>", lambda e: self.strategy_changed())
        self._row(p, "Extra check rounds", lambda r: ttk.Spinbox(r, from_=0, to=3, textvariable=self.rechecks_var, width=7))

        ttk.Label(p, text="Run", style="Head.TLabel").pack(anchor="w", pady=(12, 4))
        self.run_button = ttk.Button(p, text="Run once", style="Run.TButton", command=lambda: self.start_run(batch=False))
        self.run_button.pack(fill="x")
        row = ttk.Frame(p, style="Panel.TFrame")
        row.pack(fill="x", pady=(6, 0))
        self.batch_button = ttk.Button(row, text="Run a batch of", command=lambda: self.start_run(batch=True))
        self.batch_button.pack(side="left")
        ttk.Spinbox(row, from_=2, to=50, textvariable=self.batch_var, width=4).pack(side="left", padx=6)
        ttk.Label(row, text="layouts", style="Muted.TLabel").pack(side="left")
        self.save_button = ttk.Button(p, text="Save the picture...", command=self.save_picture)
        self.save_button.pack(fill="x", pady=(10, 0))

        ttk.Label(p, text="How wrong is the car?", style="Head.TLabel").pack(anchor="w", pady=(14, 2))
        ttk.Checkbutton(p, text="Perfect car (no errors at all)", variable=self.perfect_var, command=self._perfect_changed).pack(anchor="w")
        for s in SLIDERS:
            self._slider(p, s)
        ttk.Button(p, text="Reset the errors to normal", command=self._reset_sliders).pack(anchor="w", pady=(6, 0))

    @staticmethod
    def _fmt(s):
        return "{:.2f}" if s.hi <= 1 else ("{:.1f}" if s.hi <= 5 else "{:.0f}")

    def _slider(self, parent, s):
        var = tk.DoubleVar(value=s.default)
        self.slider_vars[s.key] = var
        frame = ttk.Frame(parent, style="Panel.TFrame")
        frame.pack(fill="x", pady=1)
        top = ttk.Frame(frame, style="Panel.TFrame")
        top.pack(fill="x")
        ttk.Label(top, text=s.label, style="Muted.TLabel").pack(side="left")
        value = ttk.Label(top, text="", style="Muted.TLabel")
        value.pack(side="right")
        self.slider_labels[s.key] = value

        def changed(_=None, s=s, var=var, value=value):
            value.configure(text=f"{self._fmt(s).format(var.get())} {s.unit}")

        scale = ttk.Scale(frame, from_=s.lo, to=s.hi, variable=var, command=changed)
        scale.pack(fill="x")
        self.slider_scales[s.key] = scale
        changed()

    def _build_right(self, p):
        p.columnconfigure(0, weight=1)
        p.rowconfigure(0, weight=1)
        self.picture = tk.Label(p, bg="#0b0f14", fg=MUTED, text="Press  Run once  to watch the car work.", font=("Segoe UI", 12))
        self.picture.grid(row=0, column=0, sticky="nsew")

        bar = ttk.Frame(p, style="TFrame")
        bar.grid(row=1, column=0, sticky="ew", pady=(8, 0))
        bar.columnconfigure(1, weight=1)
        self.play_button = ttk.Button(bar, text="▶  Play", style="Play.TButton", command=self.toggle_play, state="disabled")
        self.play_button.grid(row=0, column=0, padx=(0, 8))
        self.timeline = ttk.Scale(bar, from_=0, to=1, command=self._slider_moved)
        self.timeline.grid(row=0, column=1, sticky="ew")
        ttk.Label(bar, textvariable=self.time_var, style="Time.TLabel").grid(row=0, column=2, padx=8)
        ttk.Label(bar, text="speed", style="Status.TLabel").grid(row=0, column=3)
        speed = ttk.Combobox(bar, textvariable=self.speed_var, values=SPEEDS, state="readonly", width=5)
        speed.grid(row=0, column=4, padx=(4, 0))

        opts = ttk.Frame(p, style="TFrame")
        opts.grid(row=2, column=0, sticky="ew", pady=(4, 0))
        ttk.Checkbutton(opts, text="Show the whole run at once", variable=self.overview_var, style="Bar.TCheckbutton",
                        command=self.refresh).pack(side="left")
        ttk.Checkbutton(opts, text="Show where the car THINKS it is", variable=self.guess_var, style="Bar.TCheckbutton",
                        command=self.refresh).pack(side="left", padx=14)
        ttk.Checkbutton(opts, text="Show what the camera sees", variable=self.camera_var, style="Bar.TCheckbutton",
                        command=self.refresh).pack(side="left", padx=(0, 14))
        ttk.Checkbutton(opts, text="Show the car's own map", variable=self.seen_var, style="Bar.TCheckbutton",
                        command=self.refresh).pack(side="left", padx=(0, 14))
        ttk.Checkbutton(opts, text="Show the lane route", variable=self.route_var, style="Bar.TCheckbutton",
                        command=self.refresh).pack(side="left", padx=(0, 14))
        ttk.Checkbutton(opts, text="Slow down for arm moves", variable=self.slow_arm_var, style="Bar.TCheckbutton").pack(side="left")
        self.pick_combo = ttk.Combobox(opts, textvariable=self.pick_var, state="readonly", width=24)
        self.pick_combo.bind("<<ComboboxSelected>>", lambda e: self.choose_run(self.pick_combo.current()))

        legend = ttk.Frame(p, style="TFrame")
        legend.grid(row=3, column=0, sticky="ew", pady=(6, 0))
        for colour, name in LEGEND:
            tk.Label(legend, text="●", fg=_hex(colour), bg=BG, font=("Segoe UI", 11)).pack(side="left")
            tk.Label(legend, text=name, fg=MUTED, bg=BG, font=("Segoe UI", 9)).pack(side="left", padx=(0, 10))

        box = ttk.Frame(p, style="TFrame")
        box.grid(row=4, column=0, sticky="ew", pady=(8, 0))
        box.columnconfigure(0, weight=1)
        self.text = tk.Text(box, height=7, bg="#0b0f14", fg=TEXT, relief="flat", wrap="word", font=("Segoe UI", 10), state="disabled",
                            padx=10, pady=6)
        sb = ttk.Scrollbar(box, command=self.text.yview)
        self.text.configure(yscrollcommand=sb.set)
        self.text.tag_configure("head", font=("Segoe UI", 10, "bold"), spacing1=4)
        self.text.tag_configure("muted", foreground=MUTED)
        self.text.tag_configure("event", foreground="#93c5fd")
        self.text.grid(row=0, column=0, sticky="ew")
        sb.grid(row=0, column=1, sticky="ns")
        ttk.Label(p, textvariable=self.status_var, style="Status.TLabel").grid(row=5, column=0, sticky="w", pady=(4, 0))

    def legend_names(self):
        return [name for _, name in LEGEND]

    # -- the sliders --
    def strategy_key(self):
        """'map' or 'lanes', for what is chosen in the dropdown."""
        for key, label in STRATEGY_LABELS.items():
            if label == self.strategy_var.get():
                return key
        return "map"

    def strategy_changed(self):
        self.strategy_hint.configure(text=STRATEGY_HINTS[self.strategy_key()])

    def _perfect_changed(self):
        state = "disabled" if self.perfect_var.get() else "normal"
        for scale in self.slider_scales.values():
            scale.configure(state=state)

    def _reset_sliders(self):
        for s in SLIDERS:
            self.slider_vars[s.key].set(s.default)
            self.slider_labels[s.key].configure(text=f"{self._fmt(s).format(s.default)} {s.unit}")

    # -- running --
    def summary(self):
        return self.text.get("1.0", "end").strip()

    def _set_busy(self, busy):
        self.busy = busy
        state = "disabled" if busy else "normal"
        for b in (self.run_button, self.batch_button):
            b.configure(state=state)

    def start_run(self, batch):
        if self.busy:
            return
        try:
            values = {k: v.get() for k, v in self.slider_vars.items()}
            noise = build_noise(values, self.perfect_var.get())
            args = (self.arena_var.get(), int(self.seed_var.get()), int(self.balls_var.get()), float(self.minutes_var.get()), noise,
                    int(self.batch_var.get()) if batch else 0, self.strategy_key(), int(self.rechecks_var.get()))
        except (tk.TclError, ValueError) as e:
            self.finish_error(f"Check the numbers on the left: {e}")
            return
        self.pause()
        self._set_busy(True)
        n = args[5] or 1
        self.status_var.set(f"Simulating {n} {'layouts' if n > 1 else 'layout'}...  (a long run takes a few seconds)")
        threading.Thread(target=self._worker, args=args, daemon=True).start()
        self.root.after(100, self._poll)

    def _worker(self, *args):
        try:
            self.queue.put(("done", run_job(*args)))
        except Exception as e:                                  # shown in the window, not lost in a console nobody sees
            self.queue.put(("error", f"{type(e).__name__}: {e}"))

    def _poll(self):
        try:
            kind, payload = self.queue.get_nowait()
        except queue.Empty:
            self.root.after(100, self._poll)
            return
        if kind == "done":
            self.finish(payload)
        else:
            self.finish_error(payload)

    def finish(self, results):
        self.results = results
        choices = self.run_choices()
        self.pick_combo.configure(values=choices)
        if len(results) > 1:
            self.pick_combo.pack(side="right")
        else:
            self.pick_combo.pack_forget()
        self.status_var.set("Done. Press Play to watch again, drag the slider, or click a line below to jump to it.")
        self._set_busy(False)
        self.play_button.configure(state="normal")
        self.choose_run(0)

    def finish_error(self, message):
        self._write([("Could not run: " + message, "head")])
        self.status_var.set("Error")
        self._set_busy(False)

    def run_choices(self):
        return [f"Layout {r.seed}: {r.delivered} of {r.balls} balls" for r in self.results]

    def choose_run(self, index):
        """Look at one of the runs (a batch has several)."""
        self.current = self.results[index]
        self.pick_var.set(self.run_choices()[index])
        self.t = 0.0
        self.timeline.configure(to=max(1.0, self.current.seconds))
        self._fill_text()
        self.refresh()
        self.play()

    # -- the text under the picture --
    def _write(self, parts):
        self.text.configure(state="normal")
        self.text.delete("1.0", "end")
        for string, tag in parts:
            self.text.insert("end", string, tag)
        self.text.configure(state="disabled")

    def _fill_text(self):
        r = self.current
        parts = []
        if len(self.results) > 1:
            parts += [(line + "\n", "head") for line in plain_batch(self.results)]
            parts.append((f"\nLayout {r.seed} (the one shown):\n", "muted"))
        parts += [(line + "\n", "") for line in plain_summary(r)]
        parts += [("\nWhat happened (click a line to jump there):\n", "head")]
        self.event_times = []
        self.events = events_of(r)
        for i, (t, line) in enumerate(self.events):
            self.event_times.append(t)
            parts.append((f"{clock(t):>6}   {line}\n", f"ev{i}"))
        self._write(parts)
        for i in range(len(self.event_times)):
            tag = f"ev{i}"
            self.text.tag_configure(tag, foreground="#93c5fd")
            self.text.tag_bind(tag, "<Button-1>", lambda e, i=i: self.jump_to_event(i))
            self.text.tag_bind(tag, "<Enter>", lambda e: self.text.configure(cursor="hand2"))
            self.text.tag_bind(tag, "<Leave>", lambda e: self.text.configure(cursor=""))

    # -- playback --
    def refresh(self):
        """Draw the picture for the current time (or the whole run)."""
        r = self.current
        if r is None:
            return
        self.overview = bool(self.overview_var.get())
        img = R.render_frame(r, None if self.overview else self.t, show_guess=bool(self.guess_var.get()), show_camera=bool(self.camera_var.get()),
                             show_route=bool(self.route_var.get()), show_seen=bool(self.seen_var.get()),
                             show_blocked=bool(self.seen_var.get()), show_pending=bool(self.seen_var.get()))
        room = max(300, self.root.winfo_screenheight() - 480)
        if img.height > room:                                   # small screens: shrink the picture to fit
            img = img.resize((int(img.width * room / img.height), room))
        self.shown_image = ImageTk.PhotoImage(img)
        self.picture.configure(image=self.shown_image, text="")
        total = clock(r.seconds)
        self.time_var.set(f"Whole run ({total})" if self.overview else f"{clock(self.t)} / {total}")
        self._seeking = True
        self.timeline.set(self.t)
        self._seeking = False

    def _slider_moved(self, value):
        if self._seeking or self.current is None:
            return
        self.t = float(value)
        self.refresh()

    def seek(self, t):
        if self.current is None:
            return
        self.t = min(max(0.0, float(t)), self.current.seconds)
        self.refresh()

    def jump_to_event(self, index):
        self.seek(self.event_times[index] - 3.0)                # a few seconds before it, so the build-up can be seen
        self.play()

    def effective_speed(self):
        """The playback speed right now: what was asked for, but slowed to ARM_SPEED while the arm picks or releases, so it can be watched."""
        speed = float(self.speed_var.get().rstrip("x") or 1)
        if self.slow_arm_var.get() and self.current is not None and state_at(self.current, self.t).arm is not None:
            return min(speed, ARM_SPEED)
        return speed

    def toggle_play(self):
        if self.playing:
            self.pause()
        else:
            self.play()

    def play(self):
        if self.current is None:
            return
        if self.t >= self.current.seconds - 0.01:
            self.t = 0.0                                        # Play on a finished run starts it over
        self.playing = True
        self.play_button.configure(text="⏸  Pause")
        self._last_tick = time.monotonic()
        if self._after is None:
            self._after = self.root.after(30, self._tick)

    def pause(self):
        self.playing = False
        if self._after is not None:
            try:
                self.root.after_cancel(self._after)
            except tk.TclError:
                pass
            self._after = None
        try:
            self.play_button.configure(text="▶  Play")
        except tk.TclError:
            pass

    def _tick(self):
        self._after = None
        if not self.playing:
            return
        try:
            now = time.monotonic()
            self.t += (now - self._last_tick) * self.effective_speed()
            self._last_tick = now
            if self.t >= self.current.seconds:
                self.t = self.current.seconds
                self.pause()
                self.refresh()
                return
            self.refresh()
            self._after = self.root.after(30, self._tick)
        except tk.TclError:                                     # the window was closed while playing
            self.playing = False

    def _on_destroy(self, event):
        if event.widget is self.root:
            self.playing = False

    def save_picture(self, path=None):
        if self.current is None:
            self.status_var.set("Run something first")
            return
        path = path or filedialog.asksaveasfilename(parent=self.root, defaultextension=".png", initialfile="simulator_run.png",
                                                    filetypes=[("PNG picture", "*.png")])
        if path:
            R.render_frame(self.current, None if self.overview else self.t, show_guess=bool(self.guess_var.get()),
                           show_camera=bool(self.camera_var.get()), show_route=bool(self.route_var.get()),
                           show_seen=bool(self.seen_var.get()), show_blocked=bool(self.seen_var.get()),
                           show_pending=bool(self.seen_var.get())).save(path)
            self.status_var.set(f"Saved {path}")


def main():
    root = tk.Tk()
    SimWindow(root)
    root.lift()                                                  # come to the front even when started from another program
    root.attributes("-topmost", True)
    root.after(800, lambda: root.attributes("-topmost", False))
    root.mainloop()


if __name__ == "__main__":
    main()
