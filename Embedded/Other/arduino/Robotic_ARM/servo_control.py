import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import math
import time
import threading

class ServoControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm 4-Axis Controller")
        self.root.geometry("760x900")
        self.root.configure(bg="#f4f5f7")

        self.arduino = None
        self.recorded_poses = []
        self.is_playing = False
        self.is_moving = False
        self._last_send_time = [0.0] * 4  # Rate limit per servo
        self.move_id = 0

        # Servo definitions: pin, name, default angle, angle limits.
        # Keep the pins and limits in sync with servo.ino. Servos get no signal until first commanded.
        self.servo_config = [
            {"pin": 6,  "name": "Gripper",  "angle": 90,  "min": 0, "max": 120},
            {"pin": 9,  "name": "Elbow",    "angle": 120, "min": 80, "max": 180},   # the elbow also has a limit that depends on the
            {"pin": 10, "name": "Shoulder", "angle": 90,  "min": 40, "max": 170},   # shoulder angle: the Uno enforces it (servo.ino)
            {"pin": 11, "name": "Servo 4",  "angle": 90,  "min": 0, "max": 180},
        ]
        self.SPEED_DEFAULT = 60
        self.SPEED_MAX = 600
        self.commanded = [False] * 4                     # has this servo been sent a command yet?
        self.est_angle = [float(c["angle"]) for c in self.servo_config]  # where each servo is believed to be
        self.speed_vars = []
        self.speed_pos_vars = []                         # slider positions (log scale), see speed_to_pos
        self.target = [float(c["angle"]) for c in self.servo_config]   # last angle sent to each servo
        self.speed_labels = []

        self.sliders = []
        self.held_markers = []
        self.entries = []
        self.angle_vars = []

        self.setup_ui()
        self.refresh_ports()

    def setup_ui(self):
        style = ttk.Style()
        style.theme_use("clam")

        # --- Connection Panel ---
        conn_frame = ttk.LabelFrame(self.root, text=" Serial Connection ", padding=10)
        conn_frame.pack(fill="x", padx=15, pady=6)

        ttk.Label(conn_frame, text="Port:").pack(side="left", padx=(0, 5))
        self.port_combobox = ttk.Combobox(conn_frame, width=15, state="readonly")
        self.port_combobox.pack(side="left", padx=5)

        ttk.Button(conn_frame, text="🔄 Refresh", command=self.refresh_ports).pack(side="left", padx=5)
        self.btn_connect = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.pack(side="left", padx=5)

        self.lbl_status = ttk.Label(conn_frame, text="Disconnected", foreground="red")
        self.lbl_status.pack(side="left", padx=10)

        # --- Servo Controls Panel ---
        controls_frame = ttk.LabelFrame(self.root, text=" Servo Controls ", padding=10)
        controls_frame.pack(fill="x", padx=15, pady=6)

        for i, cfg in enumerate(self.servo_config):
            row_frame = ttk.Frame(controls_frame, padding=4)
            row_frame.pack(fill="x", pady=2)

            # Label
            lbl = ttk.Label(row_frame, text=f"[{i}] {cfg['name']} (Pin {cfg['pin']}):", width=22, font=("Helvetica", 9, "bold"))
            lbl.grid(row=0, column=0, sticky="w")

            # Var & Entry
            var = tk.IntVar(value=cfg["angle"])
            self.angle_vars.append(var)

            # -5 Deg Button
            btn_minus = ttk.Button(row_frame, text="-5°", width=4, command=lambda idx=i: self.adjust_angle(idx, -5))
            btn_minus.grid(row=0, column=1, padx=2)

            # Slider
            slider = ttk.Scale(row_frame, from_=cfg["min"], to=cfg["max"], orient="horizontal", variable=var,
                               command=lambda val, idx=i: self.on_slider_move(idx, val))
            slider.grid(row=0, column=2, sticky="ew", padx=5)
            row_frame.columnconfigure(2, weight=1)
            self.sliders.append(slider)
            # red marker (only the elbow's is ever shown): the angle the Uno's elbow limit is holding the elbow at, while it acts
            self.held_markers.append(tk.Frame(slider, bg="#d1242f", width=6))

            # +5 Deg Button
            btn_plus = ttk.Button(row_frame, text="+5°", width=4, command=lambda idx=i: self.adjust_angle(idx, 5))
            btn_plus.grid(row=0, column=3, padx=2)

            # Entry box
            entry = ttk.Entry(row_frame, textvariable=var, width=5)
            entry.grid(row=0, column=4, padx=5)
            entry.bind("<Return>", lambda event, idx=i: self.on_entry_submit(idx))
            self.entries.append(entry)

            # Speed row (degrees per second, handled by the Arduino)
            speed_var = tk.IntVar(value=self.SPEED_DEFAULT)
            self.speed_vars.append(speed_var)
            ttk.Label(row_frame, text="speed", foreground="#666").grid(row=1, column=0, sticky="e")
            pos_var = tk.DoubleVar(value=self.speed_to_pos(self.SPEED_DEFAULT))
            self.speed_pos_vars.append(pos_var)
            sp_scale = ttk.Scale(row_frame, from_=1, to=self.SPEED_MAX, orient="horizontal", variable=pos_var,
                                 command=lambda val, idx=i: self.on_servo_speed(idx, val))
            sp_scale.grid(row=1, column=2, sticky="ew", padx=5)
            sp_label = ttk.Label(row_frame, text=f"{self.SPEED_DEFAULT}°/s", width=8, foreground="#666")
            sp_label.grid(row=1, column=4)
            self.speed_labels.append(sp_label)

        # a limit message from the Uno (empty = nothing to report): shown in red while it lasts
        self.limit_alert = tk.StringVar(value="")
        self.limit_alert_label = ttk.Label(self.root, textvariable=self.limit_alert, foreground="#d1242f", font=("Helvetica", 10, "bold"),
                                           wraplength=700, justify="left")

        # --- Speed presets (apply to all servos; each servo's slider can still be tuned) ---
        speed_frame = self.speed_frame = ttk.LabelFrame(self.root, text=" Motion Speed (all servos) ", padding=10)
        speed_frame.pack(fill="x", padx=15, pady=6)
        for text, spd in (("Slow 30°/s", 30), ("Default 60°/s", 60), ("Fast 180°/s", 180), ("Max 600°/s", 600)):
            ttk.Button(speed_frame, text=text, command=lambda v=spd: self.set_all_speeds(v)).pack(side="left", expand=True, fill="x", padx=3)

        # --- Global Presets Panel ---
        preset_frame = ttk.LabelFrame(self.root, text=" Quick Presets ", padding=10)
        preset_frame.pack(fill="x", padx=15, pady=6)

        ttk.Button(preset_frame, text="All default", command=lambda: self.move_to_pose_thread(self.default_pose())).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Button(preset_frame, text="Toggle Gripper", command=self.toggle_claw_smooth).pack(side="left", expand=True, fill="x", padx=3)
        ttk.Label(self.root, text="Servos stay idle after power-up until you first move a slider or press All default.",
                  foreground="#666").pack(padx=15, anchor="w")

        # --- Pose Recorder & Sequence Panel ---
        seq_frame = ttk.LabelFrame(self.root, text=" Pose Sequence Recorder ", padding=10)
        seq_frame.pack(fill="both", expand=True, padx=15, pady=6)

        btn_box = ttk.Frame(seq_frame)
        btn_box.pack(fill="x", pady=2)

        ttk.Button(btn_box, text="📸 Record Current Pose", command=self.record_pose).pack(side="left", padx=3)
        ttk.Button(btn_box, text="▶ Play Sequence", command=self.play_sequence_thread).pack(side="left", padx=3)
        ttk.Button(btn_box, text="⏹ Stop Motion", command=self.stop_motion).pack(side="left", padx=3)
        ttk.Button(btn_box, text="🗑 Clear Poses", command=self.clear_poses).pack(side="left", padx=3)

        self.pose_listbox = tk.Listbox(seq_frame, height=6, font=("Consolas", 9))
        self.pose_listbox.pack(fill="both", expand=True, pady=5)

    # --- Serial Communication Functions ---
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox['values'] = ports
        if ports:
            self.port_combobox.current(0)

    def toggle_connection(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            self.arduino = None
            self.btn_connect.config(text="Connect")
            self.lbl_status.config(text="Disconnected", foreground="red")
            self.show_limit(-1, "")
        else:
            port = self.port_combobox.get()
            if not port:
                messagebox.showerror("Error", "No COM port selected!")
                return
            try:
                self.arduino = serial.Serial(port, 9600, timeout=1)
                time.sleep(2)
                self.btn_connect.config(text="Disconnect")
                threading.Thread(target=self.read_replies, args=(self.arduino,), daemon=True).start()
                self.lbl_status.config(text=f"Connected ({port})", foreground="green")
            except Exception as e:
                messagebox.showerror(
                    "Connection Error",
                    f"Could not open {port}:\n{e}\n\nTip: If Arduino IDE's Serial Monitor is open, CLOSE IT so Python can access the port."
                )

    def read_replies(self, port):
        """Show the Arduino's 'OK <servo> <target> <speed>' echo, so you can see the speed it is really using."""
        while port.is_open and self.arduino is port:
            try:
                line = port.readline().decode("utf-8", "ignore").strip()
            except Exception:
                return
            if line.startswith("OK"):
                self.root.after(0, lambda s=line: self.lbl_status.config(text=f"Arduino: {s}", foreground="green"))
            elif line.startswith("LIM"):
                parts = line.split(" ", 2)      # "LIM <held angle or -1> <message>"
                try:
                    held = int(parts[1])
                except (IndexError, ValueError):
                    continue
                self.root.after(0, lambda h=held, m=(parts[2] if len(parts) > 2 else ""): self.show_limit(h, m))

    def show_limit(self, held, message):
        """Red message and a red, non-draggable marker on the elbow slider at the angle the limit holds the elbow at."""
        if message:
            self.limit_alert.set("⚠ " + message)
            if not self.limit_alert_label.winfo_ismapped():
                self.limit_alert_label.pack(fill="x", padx=15, before=self.speed_frame)
        else:
            self.limit_alert.set("")
            self.limit_alert_label.pack_forget()
        marker, slider = self.held_markers[1], self.sliders[1]
        if held < 0:
            marker.place_forget()
        else:
            lo, hi = float(slider.cget("from")), float(slider.cget("to"))
            x, _ = slider.coords(min(max(held, lo), hi))
            marker.place(x=x - 3, y=0, width=6, relheight=1.0)

    def clamp(self, idx, angle):
        cfg = self.servo_config[idx]
        return max(cfg["min"], min(cfg["max"], int(round(angle))))

    def send_command(self, idx, angle):
        """Send '<servo> <angle> <speed>'; the Arduino moves the servo there at that speed."""
        if self.arduino and self.arduino.is_open:
            angle = self.clamp(idx, angle)
            speed = max(1, min(self.SPEED_MAX, int(self.speed_vars[idx].get())))
            try:
                self.arduino.write(f"{idx} {angle} {speed}\n".encode('utf-8'))
                self.target[idx] = float(angle)
                self.arduino.flush()
                if not self.commanded[idx]:
                    self.commanded[idx] = True
                    self.est_angle[idx] = float(angle)   # first command: the servo starts right there
            except Exception as e:
                print(f"Write error: {e}")
                self.lbl_status.config(text=f"Write Error: {e}", foreground="red")
        else:
            self.lbl_status.config(text="Not Connected! Select port & click Connect", foreground="orange")

    # --- Speed Handling ---
    def speed_to_pos(self, speed):
        return float(speed)                               # the speed slider is linear: position = deg/s

    def pos_to_speed(self, pos):
        return max(1, min(self.SPEED_MAX, int(round(float(pos)))))

    def on_servo_speed(self, idx, val):
        speed = self.pos_to_speed(val)
        if speed != self.speed_vars[idx].get():
            self.speed_vars[idx].set(speed)
            self.speed_labels[idx].config(text=f"{speed}°/s")
            self.apply_speed(idx)

    def apply_speed(self, idx):
        """Make the Arduino use the new speed now (it only learns the speed with an angle command), without changing the
        target. Skipped while a recorded move is running: that move re-sends the speed with its own commands."""
        if self.commanded[idx] and not (self.is_moving or self.is_playing):
            self.send_command(idx, int(round(self.target[idx])))

    def set_all_speeds(self, value):
        for idx, (var, label, pos) in enumerate(zip(self.speed_vars, self.speed_labels, self.speed_pos_vars)):
            var.set(value)
            pos.set(self.speed_to_pos(value))
            label.config(text=f"{value}°/s")
            self.apply_speed(idx)

    def default_pose(self):
        return [c["angle"] for c in self.servo_config]

    def stop_motion(self):
        self.is_moving = False
        self.is_playing = False
        self.move_id += 1
        # hold each servo where it has got to
        for i in range(len(self.servo_config)):
            if self.commanded[i]:
                self.send_command(i, int(round(self.est_angle[i])))

    def move_to_pose(self, target_pose):
        """Send every servo to its target (each at its own speed) and follow the estimated positions until done."""
        self.is_moving = True
        self.move_id += 1
        my_id = self.move_id
        target_pose = [self.clamp(i, a) for i, a in enumerate(target_pose)]
        start = list(self.est_angle)
        was_commanded = list(self.commanded)
        for idx, angle in enumerate(target_pose):
            self.send_command(idx, angle)
            time.sleep(0.01)
        # a servo that was idle snaps to its target at once; the others travel at their set speed
        total = 0.0
        for i, target in enumerate(target_pose):
            spd = max(1, int(self.speed_vars[i].get()))
            if was_commanded[i]:
                total = max(total, abs(target - start[i]) / spd)
        t0 = time.time()
        while self.is_moving and my_id == self.move_id:
            elapsed = time.time() - t0
            for i, target in enumerate(target_pose):
                spd = max(1, int(self.speed_vars[i].get()))
                if not was_commanded[i]:
                    pos = float(target)
                else:
                    moved = min(abs(target - start[i]), spd * elapsed)
                    pos = start[i] + (moved if target >= start[i] else -moved)
                self.est_angle[i] = pos
                self.angle_vars[i].set(int(round(pos)))
            if elapsed >= total:
                break
            time.sleep(0.03)
        if my_id == self.move_id:
            self.is_moving = False

    def move_to_pose_thread(self, target_pose):
        if self.is_moving or self.is_playing:
            self.is_moving = False
            time.sleep(0.05)
        threading.Thread(target=self.move_to_pose, args=(target_pose,), daemon=True).start()

    # --- Motion Handling ---
    def on_slider_move(self, idx, val):
        angle = self.clamp(idx, float(val))
        self.angle_vars[idx].set(angle)
        self.est_angle[idx] = float(angle)
        now = time.time()
        # Rate limit slider events to max 30Hz (every 30ms) to avoid buffer overflow
        if (now - self._last_send_time[idx]) >= 0.03:
            self._last_send_time[idx] = now
            self.send_command(idx, angle)

    def on_entry_submit(self, idx):
        angle = self.clamp(idx, self.angle_vars[idx].get())
        self.angle_vars[idx].set(angle)
        self.send_command(idx, angle)
        self.est_angle[idx] = float(angle)

    def adjust_angle(self, idx, delta):
        new_angle = self.clamp(idx, self.angle_vars[idx].get() + delta)
        self.angle_vars[idx].set(new_angle)
        self.send_command(idx, new_angle)
        self.est_angle[idx] = float(new_angle)

    def toggle_claw_smooth(self):
        claw_idx = 0  # Index 0 is the Gripper
        curr = self.angle_vars[claw_idx].get()
        target = self.servo_config[claw_idx]["min"] if curr > 45 else 90
        target_pose = [var.get() for var in self.angle_vars]
        target_pose[claw_idx] = target
        self.move_to_pose_thread(target_pose)

    # --- Sequence Recording ---
    def record_pose(self):
        pose = [var.get() for var in self.angle_vars]
        self.recorded_poses.append(pose)
        pose_str = (f"Pose {len(self.recorded_poses)}: Gripper={pose[0]}°, Elbow={pose[1]}°, "
                    f"Shoulder={pose[2]}°, Servo 4={pose[3]}°")
        self.pose_listbox.insert(tk.END, pose_str)

    def clear_poses(self):
        self.recorded_poses.clear()
        self.pose_listbox.delete(0, tk.END)

    def play_sequence_thread(self):
        if not self.recorded_poses:
            messagebox.showinfo("Info", "No poses recorded!")
            return
        if self.is_playing:
            return
        threading.Thread(target=self.play_sequence, daemon=True).start()

    def play_sequence(self):
        self.is_playing = True
        for pose in self.recorded_poses:
            if not self.is_playing:
                break
            self.move_to_pose(pose)
            # Dwell at each keyframe
            start_dwell = time.time()
            while (time.time() - start_dwell) < 0.6 and self.is_playing:
                time.sleep(0.05)
        self.is_playing = False

if __name__ == "__main__":
    root = tk.Tk()
    app = ServoControllerApp(root)
    root.mainloop()
