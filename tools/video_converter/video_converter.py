"""Video Converter: pick MOV (or other) videos, get smaller MP4s that play anywhere.

Run:      double-click "Video Converter.bat", or:  py -3.12 tools/video_converter/video_converter.py
Install:  py -3.12 -m pip install imageio-ffmpeg

The originals are only read, and an existing file is never replaced.
"""

import queue
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

import converter

VIDEO_TYPES = [("Videos", "*.mov *.mp4 *.m4v *.mkv *.avi *.webm"), ("All files", "*.*")]
SAME_FOLDER = "same folder as the original"
NAME, ORIGINAL, MADE, STATUS = "name", "original", "made", "status"
POLL_MS = 100


def change_text(before, after):
    percent = round(abs(before - after) / before * 100)
    return f"done, {percent}% {'smaller' if after <= before else 'bigger'}"


class ConverterApp:
    def __init__(self, root, ffmpeg):
        self.root = root
        self.ffmpeg = ffmpeg
        self.paths = {}                  # table row -> source file
        self.waiting = set()             # rows that Convert will work on
        self.folder = None               # None = beside each original
        self.events = queue.Queue()      # worker thread -> window
        self.cancel_event = threading.Event()
        self.worker = None
        self.busy = False

        root.title("Video Converter")
        root.minsize(620, 360)
        frame = ttk.Frame(root, padding=10)
        frame.pack(fill="both", expand=True)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        top = ttk.Frame(frame)
        top.grid(row=0, column=0, sticky="w")
        self.add_button = ttk.Button(top, text="Add videos...", command=self.choose_videos)
        self.add_button.pack(side="left")
        self.remove_button = ttk.Button(top, text="Remove", command=self.remove_selected)
        self.remove_button.pack(side="left", padx=(8, 0))

        table = ttk.Frame(frame)
        table.grid(row=1, column=0, sticky="nsew", pady=8)
        self.tree = ttk.Treeview(table, columns=(NAME, ORIGINAL, MADE, STATUS), show="headings", height=7)
        for column, title, width in ((NAME, "Video", 200), (ORIGINAL, "Original", 90), (MADE, "MP4", 90),
                                     (STATUS, "Status", 220)):
            self.tree.heading(column, text=title, anchor="w")
            self.tree.column(column, width=width, stretch=column in (NAME, STATUS))
        scroll = ttk.Scrollbar(table, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=scroll.set)
        self.tree.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.tree.bind("<Delete>", lambda _event: self.remove_selected())

        options = ttk.Frame(frame)
        options.grid(row=2, column=0, sticky="w")
        defaults = converter.Options()
        self.size_var = tk.StringVar(root, defaults.size)
        self.quality_var = tk.StringVar(root, defaults.quality)
        self.location_var = tk.BooleanVar(root, defaults.remove_location)
        ttk.Label(options, text="Size:").pack(side="left")
        size_box = ttk.Combobox(options, textvariable=self.size_var, values=list(converter.SIZES), width=7,
                                state="readonly")
        size_box.pack(side="left", padx=(4, 14))
        ttk.Label(options, text="Quality:").pack(side="left")
        quality_box = ttk.Combobox(options, textvariable=self.quality_var, values=list(converter.QUALITIES), width=10,
                                   state="readonly")
        quality_box.pack(side="left", padx=(4, 14))
        location_box = ttk.Checkbutton(options, text="Remove location and other file details",
                                       variable=self.location_var)
        location_box.pack(side="left")

        save = ttk.Frame(frame)
        save.grid(row=3, column=0, sticky="ew", pady=8)
        ttk.Label(save, text="Save to:").pack(side="left")
        self.folder_var = tk.StringVar(root, SAME_FOLDER)
        choose = ttk.Button(save, text="Choose folder...", command=self.choose_folder)
        choose.pack(side="right")
        same = ttk.Button(save, text="Same as original", command=lambda: self.set_folder(None))
        same.pack(side="right", padx=(0, 8))
        self.wrapping_label(save, self.folder_var).pack(side="left", fill="x", expand=True, padx=4)

        actions = ttk.Frame(frame)
        actions.grid(row=4, column=0, sticky="ew")
        self.convert_button = ttk.Button(actions, text="Convert", command=self.start)
        self.convert_button.pack(side="left")
        self.cancel_button = ttk.Button(actions, text="Cancel", command=self.cancel)
        self.cancel_button.pack(side="left", padx=(8, 12))
        self.bar = ttk.Progressbar(actions, maximum=1.0)
        self.bar.pack(side="left", fill="x", expand=True)
        self.status_var = tk.StringVar(root, "Add the videos you want to convert.")
        self.wrapping_label(frame, self.status_var).grid(row=5, column=0, sticky="ew", pady=(6, 0))

        # what cannot be touched during a conversion, with the state it returns to
        self.locked = [(self.add_button, "normal"), (self.remove_button, "normal"), (size_box, "readonly"),
                       (quality_box, "readonly"), (location_box, "normal"), (choose, "normal"), (same, "normal")]
        self.refresh_buttons()

    @staticmethod
    def wrapping_label(parent, variable):
        """A label for long paths: it takes the room it is given and wraps, instead of widening the window."""
        label = ttk.Label(parent, textvariable=variable, width=1)
        label.bind("<Configure>", lambda event: label.configure(wraplength=event.width))
        return label

    # the list

    def choose_videos(self):
        self.add(filedialog.askopenfilenames(parent=self.root, title="Add videos", filetypes=VIDEO_TYPES))

    def add(self, paths):
        for text in paths:
            path = Path(text).resolve()
            if path in self.paths.values() or not path.is_file():
                continue
            try:
                converter.probe(self.ffmpeg, path)
                status = "waiting"
            except converter.ConvertError as error:
                status = str(error)
            row = self.tree.insert("", "end", values=(path.name, converter.size_text(path.stat().st_size), "", status))
            self.paths[row] = path
            if status == "waiting":
                self.waiting.add(row)
                self.status_var.set("Press Convert to start.")
        self.refresh_buttons()

    def remove_selected(self):
        if self.busy:
            return
        for row in self.tree.selection():
            self.tree.delete(row)
            del self.paths[row]
            self.waiting.discard(row)
        self.refresh_buttons()

    # options

    def choose_folder(self):
        chosen = filedialog.askdirectory(parent=self.root, title="Save the MP4 files to", mustexist=True)
        if chosen:
            self.set_folder(chosen)

    def set_folder(self, folder):
        self.folder = Path(folder) if folder else None
        self.folder_var.set(str(self.folder) if self.folder else SAME_FOLDER)

    # converting

    def refresh_buttons(self):
        for widget, usual in self.locked:
            widget.configure(state="disabled" if self.busy else usual)
        self.convert_button.configure(state="normal" if self.waiting and not self.busy else "disabled")
        self.cancel_button.configure(state="normal" if self.busy else "disabled")

    def start(self):
        jobs = [(row, self.paths[row]) for row in self.tree.get_children() if row in self.waiting]
        if self.busy or not jobs:
            return
        options = converter.Options(self.size_var.get(), self.quality_var.get(), self.location_var.get())
        self.busy = True
        self.results, self.failures = [], 0           # of this run
        self.cancel_event.clear()
        self.bar["value"] = 0
        self.status_var.set("Starting...")
        self.refresh_buttons()
        self.worker = threading.Thread(target=self.work, args=(jobs, options, self.folder), daemon=True)
        self.worker.start()
        self.root.after(POLL_MS, self.poll)

    def work(self, jobs, options, folder):
        """Runs on the worker thread: it must not touch the window, only post events."""
        for number, (row, source) in enumerate(jobs, 1):
            if self.cancel_event.is_set():
                break
            self.events.put(("started", row, (number, len(jobs), time.monotonic())))
            try:
                made = converter.convert(
                    self.ffmpeg, source, options, folder, cancel=self.cancel_event,
                    on_progress=lambda done, row=row: self.events.put(("progress", row, done)))
                self.events.put(("done", row, (source.stat().st_size, made)))
            except converter.Cancelled:
                self.events.put(("cancelled", row, None))
                break
            except (converter.ConvertError, OSError) as error:
                self.events.put(("failed", row, str(error)))
        self.events.put(("finished", None, None))

    def poll(self):
        while True:
            try:
                kind, row, data = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "started":
                self.current = (*data, self.paths[row].name)
                self.bar["value"] = 0
                self.tree.set(row, STATUS, "converting")
                self.tree.see(row)
                self.show_progress(row, 0.0)
            elif kind == "progress":
                self.show_progress(row, data)
            elif kind == "done":
                before, made = data
                after = made.stat().st_size
                self.tree.set(row, MADE, converter.size_text(after))
                self.tree.set(row, STATUS, change_text(before, after))
                self.waiting.discard(row)
                self.results.append(made)
            elif kind == "cancelled":
                self.tree.set(row, STATUS, "cancelled")
            elif kind == "failed":
                self.tree.set(row, STATUS, f"failed: {data}")
                self.waiting.discard(row)
                self.failures += 1
            elif kind == "finished":
                self.finish()
                return
        self.root.after(POLL_MS, self.poll)

    def show_progress(self, row, done):
        number, total, began, name = self.current
        self.bar["value"] = done
        self.tree.set(row, STATUS, f"converting, {done:.0%}")
        if not self.cancel_event.is_set():
            left = converter.time_left_text(done, time.monotonic() - began)
            self.status_var.set(f"Video {number} of {total}: {name}, {done:.0%}" + (f", {left}" if left else ""))

    def finish(self):
        self.busy = False
        if self.cancel_event.is_set():
            self.bar["value"] = 0
            self.status_var.set("Cancelled.")
        else:
            parts = []
            if self.results:
                count = len(self.results)
                folders = {path.parent for path in self.results}
                where = str(folders.pop()) if len(folders) == 1 else "the same folders as the originals"
                parts.append(f"{count} video{'s' if count > 1 else ''} converted. Saved in {where}")
            if self.failures:
                parts.append(f"{self.failures} failed (remove and add again to retry)")
            self.status_var.set(". ".join(parts))
        self.refresh_buttons()

    def cancel(self):
        if self.busy:
            self.cancel_event.set()
            self.status_var.set("Cancelling...")

    def stop_and_wait(self):
        """Stops any conversion and waits until its half-written file is gone."""
        self.cancel_event.set()
        if self.worker:
            self.worker.join(timeout=15)

    def close(self):
        if self.busy and not messagebox.askyesno("Video Converter", "A video is still converting. Stop it and close?",
                                                 parent=self.root):
            return
        self.stop_and_wait()
        self.root.destroy()


def main():
    if sys.platform == "win32":
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)      # sharp text on a scaled screen
    root = tk.Tk()
    try:
        ffmpeg = converter.find_ffmpeg()
    except converter.ConvertError as error:
        root.withdraw()
        messagebox.showerror("Video Converter", str(error))
        return
    app = ConverterApp(root, ffmpeg)
    root.protocol("WM_DELETE_WINDOW", app.close)
    app.add(sys.argv[1:])                                   # videos dropped onto the .bat file
    root.mainloop()


if __name__ == "__main__":
    main()
