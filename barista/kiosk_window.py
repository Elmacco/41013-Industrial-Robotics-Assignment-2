"""
A desktop ordering window for the barista cell (tkinter, part of Python).

One button per drink, a live queue with each order's status and a countdown
to when it can be collected, what the robots are doing right now, the status
beacon, a banner showing the e-stop state, and E-STOP, Release, Reset and
Resume buttons (see estop.py). It runs in the same loop as the simulation: the Cell
calls update() once per simulation step, so nothing here needs a thread.

    python -m barista.full_scene            # opens Swift and this window
"""
import time
import tkinter as tk
from tkinter import ttk

REFRESH = 0.25               # seconds between queue redraws
BEACON_COLOURS = {"green": "#22c55e", "amber": "#f59e0b", "red": "#ef4444", "blue": "#3b82f6"}
# E-stop state -> (banner text, background); nothing shown while running.
BANNERS = {
    "e-stop": ("STOPPED - ALL ROBOTS HALTED", "#dc2626"),
    "released": ("STOPPED - clear now, press Reset", "#dc2626"),
    "stopped": ("STOPPED - reset done, press Resume", "#2563eb"),
}


def mmss(seconds):
    seconds = max(int(round(seconds)), 0)
    return f"{seconds // 60}:{seconds % 60:02d}"


class KioskWindow:
    """Ordering window for a Cell (see full_scene.Cell.snapshot)."""

    def __init__(self, cell, title="Barista robots - order here"):
        self.cell = cell
        self.closed = False
        self._last = 0.0
        self._estop_shown = None

        root = self.root = tk.Tk()
        root.title(title)
        root.minsize(640, 600)
        root.protocol("WM_DELETE_WINDOW", self.close)
        pad = dict(padx=10, pady=6)

        self.banner = tk.Label(root, font=("Segoe UI", 16, "bold"), fg="white")
        tk.Label(root, text="What would you like?", font=("Segoe UI", 16, "bold")).pack(anchor="w", **pad)
        menu = tk.Frame(root)
        menu.pack(fill="x", **pad)
        self.buttons = {}
        for i, drink in enumerate(cell.menu()):
            b = tk.Button(menu, font=("Segoe UI", 12), height=2, width=18,
                          command=lambda d=drink: cell.order(d))
            b.grid(row=i // 2, column=i % 2, sticky="ew", padx=4, pady=4)
            self.buttons[drink] = b
        menu.columnconfigure(0, weight=1)
        menu.columnconfigure(1, weight=1)

        tk.Label(root, text="Orders", font=("Segoe UI", 13, "bold")).pack(anchor="w", **pad)
        cols = ("order", "drink", "status", "ready")
        self.table = ttk.Treeview(root, columns=cols, show="headings", height=8)
        for col, text, width in zip(cols, ("#", "Drink", "Status", "Ready in"), (40, 120, 230, 80)):
            self.table.heading(col, text=text)
            self.table.column(col, width=width, anchor="w")
        self.table.pack(fill="both", expand=True, **pad)

        status = tk.Frame(root)
        status.pack(fill="x", **pad)
        self.beacon = tk.Canvas(status, width=26, height=26, highlightthickness=0)
        self.lamp = self.beacon.create_oval(3, 3, 23, 23, fill=BEACON_COLOURS["green"], outline="#333")
        self.beacon.pack(side="left")
        self.status = tk.Label(status, text="", anchor="w", justify="left", wraplength=460)
        self.status.pack(side="left", fill="x", expand=True, padx=8)

        controls = tk.Frame(root)
        controls.pack(fill="x", **pad)
        tk.Button(controls, text="E-STOP", font=("Segoe UI", 14, "bold"), bg="#dc2626", fg="white",
                  activebackground="#b91c1c", width=12, height=2,
                  command=lambda: cell.press_estop("the order window")).pack(side="left")
        for text, command in (("Release e-stop", cell.release_estop), ("Reset", cell.reset_estop),
                              ("Resume", cell.resume)):
            tk.Button(controls, text=text, font=("Segoe UI", 11), height=2, width=12,
                      command=command).pack(side="left", padx=(10, 0))
        # Stand-ins for what a light curtain or camera would report (full_scene.DROPPED).
        things = tk.Frame(root)
        things.pack(fill="x", **pad)
        tk.Label(things, text="Leave in the cell:", font=("Segoe UI", 10)).pack(side="left")
        for name in cell.dropped_names():
            tk.Button(things, text=name, command=lambda n=name: cell.place(n)).pack(side="left", padx=(6, 0))
        tk.Button(things, text="Clear the cell", command=cell.clear_dropped).pack(side="left", padx=(6, 0))
        staff = tk.Frame(root)
        staff.pack(fill="x", **pad)
        tk.Label(staff, text="Staff member:", font=("Segoe UI", 10)).pack(side="left")
        for where, text in cell.staff_moves():
            tk.Button(staff, text=text, command=lambda w=where: cell.safety.send(w)).pack(side="left", padx=(6, 0))
        self.update(force=True)

    def close(self):
        self.closed = True
        self.root.destroy()

    def update(self, force=False):
        """Redraw (a few times a second) and let tkinter handle clicks."""
        if self.closed:
            return
        now = time.monotonic()
        if (self.cell.estop.state, self.cell.safety.speed) != self._estop_shown:
            force = True                  # show an e-stop change on this frame, not up to REFRESH later
        if force or now - self._last >= REFRESH:
            self._last = now
            self._estop_shown = (self.cell.estop.state, self.cell.safety.speed)
            self._redraw(self.cell.snapshot())
        try:
            self.root.update()
        except tk.TclError:              # window closed mid-update
            self.closed = True

    def _redraw(self, snap):
        for drink, b in self.buttons.items():
            b.configure(text=f"{drink.replace('_', ' ').title()}\nabout {mmss(snap['typical'][drink])}")
        self.table.delete(*self.table.get_children())
        for row in snap["orders"]:
            eta = mmss(row["eta"]) if row["eta"] is not None else ""
            approx = "~" if row["estimate"] and eta else ""
            self.table.insert("", "end", values=(row["id"], row["drink"].replace("_", " "),
                                                 row["status"], approx + eta))
        self.beacon.itemconfigure(self.lamp, fill=BEACON_COLOURS[snap["beacon"]])
        if snap["estop"] in BANNERS:
            text, bg = BANNERS[snap["estop"]]
            self.banner.configure(text=text, bg=bg)
            self.banner.pack(fill="x", before=self.root.winfo_children()[1], ipady=8)
        elif snap["slow"]:
            self.banner.configure(text="REDUCED SPEED - someone near the maintenance door", bg="#ca8a04")
            self.banner.pack(fill="x", before=self.root.winfo_children()[1], ipady=8)
        else:
            self.banner.pack_forget()
        self.status.configure(text=snap["status"])
