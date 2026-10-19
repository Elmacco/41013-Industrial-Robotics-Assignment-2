"""
A desktop ordering window for the barista cell (tkinter, part of Python).

One button per drink, a live queue with each order's status and a countdown
to when it can be collected, what the robots are doing right now, the status
beacon, and an e-stop. It runs in the same loop as the simulation: the Cell
calls update() once per simulation step, so nothing here needs a thread.

    python -m barista.full_scene            # opens Swift and this window
"""
import time
import tkinter as tk
from tkinter import ttk

REFRESH = 0.25               # seconds between queue redraws
BEACON_COLOURS = {"green": "#22c55e", "amber": "#f59e0b", "red": "#ef4444"}


def mmss(seconds):
    seconds = max(int(round(seconds)), 0)
    return f"{seconds // 60}:{seconds % 60:02d}"


class KioskWindow:
    """Ordering window for a Cell (see full_scene.Cell.snapshot)."""

    def __init__(self, cell, title="Barista robots - order here"):
        self.cell = cell
        self.closed = False
        self._last = 0.0

        root = self.root = tk.Tk()
        root.title(title)
        root.minsize(520, 560)
        root.protocol("WM_DELETE_WINDOW", self.close)
        pad = dict(padx=10, pady=6)

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
        tk.Button(controls, text="Reset e-stop", font=("Segoe UI", 11), height=2,
                  command=cell.reset_estop).pack(side="left", padx=10)
        self.update(force=True)

    def close(self):
        self.closed = True
        self.root.destroy()

    def update(self, force=False):
        """Redraw (a few times a second) and let tkinter handle clicks."""
        if self.closed:
            return
        now = time.monotonic()
        if force or now - self._last >= REFRESH:
            self._last = now
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
        self.status.configure(text=snap["status"])
