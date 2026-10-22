"""
Emergency stop: one latch shared by every e-stop input (the buttons on the
screens and the USB game controller, see gamepad.py).

    RUNNING ---- any e-stop pressed ----> TRIPPED      nothing moves
    TRIPPED ---- every e-stop released --> RELEASED    still nothing moves
    RELEASED --- Reset (acknowledge) ----> STOPPED     still nothing moves
    STOPPED ---- Resume -----------------> RUNNING

A press from any state goes straight to TRIPPED. Releasing the e-stop never
restarts anything: it takes a deliberate Reset and then a separate Resume.
Reset is refused while any e-stop is still pressed, and Resume is refused
until after a Reset.

Nothing here waits or loops: the program's normal frame loop asks `running`
every frame and simply does not advance the robots while it is False. The
e-stops are buttons on a screen and a game controller in a simulation: this
is not a certified safety system.
"""
RUNNING, TRIPPED, RELEASED, STOPPED = "running", "e-stop", "released", "stopped"

COLOURS = {RUNNING: None, TRIPPED: "red", RELEASED: "red", STOPPED: "blue"}
BANNER_COLOURS = {"red": "#dc2626", "blue": "#2563eb"}


class EStop:
    def __init__(self, log=print):
        self.state = RUNNING
        self.pressed = set()          # e-stops held down right now, by name
        self.tripped_by = None        # the first one pressed, for the screens
        self.log = log

    @property
    def running(self):
        return self.state == RUNNING

    # ---- inputs
    def press(self, source):
        if source not in self.pressed:
            self.pressed.add(source)
            if self.state != TRIPPED:
                self.tripped_by = source
                self.log(f"  E-STOP pressed ({source}): all motion halted")
        self.state = TRIPPED

    def release(self, source):
        """An e-stop let go (a mushroom head twisted out, a controller button up)."""
        if source in self.pressed:
            self.pressed.discard(source)
            self.log(f"  e-stop released ({source})")
            if not self.pressed and self.state == TRIPPED:
                self.state = RELEASED

    def release_all(self, prefix=""):
        for source in sorted(self.pressed):
            if source.startswith(prefix):
                self.release(source)

    def reset(self):
        """Acknowledge the stop. Returns a message if refused, else None."""
        if self.pressed:
            return f"Can't reset: still active ({', '.join(sorted(self.pressed))})"
        if self.state != RELEASED:
            return None if self.state == STOPPED else "Nothing to reset"
        self.state = STOPPED
        self.log("  e-stop reset: safe and stopped, press Resume to carry on")
        return None

    def resume(self):
        """Restart motion. Returns a message if refused, else None."""
        if self.state in (TRIPPED, RELEASED):
            return "Can't resume: reset the e-stop first"
        if self.state == STOPPED:
            self.state = RUNNING
            self.tripped_by = None
            self.log("  resumed")
        return None

    # ---- for the screens
    def colour(self):
        return COLOURS[self.state]

    def text(self):
        return {
            RUNNING: "",
            TRIPPED: f"STOPPED ({self.tripped_by}) - release or clear it, then Reset",
            RELEASED: "Released / clear - still stopped: press Reset to acknowledge",
            STOPPED: "Reset - stopped and safe: press Resume to carry on",
        }[self.state]
