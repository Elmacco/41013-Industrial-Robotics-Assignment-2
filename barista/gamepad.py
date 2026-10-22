"""
A USB game controller as the physical e-stop input (pygame, already installed).

    python -m barista.gamepad        # test: prints every button as it is pressed

On an Xbox controller:

    B (red)   E-STOP     held = pressed; letting go releases it, which on its
                         own restarts nothing
    Y         Reset      acknowledge the stop once the e-stop is released
    A         Resume     restart motion after a Reset

Unplugging the controller counts as pressing the e-stop (losing the e-stop
must stop the robots, not leave them unguarded); plugging it back in releases
that, and it still takes Reset then Resume.

poll() reads the controller once and returns at once, so it is called from the
program's normal frame loop: nothing waits on the controller.
"""
import os

# Deliver controller input even when the pygame "window" is not focused (there
# is no window: Swift and tkinter have the focus).
os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame  # noqa: E402

# Button numbers as SDL reports an Xbox controller. Run this file to check yours.
ESTOP_BUTTON, RESET_BUTTON, RESUME_BUTTON = 1, 3, 0          # B, Y, A
SOURCE = "controller e-stop (B)"
LOST = "controller unplugged"


class Gamepad:
    """Feeds an EStop from a game controller. on_message(text) shows refusals."""

    def __init__(self, estop, on_reset=None, on_resume=None, log=print):
        self.estop = estop
        self.on_reset = on_reset or estop.reset
        self.on_resume = on_resume or estop.resume
        self.log = log
        self.joy = None
        self.seen = False                 # has a controller ever been connected
        self.down = {}
        pygame.display.init()             # pygame's event queue lives in its display module
        pygame.joystick.init()
        if pygame.joystick.get_count():
            self._connect(0)
        else:
            log("  no game controller found: plug one in any time to use it as the e-stop")

    def _connect(self, index):
        self.joy = pygame.joystick.Joystick(index)
        self.seen = True
        self.down = {b: False for b in (ESTOP_BUTTON, RESET_BUTTON, RESUME_BUTTON)}
        self.log(f"  e-stop controller: {self.joy.get_name()}  (B e-stop, Y reset, A resume)")
        self.estop.release(LOST)

    def poll(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED and self.joy is None:
                self._connect(event.device_index)
            elif event.type == pygame.JOYDEVICEREMOVED and self.joy is not None \
                    and event.instance_id == self.joy.get_instance_id():
                self.joy = None
                self.estop.release(SOURCE)
                self.estop.press(LOST)
        if self.joy is None:
            return
        for button in self.down:
            now = bool(self.joy.get_button(button))
            if now == self.down[button]:
                continue
            self.down[button] = now
            if button == ESTOP_BUTTON:
                (self.estop.press if now else self.estop.release)(SOURCE)
            elif now and button == RESET_BUTTON:
                self._say(self.on_reset())
            elif now and button == RESUME_BUTTON:
                self._say(self.on_resume())

    def _say(self, refused):
        if refused:
            self.log(f"  {refused}")


if __name__ == "__main__":
    # Same code path as the cell: an EStop fed by Gamepad.poll(), plus the raw
    # button numbers, so a different controller's mapping can be checked.
    from barista.estop import EStop

    estop = EStop()
    pad = Gamepad(estop)
    print("Press buttons on the controller (Ctrl+C to stop).")
    clock, held, state = pygame.time.Clock(), set(), None
    try:
        while True:
            pad.poll()
            if pad.joy is not None:
                now = {b for b in range(pad.joy.get_numbuttons()) if pad.joy.get_button(b)}
                for b in sorted(now - held):
                    print(f"button {b} down")
                held = now
            if estop.state != state:
                state = estop.state
                print(f"    e-stop state: {state.upper()}")
            clock.tick(30)                # wait for the next frame, not a busy loop
    except KeyboardInterrupt:
        pass
