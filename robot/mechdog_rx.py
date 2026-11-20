# MechDog main ESP32 (MicroPython) - USB command receiver, v RX2.
# Run from RAM via tools\rx_test.py / the PC line follower, or later install as /main.py.
#
# Protocol (1 byte in, one text line out):
#   P -> 'PONG RX2'            I -> init MechDog (reuses an existing global `dog`)
#   A -> 'ARMED' (needs init)  D/S -> stop + 'DISARMED'
#   F/L/R -> heartbeat + desired motion (only while armed)
#   0..8  -> heartbeat + proportional steering: raw turn level = digit-4 (-4..+4),
#            move(STEP_FAST - |lvl|*STEP_DROP, lvl*TURN_UNIT). Sign is decided on the PC.
#   Q -> status line           X -> exit to REPL (only while disarmed)
#   G / E / B / O -> head lamp (ultrasonic RGB) green / red / blue / off -> 'LED x' (no motors, no init needed)
#   Ctrl-C -> KeyboardInterrupt -> motors stopped, back to REPL
#
# Safety (software only - the power switch is the real emergency stop):
#   - no motion until I and A; re-sending A while armed does NOT extend MAX_RUN_MS
#   - WATCHDOG_MS without F/L/R -> stop;  MAX_RUN_MS after A -> stop
#   - move() only on change and at most every MIN_MOVE_GAP_MS (stops are never delayed)
#   - every move() call is timed; slow calls are reported (SLOW_MOVE)
import sys
import time
import gc
try:
    import uselect as select
except ImportError:
    import select

DRY_RUN = False          # True: never touches motors, prints DRY_MOVE instead
STEP_MM = 30
TURN_DEG = 15
WATCHDOG_MS = 500
MAX_RUN_MS = 6000
MIN_MOVE_GAP_MS = 150
SLOW_MOVE_MS = 60

STEP_FAST = 60           # proportional mode: straight stride
STEP_DROP = 6            # stride reduction per turn level
TURN_UNIT = 10           # turn per level (Hiwonder app uses up to +-40)

MOTION = {'F': (STEP_MM, 0), 'L': (STEP_MM, TURN_DEG), 'R': (STEP_MM, -TURN_DEG)}
for _d in range(9):
    _lvl = _d - 4
    MOTION[str(_d)] = (STEP_FAST - abs(_lvl) * STEP_DROP, _lvl * TURN_UNIT)

_g = globals()
dog = _g.get('dog')      # reuse an instance created earlier in this boot (never make two)
sonar = _g.get('sonar')
LED_RGB = {'G': (0, 255, 0), 'E': (255, 0, 0), 'B': (0, 0, 255), 'O': (0, 0, 0)}


def set_led(c):
    global sonar
    try:
        if sonar is None:
            import Hiwonder_IIC
            sonar = Hiwonder_IIC.I2CSonar(Hiwonder_IIC.IIC(1))
            _g['sonar'] = sonar
        r, g, b = LED_RGB[c]
        sonar.setRGB(0, r, g, b)
        log('LED', c)
    except Exception as e:
        log('LED_ERR', repr(e))


def log(*a):
    print(*a)


def init_dog():
    global dog
    if dog is not None or DRY_RUN:
        log('INIT_OK', 'dry' if DRY_RUN else 'reused')
        return True
    try:
        t = time.ticks_ms()
        from HW_MechDog import MechDog
        dog = MechDog()
        _g['dog'] = dog
        gc.collect()
        log('INIT_OK', time.ticks_diff(time.ticks_ms(), t), 'ms', gc.mem_free())
        return True
    except Exception as e:
        log('INIT_ERR', repr(e))
        return False


def drive(step, turn):
    """Returns True on success. Never raises."""
    t = time.ticks_ms()
    try:
        if DRY_RUN:
            log('DRY_MOVE', step, turn)
        elif dog is not None:
            dog.move(step, turn)
        else:
            return False
    except Exception as e:
        log('MOVE_ERR', repr(e))
        return False
    dt = time.ticks_diff(time.ticks_ms(), t)
    if dt > SLOW_MOVE_MS:
        log('SLOW_MOVE', dt)
    return True


class Rx:
    def __init__(self):
        self.armed = False
        self.current = 'S'       # what the motors were last told
        self.desired = 'S'
        self.last_rx = 0
        self.arm_at = 0
        self.last_move = time.ticks_add(time.ticks_ms(), -1000)

    def stop(self, reason):
        was = self.armed or self.current != 'S'
        self.armed = False
        self.desired = 'S'
        if self.current != 'S' or reason == 'BOOT':
            drive(0, 0)
            self.last_move = time.ticks_ms()
        self.current = 'S'
        if was or reason in ('CMD', 'BOOT'):
            log('STOP', reason)

    def handle(self, c, now):
        if c == 'P':
            log('PONG RX2')
        elif c == 'I':
            init_dog()
        elif c == 'A':
            if self.armed:
                log('ARMED')                      # already armed: run limit NOT extended
            elif dog is None and not DRY_RUN:
                log('ARM_REFUSED no_init')
            else:
                self.stop('ARM')
                self.armed = True
                self.last_rx = now
                self.arm_at = now
                log('ARMED')
        elif c in ('D', 'S'):
            self.stop('CMD')
            log('DISARMED')
        elif c in MOTION:
            if self.armed:
                self.last_rx = now
                self.desired = c
        elif c in LED_RGB:
            set_led(c)
        elif c == 'Q':
            gc.collect()
            log('STATUS armed=%d cur=%s dry=%d init=%d gc=%d' % (
                self.armed, self.current, DRY_RUN, dog is not None, gc.mem_free()))
        elif c == 'X':
            if not self.armed:
                log('RX_EXIT')
                return False
            log('X_REFUSED armed')
        return True

    def tick(self, now):
        if not self.armed:
            return
        if time.ticks_diff(now, self.last_rx) > WATCHDOG_MS:
            self.stop('WATCHDOG')
        elif time.ticks_diff(now, self.arm_at) > MAX_RUN_MS:
            self.stop('RUN_LIMIT')
        elif self.desired != self.current and \
                time.ticks_diff(now, self.last_move) >= MIN_MOVE_GAP_MS:
            step, turn = MOTION[self.desired]
            if drive(step, turn):
                self.current = self.desired
                self.last_move = time.ticks_ms()
            else:
                self.stop('MOVE_FAIL')


def main():
    src = sys.stdin.buffer if hasattr(sys.stdin, 'buffer') else sys.stdin
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    rx = Rx()
    rx.stop('BOOT')
    log('ROBOT_READY RX2 dry=%d init=%d' % (DRY_RUN, dog is not None))
    try:
        running = True
        while running:
            for _ in poller.poll(10):
                b = src.read(1)
                if not b:
                    continue
                c = chr(b[0] if isinstance(b, (bytes, bytearray)) else ord(b)).upper()
                if c in '\r\n ':
                    continue
                if not rx.handle(c, time.ticks_ms()):
                    running = False
                    break
            rx.tick(time.ticks_ms())
    finally:
        rx.stop('EXIT')            # also runs on Ctrl-C / any exception


main()
