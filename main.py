# MechDog main ESP32 only (MicroPython). Not for the ESP32-S3 camera.
# Receives 1-byte commands on USB-C / REPL serial (UART0): P A F L R S D.
# Watchdog and max run time are safety features, not a certified safety system.
import sys
import time
try:
    import uselect as select
except ImportError:
    import select
from HW_MechDog import MechDog

STEP_MM = 30
TURN_DEG = 15
WATCHDOG_MS = 600
MAX_RUN_MS = 6000

dog = MechDog()
current = 'S'
armed = False
last_rx = time.ticks_ms()
arm_at = last_rx


def stop():
    global current
    dog.move(0, 0)
    current = 'S'


def motion(command):
    global current
    if command == current:
        return
    if command == 'F':
        dog.move(STEP_MM, 0)
    elif command == 'L':
        dog.move(STEP_MM, TURN_DEG)
    elif command == 'R':
        dog.move(STEP_MM, -TURN_DEG)
    current = command


def main():
    global armed, last_rx, arm_at
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    stop()                  # Always start stationary
    print('ROBOT_READY')
    while True:
        for event in poller.poll(20):
            if not (event[1] & select.POLLIN):
                continue
            ch = sys.stdin.read(1)  # exactly one character; read() may block
            if not ch:
                continue
            if isinstance(ch, bytes):
                ch = ch.decode()
            ch = ch[0].upper()
            now = time.ticks_ms()
            if ch == 'P':
                print('PONG')
            elif ch == 'A':
                stop()
                armed = True
                last_rx = now
                arm_at = now
                print('ARMED')
            elif ch in ('D', 'S'):
                armed = False
                stop()
                print('DISARMED')
            elif ch in ('F', 'L', 'R') and armed:
                last_rx = now
                motion(ch)
        if armed:
            now = time.ticks_ms()
            if time.ticks_diff(now, last_rx) > WATCHDOG_MS:
                armed = False
                stop()
                print('WATCHDOG_STOP')
            elif time.ticks_diff(now, arm_at) > MAX_RUN_MS:
                armed = False
                stop()
                print('RUN_LIMIT_STOP')


try:
    main()
except BaseException:
    stop()
    print('ROBOT_STOPPED')
    raise
