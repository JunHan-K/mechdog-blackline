"""STEP 4b/4c - test robot/mechdog_rx.py from RAM (nothing written to flash).

    py tools\\rx_test.py            # DRY_RUN: protocol, arming, watchdog, run limit, rate limit. No motion.
    py tools\\rx_test.py --motor    # REAL motors: ~0.6 s forward, stop; then watchdog stop test. Asks first.
"""
import argparse
import os
import re
import sys
import time

from mdlink import Board, BoardError, new_log_path

HERE = os.path.dirname(os.path.abspath(__file__))
RX_PATH = os.path.join(HERE, '..', 'robot', 'mechdog_rx.py')


class T:
    def __init__(self, b):
        self.b = b
        self.buf = b''
        self.results = []

    def pump(self, seconds):
        self.buf += self.b.read_for(seconds)

    def wait(self, token, timeout):
        """Wait for token in NEW output; returns (ok, elapsed_ms)."""
        t0 = time.monotonic()
        start = len(self.buf)
        while time.monotonic() - t0 < timeout:
            self.buf += self.b.read_available()
            if token.encode() in self.buf[start:]:
                return True, (time.monotonic() - t0) * 1000
        return False, (time.monotonic() - t0) * 1000

    def send(self, c):
        self.b.write(c.encode())

    def heartbeat(self, c, seconds, period=0.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(c)
            self.pump(period)

    def check(self, name, ok, extra=''):
        self.results.append((name, ok))
        print(('PASS ' if ok else 'FAIL ') + name + (f'  [{extra}]' if extra else ''))

    def count_since(self, mark, pattern):
        return len(re.findall(pattern.encode(), self.buf[mark:]))


def dry_tests(t):
    ok, _ = t.wait('ROBOT_READY RX2 dry=1', 5)
    t.check('receiver ready (dry)', ok)
    t.send('P'); ok, ms = t.wait('PONG RX2', 1); t.check('P -> PONG RX2', ok, f'{ms:.0f} ms')

    m = len(t.buf); t.heartbeat('F', 0.3)
    t.check('F before arming does nothing', t.count_since(m, 'DRY_MOVE') == 0)
    t.send('A'); ok, _ = t.wait('ARM', 1)
    t.check('A before I refused? (dry allows arm)', ok)
    t.send('D'); t.wait('DISARMED', 1)

    t.send('I'); ok, _ = t.wait('INIT_OK', 2); t.check('I -> INIT_OK', ok)
    t.send('A'); ok, _ = t.wait('ARMED', 1); t.check('A -> ARMED', ok)
    m = len(t.buf); t.heartbeat('F', 1.0)
    n = t.count_since(m, r'DRY_MOVE 30 0')
    t.check('steady F heartbeat -> exactly 1 move() call', n == 1, f'{n} calls')

    m = len(t.buf)
    end = time.monotonic() + 0.6
    i = 0
    while time.monotonic() < end:
        t.send('LR'[i % 2]); i += 1; t.pump(0.02)
    n = t.count_since(m, r'DRY_MOVE 30 -?15')
    t.check('L/R flapping is rate limited', n <= 5, f'{n} moves for {i} cmds')

    t.send('F'); t.pump(0.2)
    t0 = time.monotonic()
    ok, ms = t.wait('STOP WATCHDOG', 2)
    t.check('silence -> WATCHDOG stop', ok and 350 < ms + 200 < 900, f'{ms + 200:.0f} ms after last F (approx)')

    t.send('A'); t.wait('ARMED', 1)
    t0 = time.monotonic()
    rearmed = False
    stopped = False
    while time.monotonic() - t0 < 8:
        t.send('F')
        if not rearmed and time.monotonic() - t0 > 3:
            t.send('A'); rearmed = True
        t.pump(0.1)
        if b'STOP RUN_LIMIT' in t.buf[-400:]:
            stopped = True
            break
    el = time.monotonic() - t0
    t.check('run limit stops at ~6 s, re-A does not extend', stopped and 5.5 < el < 6.8, f'{el:.2f} s')

    t.send('Q'); ok, _ = t.wait('STATUS armed=0', 1); t.check('Q status', ok)
    t.send('X'); ok, _ = t.wait('RX_EXIT', 1); t.check('X exits cleanly', ok)


def motor_tests(t):
    ok, _ = t.wait('ROBOT_READY RX2 dry=0', 8)
    t.check('receiver ready (motor)', ok)
    t.send('I'); ok, _ = t.wait('INIT_OK', 8); t.check('init (reuse or create)', ok)
    t.send('A'); ok, _ = t.wait('ARMED', 1); t.check('armed', ok)
    print('>> forward ~0.6 s')
    m = len(t.buf)
    t.heartbeat('F', 0.6)
    t.send('D'); ok, ms = t.wait('DISARMED', 2)
    t.check('D -> stop', ok, f'{ms:.0f} ms')
    slow = re.findall(rb'SLOW_MOVE (\d+)', t.buf[m:])
    print('   move() slow calls (ms):', [int(s) for s in slow] or 'none > 60 ms')
    t.pump(1.0)
    print('>> watchdog test: one F then silence')
    t.send('A'); t.wait('ARMED', 1)
    t.send('F')
    ok, ms = t.wait('STOP WATCHDOG', 2)
    t.check('watchdog stops without PC commands', ok, f'{ms:.0f} ms')
    t.send('X'); ok, _ = t.wait('RX_EXIT', 2); t.check('exit', ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--motor', action='store_true')
    ap.add_argument('--yes', action='store_true', help='skip the YES prompt (operator confirmed elsewhere)')
    a = ap.parse_args()
    code = open(RX_PATH, 'rb').read()
    if not a.motor:
        assert b'DRY_RUN = False' in code
        code = code.replace(b'DRY_RUN = False', b'DRY_RUN = True', 1)
    else:
        print('REAL MOTOR TEST: robot on flat floor, cable clear, hand near power switch.')
        if not a.yes and input('Type YES to continue: ').strip() != 'YES':
            return 1
    log = new_log_path('rx_motor' if a.motor else 'rx_dry')
    b = Board(a.port, log_path=log)
    t = T(b)
    try:
        b.enter_raw()
        t.buf += b.send_code(code)
        (motor_tests if a.motor else dry_tests)(t)
    except BoardError as e:
        print('BOARD ERROR:', e)
    finally:
        try:
            b.write(b'D')
            b.read_until(b'\x04>', 2) or b.interrupt()
            b.exit_raw()
        except Exception:
            pass
        print('\n%d/%d passed, malloc errors: %d' % (
            sum(ok for _, ok in t.results), len(t.results), b.malloc_errors))
        b.close()
        print('Log:', log)
    return 0 if t.results and all(ok for _, ok in t.results) else 1


if __name__ == '__main__':
    sys.exit(main())
