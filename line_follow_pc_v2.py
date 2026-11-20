"""MechDog black line follower v2 (Windows PC). Original line_follow_pc.py is left unchanged.

Run from the project folder, PC connected to HW_ESP32S3CAM Wi-Fi, robot USB on COM9:
    py line_follow_pc_v2.py --no-robot          # stage 1: camera + detection only, robot untouched
    py line_follow_pc_v2.py                     # stages 2-4: robot connected
    py line_follow_pc_v2.py --no-robot --selftest 8   # headless camera check, prints stats

Keys (window focused):
    W        manual forward pulse (0.6 s)      A / D   manual left / right pulse (0.6 s)
    G        auto line following (max RUN_SECONDS)
    S/SPACE  stop                              Q       quit (robot stopped, receiver exited)

Robot side: robot/mechdog_rx.py is uploaded to RAM at start (nothing written to flash).
Stops when: line lost > LINE_LOST_GRACE, frame older than MAX_FRAME_AGE, camera error,
serial error, time limit, key S/space/Q. Robot itself stops ~0.5 s after PC commands stop.
"""
import argparse
import os
import re
import sys
import threading
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tools'))
from mdlink import Board, BoardError, new_log_path   # noqa: E402
from line_follow_pc import black_line                 # noqa: E402  (same detector as v1)

CAMERA_URL = 'http://192.168.5.1:81/stream'
PORT = 'COM9'
RUN_SECONDS = 5.0
PULSE_SECONDS = 0.6
HEARTBEAT_SECONDS = 0.1
MAX_FRAME_AGE = 0.4       # camera measured ~6 fps (frame age max 0.24 s) on 2026-11-20
LINE_LOST_GRACE = 0.25
INVERT_STEERING = False   # True if the robot steers AWAY from the line
RX_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'robot', 'mechdog_rx.py')


class Camera:
    """Background grabber: always keeps only the newest frame + its timestamp."""

    def __init__(self, url):
        self.url = url
        self.lock = threading.Lock()
        self.frame = None
        self.stamp = 0.0
        self.error = 'starting'
        self.frames = 0
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.running:
            cap = cv2.VideoCapture(self.url)
            if not cap.isOpened():
                self.error = 'cannot open stream (PC on HW_ESP32S3CAM Wi-Fi?)'
                time.sleep(1.0)
                continue
            self.error = None
            while self.running:
                ok, frame = cap.read()
                if not ok:
                    self.error = 'stream read failed, reconnecting'
                    break
                with self.lock:
                    self.frame, self.stamp = frame, time.monotonic()
                    self.frames += 1
            cap.release()
            time.sleep(0.5)

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, float('inf')
            return self.frame.copy(), time.monotonic() - self.stamp

    def close(self):
        self.running = False


class Robot:
    """Owns the serial link; tracks the robot's own reported state."""

    def __init__(self, port):
        self.log_path = new_log_path('drive')
        self.b = Board(port, log_path=self.log_path)
        self.buf = b''
        self.state = 'DISARMED'
        self.last_event = ''

    def start(self, overrides=None):
        """overrides: {'MAX_RUN_MS': 120000, ...} replaces 'NAME = value' lines in mechdog_rx.py."""
        code = open(RX_PATH, 'rb').read().decode()
        for name, value in (overrides or {}).items():
            pat = re.compile(r'^%s = .*$' % name, re.M)
            if not pat.search(code):
                raise BoardError('override target not found: ' + name)
            code = pat.sub('%s = %r' % (name, value), code, count=1)
        self.b.enter_raw()
        self.buf = self.b.send_code(code.encode())
        if not self.wait('ROBOT_READY RX2 dry=0', 5):
            raise BoardError('receiver did not start')
        print('Receiver running. Initialising MechDog (robot may stand up, ~4 s)...')
        self.send('I')
        if not self.wait('INIT_OK', 10):
            raise BoardError('MechDog init failed: %r' % self.buf[-200:])
        self.send('P')
        if not self.wait('PONG RX2', 1):
            raise BoardError('no PONG after init')

    def wait(self, token, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.poll()
            if token.encode() in self.buf:
                self.buf = self.buf[self.buf.find(token.encode()) + len(token):]
                return True
            time.sleep(0.01)
        return False

    def send(self, c):
        self.b.write(c.encode())

    def poll(self):
        """Non-blocking read; updates state from robot messages."""
        n = self.b.s.in_waiting
        if not n:
            return
        data = self.b._got(self.b.s.read(n))
        self.buf += data
        for line in data.decode('utf-8', 'replace').splitlines():
            line = line.strip()
            if line == 'ARMED':
                self.state = 'ARMED'
            elif line.startswith('STOP') or line == 'DISARMED':
                self.state = 'DISARMED'
                if line.startswith('STOP') and line != 'STOP CMD':
                    self.last_event = line
                    print('Robot:', line)
            elif line.startswith(('SLOW_MOVE', 'MOVE_ERR', 'INIT_ERR')):
                self.last_event = line
                print('Robot:', line)
        if len(self.buf) > 4000:
            self.buf = self.buf[-2000:]

    def arm(self):
        self.send('A')
        return self.wait('ARMED', 0.5)

    def stop(self):
        try:
            self.send('D')
        except Exception:
            pass

    def close(self):
        try:
            self.send('D')
            time.sleep(0.1)
            self.send('X')
            self.b.read_until(b'\x04>', 1.5)
            self.b.exit_raw()
        except Exception:
            try:
                self.b.interrupt()
            except Exception:
                pass
        finally:
            self.b.close()


def steer(direction):
    if direction == 'CENTER':
        return 'F'
    left = direction == 'LEFT'
    if INVERT_STEERING:
        left = not left
    return 'L' if left else 'R'


def selftest(cam, seconds):
    end = time.monotonic() + seconds
    counts, ages = {}, []
    while time.monotonic() < end:
        frame, age = cam.latest()
        if frame is not None:
            d, _ = black_line(frame)
            counts[d] = counts.get(d, 0) + 1
            ages.append(age)
        time.sleep(0.1)
    print('camera error:', cam.error, '| frames grabbed:', cam.frames)
    if ages:
        print('direction counts:', counts)
        print('frame age s: max %.2f avg %.2f' % (max(ages), sum(ages) / len(ages)))
        print('fps approx: %.1f' % (cam.frames / seconds))
    return 0 if ages else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--no-robot', action='store_true')
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--selftest', type=float, default=0)
    args = ap.parse_args()

    cam = Camera(CAMERA_URL)
    robot = None
    try:
        if args.selftest:
            return selftest(cam, args.selftest)
        if not args.no_robot:
            robot = Robot(args.port)
            robot.start()
            print('Robot log:', robot.log_path)

        mode = 'STOP'          # STOP | PULSE | AUTO
        pulse_cmd, mode_until, last_send, lost_since = None, 0.0, 0.0, None
        print('Keys: W/A/D pulse, G auto, S/space stop, Q quit')

        def halt(reason):
            nonlocal mode
            if robot:
                robot.stop()
            if mode != 'STOP':
                print('STOP:', reason)
            mode = 'STOP'

        while True:
            frame, age = cam.latest()
            now = time.monotonic()
            if frame is None:
                frame = np.zeros((240, 320, 3), np.uint8)
                direction, mask = 'NO FRAME', None
            else:
                direction, mask = black_line(frame)
            fresh = age <= MAX_FRAME_AGE

            if robot:
                try:
                    robot.poll()
                except Exception as e:
                    print('Serial error:', e)
                    break
                if mode != 'STOP' and robot.state != 'ARMED':
                    halt('robot reported ' + (robot.last_event or 'disarmed'))

            # ---- safety checks for any motion ----
            if mode != 'STOP':
                if not fresh:
                    halt('stale/no camera frame (%.2f s)' % age)
                elif now >= mode_until:
                    halt('time limit')
                elif mode == 'AUTO':
                    if direction == 'NO LINE':
                        lost_since = lost_since or now
                        if now - lost_since > LINE_LOST_GRACE:
                            halt('line lost')
                    else:
                        lost_since = None

            # ---- heartbeat ----
            if mode != 'STOP' and robot and now - last_send >= HEARTBEAT_SECONDS:
                cmd = pulse_cmd if mode == 'PULSE' else (steer(direction) if direction in ('LEFT', 'RIGHT', 'CENTER') else None)
                if cmd:
                    robot.send(cmd)
                    last_send = now

            # ---- display ----
            rstate = robot.state if robot else 'NO ROBOT'
            cv2.putText(frame, f'{direction} | {mode} | robot {rstate}', (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if fresh else (0, 0, 255), 2)
            cv2.putText(frame, f'frame age {age:.2f}s  {cam.error or ""}', (8, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame, 'W/A/D pulse  G auto  S/SPACE stop  Q quit', (8, 74),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow('MechDog Black Line v2', frame)
            if mask is not None:
                cv2.imshow('Black Mask', mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                halt('quit')
                break
            if key in (ord('s'), 32):
                halt('manual stop')
            if robot and mode == 'STOP' and key in (ord('w'), ord('a'), ord('d'), ord('g')):
                if not fresh:
                    print('Refused: camera frame not fresh')
                    continue
                if key == ord('g') and direction not in ('LEFT', 'RIGHT', 'CENTER'):
                    print('Refused: no line visible')
                    continue
                if not robot.arm():
                    print('Refused: robot did not confirm ARMED')
                    robot.stop()
                    continue
                if key == ord('g'):
                    mode, mode_until = 'AUTO', time.monotonic() + RUN_SECONDS
                    lost_since = None
                else:
                    mode, mode_until = 'PULSE', time.monotonic() + PULSE_SECONDS
                    pulse_cmd = {'w': 'F', 'a': 'L', 'd': 'R'}[chr(key)]
                last_send = 0.0
                print('START', mode, pulse_cmd if mode == 'PULSE' else '')
    except BoardError as e:
        print('ROBOT ERROR:', e)
        return 1
    finally:
        if robot:
            robot.close()
        cam.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
