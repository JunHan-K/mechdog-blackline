"""MechDog autonomous black-line follower. Just run it:

    py line_follow_auto.py

What happens:
 1. If the camera (192.168.5.1) is not reachable, switches Wi-Fi ONCE to HW_ESP32S3CAM
    (the BSSID in cam_bssid.txt if present - several teams' cameras share that name).
    A Wi-Fi switch made by the user during the run is left alone; on exit the previous Wi-Fi
    is restored only if the PC is still on the camera.
 2. Uploads robot/mechdog_rx.py to the robot RAM (flash untouched), robot stands up.
 3. First run only (or --calibrate): short turn test on the line to learn which turn
    direction moves the line which way in the image -> steer_calib.json.
 4. Follows the line by itself: proportional steering, slows down in curves.
    Line lost -> turns toward the side it was last seen for LOST_SEARCH s, then stops
    and waits; resumes automatically when the line is steadily visible again.

Stops immediately on: stale/no camera frame, serial error, robot watchdog, Q/ESC, session limit.
Keys (window): SPACE pause/resume, Q or ESC quit.  --headless: no window (Ctrl+C to quit).
Power switch = real emergency stop.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'tools'))
from mdlink import BoardError                             # noqa: E402
from line_follow_pc_v2 import Camera, Robot, CAMERA_URL   # noqa: E402
from linevision import LineDetector                       # noqa: E402

PORT = 'COM9'
CAM_SSID = 'HW_ESP32S3CAM'
CALIB_PATH = os.path.join(HERE, 'steer_calib.json')

GAIN = 1.4               # error (-1..1) * GAIN * 4 -> turn level
DEADBAND = 0.07
HEARTBEAT = 0.1
MAX_FRAME_AGE = 0.7      # 45 s test: Wi-Fi stalls of 0.41-0.43 s caused needless stops at 0.4
STABLE_TO_START = 0.3    # line must be seen this long before (re)starting
LOST_SEARCH = 1.0        # keep turning toward last side this long, then stop
SEGMENT_MAX_MS = 120000  # robot-side run limit per armed segment


# ---------------------------------------------------------------- Wi-Fi
def run(cmd):
    return subprocess.run(cmd, capture_output=True, text=True, errors='replace').stdout


def current_ssid():
    for line in run(['netsh', 'wlan', 'show', 'interfaces']).splitlines():
        s = line.strip()
        if s.startswith('SSID') and 'BSSID' not in s:
            return s.split(':', 1)[1].strip()
    return None


def camera_reachable():
    return 'TTL=' in run(['ping', '-n', '1', '-w', '700', '192.168.5.1'])


def pinned_bssid():
    """cam_bssid.txt (local, gitignored): this team's camera when several share HW_ESP32S3CAM.
    Create with: py tools\\find_my_camera.py --set <bssid>"""
    p = os.path.join(HERE, 'cam_bssid.txt')
    return open(p).read().strip().lower() if os.path.exists(p) else None


def ensure_camera_wifi(log):
    import wifi_bssid
    pin = pinned_bssid()
    ssid, bssid = wifi_bssid.current()
    if camera_reachable() and (not pin or bssid == pin):
        return None
    prev = ssid if ssid != CAM_SSID else None
    if pin:
        log(f'Switching Wi-Fi {ssid} -> {CAM_SSID} @ {pin}')
        if not wifi_bssid.connect(pin):
            raise RuntimeError(f'cannot join camera {pin} (powered? another camera nearby?)')
    else:
        log(f'Switching Wi-Fi {ssid} -> {CAM_SSID}')
        run(['netsh', 'wlan', 'connect', f'name={CAM_SSID}', f'ssid={CAM_SSID}'])
    for _ in range(30):
        if camera_reachable():
            return prev
        time.sleep(0.5)
    raise RuntimeError('camera Wi-Fi not reachable (is the camera powered?)')


def restore_wifi(prev, log):
    # only if still on the camera: a Wi-Fi the user picked during the run is left alone
    if prev and prev != CAM_SSID and current_ssid() == CAM_SSID:
        log(f'Restoring Wi-Fi -> {prev}')
        run(['netsh', 'wlan', 'connect', f'name={prev}', f'ssid={prev}'])


# ---------------------------------------------------------------- helpers
class Vision:
    def __init__(self, cam):
        self.cam = cam
        self.det = LineDetector()
        self.res = None
        self.age = float('inf')

    def update(self):
        frame, self.age = self.cam.latest()
        self.res = self.det.detect(frame) if frame is not None else None
        return self.res

    @property
    def fresh(self):
        return self.age <= MAX_FRAME_AGE

    @property
    def found(self):
        return bool(self.res and self.res['found'])


def digit(raw_level):
    return str(4 + max(-4, min(4, raw_level)))


def calibrate(robot, vis, log):
    """Turn with raw +level, watch where the line moves in the image."""
    def avg_x(seconds):
        xs, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            vis.update()
            robot.poll()
            if vis.fresh and vis.found:
                xs.append(vis.res['x_near'])
            time.sleep(0.03)
        return sum(xs) / len(xs) if xs else None

    log('Calibration: put the robot ON the line. Waiting for the line...')
    end = time.monotonic() + 30
    x0 = None
    while time.monotonic() < end and x0 is None:
        x0 = avg_x(1.0)
    if x0 is None:
        raise RuntimeError('calibration: no line visible')
    for dur in (1.0, 1.6):
        if not robot.arm():
            raise RuntimeError('calibration: robot did not arm')
        last_x, t_end = x0, time.monotonic() + dur
        while time.monotonic() < t_end:
            robot.send(digit(+3))
            for _ in range(3):
                vis.update()
                if vis.fresh and vis.found:
                    last_x = vis.res['x_near']
                time.sleep(HEARTBEAT / 3)
        robot.stop()
        x1 = avg_x(0.8)
        x1 = x1 if x1 is not None else last_x
        dx = x1 - x0
        log(f'Calibration: x0={x0:+.2f} x1={x1:+.2f} dx={dx:+.2f} ({dur}s)')
        # turn back the same amount
        robot.arm()
        t_end = time.monotonic() + dur
        while time.monotonic() < t_end:
            robot.send(digit(-3))
            robot.poll()
            time.sleep(HEARTBEAT)
        robot.stop()
        time.sleep(0.5)
        if abs(dx) >= 0.12:
            sign = 1 if dx > 0 else -1
            json.dump({'sign': sign, 'dx': dx, 'time': time.strftime('%Y-%m-%d %H:%M:%S')},
                      open(CALIB_PATH, 'w'))
            log(f'Calibration saved: positive raw turn moves line {"right" if sign > 0 else "left"}')
            return sign
        x0 = avg_x(1.0) or x0
    raise RuntimeError('calibration inconclusive (line did not move in image)')


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--headless', action='store_true')
    ap.add_argument('--calibrate', action='store_true')
    ap.add_argument('--max-minutes', type=float, default=10)
    ap.add_argument('--seconds', type=float, default=0, help='stop session after N s (testing)')
    args = ap.parse_args()

    os.makedirs(os.path.join(HERE, 'logs'), exist_ok=True)
    logf = open(os.path.join(HERE, 'logs', time.strftime('auto_%Y%m%d_%H%M%S.txt')), 'w', encoding='utf-8')
    t_start = time.monotonic()

    def log(msg):
        line = f'[{time.monotonic() - t_start:7.2f}] {msg}'
        print(line, flush=True)
        logf.write(line + '\n')
        logf.flush()

    prev_wifi = None
    robot = cam = None
    try:
        prev_wifi = ensure_camera_wifi(log)
        cam = Camera(CAMERA_URL)
        vis = Vision(cam)
        log('Connecting robot...')
        robot = Robot(args.port)
        robot.start({'MAX_RUN_MS': SEGMENT_MAX_MS})
        log('Robot ready.')
        for _ in range(50):          # wait for first camera frame
            vis.update()
            if vis.res:
                break
            time.sleep(0.1)

        if args.calibrate or not os.path.exists(CALIB_PATH):
            sign = calibrate(robot, vis, log)
        else:
            sign = json.load(open(CALIB_PATH))['sign']
            log(f'Using calibration sign={sign:+d} ({CALIB_PATH})')

        limit = args.seconds or args.max_minutes * 60
        state, paused = 'WAIT', False
        seen_since = lost_since = None
        last_err, last_send, last_stat, last_dump = 0.0, 0.0, 0.0, 0.0
        dump_dir = os.path.join(HERE, 'logs', time.strftime('frames_%Y%m%d_%H%M%S'))
        os.makedirs(dump_dir, exist_ok=True)
        stats = {'FOLLOW': 0, 'SEARCH': 0, 'WAIT': 0}

        def set_state(s, why=''):
            nonlocal state
            if s != state:
                log(f'{state} -> {s} {why}')
                if s == 'WAIT':
                    robot.stop()
                state = s

        while time.monotonic() - t_start < limit:
            vis.update()
            robot.poll()
            now = time.monotonic()
            found = vis.fresh and vis.found
            stats[state] += 1

            if not vis.fresh:
                set_state('WAIT', f'(camera frame age {vis.age:.2f}s)')
            elif paused:
                set_state('WAIT', '(paused)')
            elif state in ('FOLLOW', 'SEARCH') and robot.state != 'ARMED':
                set_state('WAIT', f'(robot: {robot.last_event or "disarmed"})')
            elif state == 'WAIT':
                seen_since = (seen_since or now) if found else None
                if seen_since and now - seen_since >= STABLE_TO_START:
                    if robot.arm():
                        set_state('FOLLOW', '(line stable)')
                        last_send = 0.0
                    else:
                        log('arm not confirmed')
            elif state == 'FOLLOW' and not found:
                lost_since = now
                set_state('SEARCH', f'(line lost, last error {last_err:+.2f})')
            elif state == 'SEARCH':
                if found:
                    set_state('FOLLOW', '(line reacquired)')
                elif now - lost_since > LOST_SEARCH:
                    seen_since = None
                    set_state('WAIT', '(line not found)')

            if state in ('FOLLOW', 'SEARCH') and now - last_send >= HEARTBEAT:
                if state == 'FOLLOW':
                    err = vis.res['error']
                    last_err = err
                    lvl = 0 if abs(err) < DEADBAND else round(err * GAIN * 4)
                else:
                    lvl = 4 if last_err > 0 else -4
                # line right of centre (err>0) -> want line to move left in image
                raw = -sign * max(-4, min(4, lvl))
                robot.send(digit(raw))
                last_send = now

            if vis.res and now - last_dump > 2:
                last_dump = now
                cv2.imwrite(os.path.join(dump_dir, f'{now - t_start:06.1f}_{state}.jpg'), vis.res['overlay'])

            if now - last_stat > 5:
                last_stat = now
                e = vis.res['error'] if vis.res and vis.res['found'] else None
                log(f'state={state} err={e if e is None else round(e, 2)} age={vis.age:.2f} robot={robot.state}')

            if not args.headless and vis.res:
                img = vis.res['overlay']
                e = vis.res['error']
                txt = f'{state}{" PAUSED" if paused else ""} err={e:+.2f}' if e is not None else f'{state} NO LINE'
                cv2.putText(img, txt, (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                            (0, 255, 0) if vis.fresh else (0, 0, 255), 2)
                cv2.imshow('MechDog auto', img)
                cv2.imshow('mask (far / near)', vis.res['mask'])
                k = cv2.waitKey(1) & 0xFF
                if k in (ord('q'), 27):
                    log('quit key')
                    break
                if k == 32:
                    paused = not paused
                    log('paused' if paused else 'resumed')
            else:
                time.sleep(0.01)
        else:
            log('session time limit')
        log(f'loop counts {stats}')
        return 0
    except KeyboardInterrupt:
        log('Ctrl+C')
        return 0
    except (BoardError, RuntimeError, OSError) as e:
        log(f'ERROR: {e}')
        return 1
    finally:
        if robot:
            robot.close()
        if cam:
            cam.close()
        cv2.destroyAllWindows()
        restore_wifi(prev_wifi, log)
        log('exit')
        logf.close()


if __name__ == '__main__':
    sys.exit(main())
