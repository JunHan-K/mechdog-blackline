"""Safety-vest check with MechDog head-lamp feedback. Rule based (OpenCV only, no training).

    py vest_check.py                    # ESP32-S3 camera + robot lamp (joins camera Wi-Fi once; never fights a manual switch)
    py vest_check.py --source 0         # PC webcam instead
    py vest_check.py --source photo.jpg # still image
    py vest_check.py --no-robot         # no lamp, screen only
    py vest_check.py --zones            # also check tool tags in hook zones A-D
    py vest_check.py --selftest         # synthetic images, no camera / robot

Wear check (default):
  vest   = biggest blob of the vest colour (default fluorescent yellow/lime; click-calibrate with v)
  face   = OpenCV FaceDetectorYN, models/face_detection_yunet_2023mar.onnx (optional: if found, vest must sit below it, under the chin)
  closed = centre column of the vest box is vest-coloured about as much as its sides
           (open vest -> shirt visible in the middle -> FAIL)
  size   = vest wide enough (>= 1.6 face widths if a face is seen)
Verdict must hold STABLE s before the lamp changes:  PASS -> green,  FAIL -> red,
nobody / only the operator's own big face / camera lost -> blue (idle).

Zones (--zones): rectangles relative to the vest box, each expects a tool tag colour
(A red, B blue, C magenta, D cyan). OK / EMPTY / WRONG; all OK needed for PASS.

Keys: v + click sample vest colour   a-d select zone   t + click sample zone tool colour
      r + drag redraw zone   f freeze frame   s save vest_config.json   q/ESC quit
Educational demo, not a certified safety check.
"""
import argparse
import copy
import json
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(HERE, 'vest_config.json')
SCALE = 2  # window magnification for 320x240 frames

# OpenCV HSV: H 0-179. h lo > hi means wrap-around (red).
DEFAULT_CONFIG = {
    # list of colour ranges, any match = vest: fluorescent lime/yellow, fluorescent orange
    'vest': [{'h': [20, 45], 's': [90, 255], 'v': [90, 255]},
             {'h': [3, 20], 's': [150, 255], 'v': [120, 255]}],
    'tools': {
        'red': {'h': [170, 8], 's': [110, 255], 'v': [70, 255]},
        'blue': {'h': [100, 125], 's': [110, 255], 'v': [50, 255]},
        'magenta': {'h': [140, 168], 's': [90, 255], 'v': [70, 255]},
        'cyan': {'h': [85, 99], 's': [110, 255], 'v': [70, 255]},
    },
    'zones': {
        'A': {'rect': [0.02, 0.10, 0.35, 0.45], 'tool': 'red'},
        'B': {'rect': [0.65, 0.10, 0.98, 0.45], 'tool': 'blue'},
        'C': {'rect': [0.02, 0.55, 0.35, 0.95], 'tool': 'magenta'},
        'D': {'rect': [0.65, 0.55, 0.98, 0.95], 'tool': 'cyan'},
    },
    'min_vest_frac': 0.03,    # total vest-colour area / image area
    'check_closed': False,    # True: centre of vest must be vest-coloured (plain vests only)
    'closed_ratio': 0.6,      # centre fill must be >= this * side fill
    'min_tag_frac': 0.03,     # tag pixels / zone area
    'stable_s': 1.0,
}
ZONE_ORDER = 'ABCD'
ZCOL = {'OK': (0, 200, 0), 'EMPTY': (0, 200, 255), 'WRONG': (0, 0, 255)}
LAMP = {'PASS': 'G', 'FAIL': 'E', 'NOBODY': 'B'}   # blue = idle / waiting for a photo
OPERATOR_FACE = 0.30      # face wider than this fraction of the image = the operator up close, not a photo
STALE_S = 1.5             # older frame = camera lost
YUNET = os.path.join(HERE, 'models', 'face_detection_yunet_2023mar.onnx')
_face_det = None


def find_faces(frame):
    """All faces [(x, y, w, h)], biggest first. OpenCV FaceDetectorYN (YuNet); skipped if model file missing."""
    global _face_det
    if not os.path.exists(YUNET):
        return []
    h, w = frame.shape[:2]
    if _face_det is None:
        _face_det = cv2.FaceDetectorYN.create(YUNET, '', (w, h), 0.7)
    _face_det.setInputSize((w, h))
    _, faces = _face_det.detect(frame)
    if faces is None or not len(faces):
        return []
    return [tuple(int(v) for v in f[:4]) for f in sorted(faces, key=lambda r: -r[2] * r[3])]


def load_config():
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if os.path.exists(CONFIG_PATH):
        cfg.update(json.load(open(CONFIG_PATH, encoding='utf-8')))
    if isinstance(cfg['vest'], dict):
        cfg['vest'] = [cfg['vest']]
    return cfg


def color_mask(hsv, r):
    if isinstance(r, list):
        m = np.zeros(hsv.shape[:2], np.uint8)
        for x in r:
            m |= color_mask(hsv, x)
        return m
    sv = cv2.inRange(hsv, np.array([0, r['s'][0], r['v'][0]]), np.array([179, r['s'][1], r['v'][1]]))
    h = hsv[:, :, 0]
    h0, h1 = r['h']
    hm = ((h >= h0) & (h <= h1)) if h0 <= h1 else ((h >= h0) | (h <= h1))
    return cv2.morphologyEx(sv & (hm.astype(np.uint8) * 255), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def sample_range(hsv, x, y):
    patch = hsv[max(0, y - 3):y + 4, max(0, x - 3):x + 4].reshape(-1, 3)
    h, s, v = [int(c) for c in np.median(patch, axis=0)]
    return {'h': [(h - 10) % 180, (h + 10) % 180],
            's': [max(40, s - 70), 255], 'v': [max(30, v - 80), 255]}, (h, s, v)


def fill(mask, x0, y0, x1, y1):
    sub = mask[y0:y1, x0:x1]
    return cv2.countNonZero(sub) / max(1, sub.size)


def evaluate(frame, cfg, use_zones=False):
    """Returns dict: vest box, face box, checks {name: bool}, zones, verdict (PASS/FAIL/NOBODY), reasons."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    H, W = frame.shape[:2]
    res = {'vest': None, 'face': None, 'checks': {}, 'zones': {}, 'reasons': []}

    # a huge face is the operator standing right at the camera, not a person in the shown photo -> ignore
    faces = [f for f in find_faces(frame) if f[2] < OPERATOR_FACE * W]
    res['face'] = faces[0] if faces else None

    vm = color_mask(hsv, cfg['vest'])
    vm = cv2.morphologyEx(vm, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    res['mask'] = vm
    cnts, _ = cv2.findContours(vm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # real vests are cut into pieces by black panels / reflective stripes / zipper:
    # keep every piece >= 0.4 % of the image and require the TOTAL to reach min_vest_frac
    big = [c for c in cnts if cv2.contourArea(c) >= 0.004 * H * W]
    if sum(cv2.contourArea(c) for c in big) < cfg['min_vest_frac'] * H * W:
        res['verdict'] = 'FAIL' if res['face'] else 'NOBODY'
        res['reasons'] = ['no vest'] if res['face'] else []
        return res
    # two halves of an open vest are separate blobs -> use the union of all big blobs
    x, y, w, h = cv2.boundingRect(np.vstack(big))
    res['vest'] = (x, y, w, h)
    ck = res['checks']

    # closed: centre column vs side columns, middle rows of the vest box
    r0, r1 = y + int(0.25 * h), y + int(0.85 * h)
    centre = fill(vm, x + int(0.42 * w), r0, x + int(0.58 * w), r1)
    sides = (fill(vm, x + int(0.08 * w), r0, x + int(0.30 * w), r1)
             + fill(vm, x + int(0.70 * w), r0, x + int(0.92 * w), r1)) / 2
    closed = sides > 0 and centre >= cfg['closed_ratio'] * sides and centre > 0.25
    res['fills'] = (round(centre, 2), round(sides, 2))
    if cfg['check_closed']:          # off by default: vests with dark centre panels look 'open'
        ck['closed'] = closed

    # the wearer's face = a face centred over the vest, just above it (a held-up photo may also show the\n    # real person behind it -> do not just take the biggest face). No such face -> judge by the vest only.\n    above = [f for f in faces if x <= f[0] + f[2] / 2 <= x + w and f[1] + 0.5 * f[3] <= y <= f[1] + 2.5 * f[3]]\n    res['face'] = above[0] if above else res['face']\n    if above:\n        ck['wide'] = w >= 1.6 * above[0][2]

    if use_zones:
        tag_masks = {n: color_mask(hsv, r) for n, r in cfg['tools'].items()}
        for zname in ZONE_ORDER:
            z = cfg['zones'][zname]
            x0, y0, x1, y1 = z['rect']
            px0, px1 = int(np.clip(x + x0 * w, 0, W)), int(np.clip(x + x1 * w, 0, W))
            py0, py1 = int(np.clip(y + y0 * h, 0, H)), int(np.clip(y + y1 * h, 0, H))
            fr = {n: fill(m, px0, py0, px1, py1) for n, m in tag_masks.items()}
            best = max(fr, key=fr.get)
            st = 'EMPTY' if fr[best] < cfg['min_tag_frac'] else ('OK' if best == z['tool'] else 'WRONG')
            res['zones'][zname] = {'box': (px0, py0, px1, py1), 'status': st, 'seen': best}
            ck['zone_' + zname] = st == 'OK'

    res['reasons'] = [k for k, v in ck.items() if not v]
    res['verdict'] = 'FAIL' if res['reasons'] else 'PASS'
    return res


class Stabilizer:
    """Commits a verdict only after it has been seen continuously for stable_s."""

    def __init__(self, stable_s):
        self.stable_s, self.cand, self.since, self.value = stable_s, None, 0.0, None

    def update(self, v, now):
        if v != self.cand:
            self.cand, self.since = v, now
        if self.cand != self.value and now - self.since >= self.stable_s:
            self.value = self.cand
            return True
        return False


def draw(frame, res, shown, sel, mode):
    out = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
    if res['face']:
        x, y, w, h = [v * SCALE for v in res['face']]
        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 200, 0), 2)
    if res['vest']:
        x, y, w, h = [v * SCALE for v in res['vest']]
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.rectangle(out, (x + int(0.42 * w), y + int(0.25 * h)), (x + int(0.58 * w), y + int(0.85 * h)),
                      (255, 255, 255), 1)
    for z, r in res['zones'].items():
        x0, y0, x1, y1 = [v * SCALE for v in r['box']]
        col = ZCOL[r['status']]
        cv2.rectangle(out, (x0, y0), (x1, y1), col, 3 if z == sel else 2)
        cv2.putText(out, f"{z}:{r['status']}", (x0 + 3, y0 + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    col = {'PASS': (0, 200, 0), 'FAIL': (0, 0, 255)}.get(shown, (180, 180, 180))
    cv2.putText(out, f'{shown or "..."}  {" ".join(res["reasons"])}', (8, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, col, 2)
    cv2.putText(out, f'mode {mode or "-"} zone {sel} | v t a-d r f s q', (8, out.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return out


# ---------------------------------------------------------------- sources
def open_source(src, log):
    """-> (get_frame() returning (frame, age_s), close()). Shared with hazard_check.py."""
    if os.path.isfile(src):
        img = cv2.imread(src)
        if img is None:
            raise RuntimeError('cannot read image ' + src)
        if img.shape[1] > 640:
            img = cv2.resize(img, (640, int(img.shape[0] * 640 / img.shape[1])))
        return (lambda: (img.copy(), 0.0)), (lambda: None)
    if src.isdigit():
        cap = cv2.VideoCapture(int(src))
        if not cap.isOpened():
            raise RuntimeError('cannot open webcam ' + src)
        return (lambda: (cap.read()[1], 0.0)), cap.release
    # Wi-Fi: joins the camera ONCE at start (pinned BSSID). If the user switches Wi-Fi while running,
    # it is left alone (no reconnect); on exit the old Wi-Fi is restored only if still on the camera.
    from line_follow_pc_v2 import Camera
    from line_follow_auto import ensure_camera_wifi, restore_wifi, CAM_SSID
    import wifi_bssid
    os.environ.setdefault('OPENCV_FFMPEG_CAPTURE_OPTIONS', 'timeout;3000000|rw_timeout;3000000')
    prev = ensure_camera_wifi(log)
    cam = Camera(src)

    def close():
        cam.close()
        if wifi_bssid.current()[0] == CAM_SSID:
            restore_wifi(prev, log)
    return cam.latest, close


def selftest():
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    cfg['check_closed'] = True
    lime = (40, 240, 220)
    tagc = {'red': (30, 30, 220), 'blue': (220, 60, 20), 'magenta': (200, 40, 200), 'cyan': (220, 220, 30)}

    def scene(vest='closed', tools=None):
        img = np.full((240, 320, 3), (120, 120, 120), np.uint8)
        if vest == 'closed':
            cv2.rectangle(img, (80, 40), (240, 230), lime, -1)
        elif vest == 'open':
            cv2.rectangle(img, (80, 40), (140, 230), lime, -1)
            cv2.rectangle(img, (180, 40), (240, 230), lime, -1)
            cv2.rectangle(img, (141, 40), (179, 230), (60, 60, 160), -1)   # shirt
        spots = {'A': (100, 80), 'B': (220, 80), 'C': (100, 180), 'D': (220, 180)}
        for z, t in (tools or {}).items():
            cx, cy = spots[z]
            cv2.rectangle(img, (cx - 10, cy - 10), (cx + 10, cy + 10), tagc[t], -1)
        return img

    allt = {'A': 'red', 'B': 'blue', 'C': 'magenta', 'D': 'cyan'}
    cases = [('closed vest', scene('closed'), False, 'PASS'),
             ('open vest', scene('open'), False, 'FAIL'),
             ('nobody', scene('none'), False, 'NOBODY'),
             ('zones ok', scene('closed', allt), True, 'PASS'),
             ('zones A<->B', scene('closed', dict(allt, A='blue', B='red')), True, 'FAIL'),
             ('zone D empty', scene('closed', {k: v for k, v in allt.items() if k != 'D'}), True, 'FAIL')]
    ok = True
    for name, img, z, want in cases:
        r = evaluate(img, cfg, z)
        good = r['verdict'] == want
        ok &= good
        print(f'{name:13s} -> {r["verdict"]:6s} {",".join(r["reasons"]):24s} {"ok" if good else "MISMATCH want " + want}')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='http://192.168.5.1:81/stream')
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--no-robot', action='store_true')
    ap.add_argument('--zones', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    sys.path.insert(0, os.path.join(HERE, 'tools'))

    cfg = load_config()
    stab = Stabilizer(cfg['stable_s'])
    st = {'sel': 'A', 'mode': None, 'drag': None, 'frozen': None, 'hsv': None, 'res': None}
    robot = None
    close = None
    try:
        if not args.no_robot:
            from line_follow_pc_v2 import Robot
            robot = Robot(args.port)
            robot.start(init=False)          # lamp only: robot does not stand up / move
            robot.send('B')
            print('Robot lamp ready (blue = waiting).')
        get_frame, close = open_source(args.source, print)
        camera_ok = True

        def on_mouse(ev, x, y, flags, _):
            x, y = x // SCALE, y // SCALE
            if st['hsv'] is None:
                return
            if st['mode'] in ('tool', 'vest') and ev == cv2.EVENT_LBUTTONDOWN:
                rng, hsv = sample_range(st['hsv'], x, y)
                if st['mode'] == 'vest':
                    cfg['vest'] = [rng]
                    print('vest colour HSV', hsv, '->', rng)
                else:
                    tool = cfg['zones'][st['sel']]['tool']
                    cfg['tools'][tool] = rng
                    print(f'tool {tool} (zone {st["sel"]}) HSV', hsv, '->', rng)
                st['mode'] = None
            elif st['mode'] == 'rect':
                if ev == cv2.EVENT_LBUTTONDOWN:
                    st['drag'] = (x, y)
                elif ev == cv2.EVENT_LBUTTONUP and st['drag'] and st['res'] and st['res']['vest']:
                    vx, vy, vw, vh = st['res']['vest']
                    (ax, ay), (bx, by) = st['drag'], (x, y)
                    cfg['zones'][st['sel']]['rect'] = [
                        round((min(ax, bx) - vx) / vw, 3), round((min(ay, by) - vy) / vh, 3),
                        round((max(ax, bx) - vx) / vw, 3), round((max(ay, by) - vy) / vh, 3)]
                    print('zone', st['sel'], 'rect', cfg['zones'][st['sel']]['rect'])
                    st['mode'], st['drag'] = None, None

        snap_dir = os.path.join(HERE, 'logs', time.strftime('vest_%Y%m%d_%H%M%S'))
        os.makedirs(snap_dir, exist_ok=True)
        last_snap = 0.0
        cv2.namedWindow('vest check')
        cv2.setMouseCallback('vest check', on_mouse)
        while True:
            frame, age = (st['frozen'], 0.0) if st['frozen'] is not None else get_frame()
            if robot:
                robot.poll()
            if age > STALE_S:                          # frozen / lost stream: never judge an old picture
                if camera_ok:
                    camera_ok = False
                    print(time.strftime('%H:%M:%S'), 'NO CAMERA (frame age %.1f s)' % age)
                    stab = Stabilizer(cfg['stable_s'])
                    if robot:
                        robot.send('B')
                if frame is not None:
                    img = cv2.resize(frame, None, fx=SCALE, fy=SCALE)
                    cv2.putText(img, 'NO CAMERA %.0fs' % age, (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
                    cv2.imshow('vest check', img)
                if cv2.waitKey(30) & 0xFF in (ord('q'), 27):
                    break
                continue
            if not camera_ok:
                camera_ok = True
                print(time.strftime('%H:%M:%S'), 'camera back')
            if frame is None:
                if cv2.waitKey(30) & 0xFF in (ord('q'), 27):
                    break
                continue
            st['hsv'] = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            res = st['res'] = evaluate(frame, cfg, args.zones)
            if stab.update(res['verdict'], time.monotonic()):
                print(time.strftime('%H:%M:%S'), stab.value, ' '.join(res['reasons']), res.get('fills', ''))
                if robot:
                    robot.send(LAMP[stab.value])
            shown = draw(frame, res, stab.value, st['sel'], st['mode'])
            cv2.imshow('vest check', shown)
            if 'mask' in res:
                cv2.imshow('vest colour mask', res['mask'])
            if time.monotonic() - last_snap > 2:
                last_snap = time.monotonic()
                cv2.imwrite(os.path.join(snap_dir, time.strftime('%H%M%S_raw.jpg')), frame)
                cv2.imwrite(os.path.join(snap_dir, time.strftime('%H%M%S_view.jpg')), shown)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            if k in (ord('a'), ord('b'), ord('c'), ord('d')):
                st['sel'] = chr(k).upper()
            elif k in (ord('t'), ord('v'), ord('r')):
                st['mode'] = {ord('t'): 'tool', ord('v'): 'vest', ord('r'): 'rect'}[k]
            elif k == ord('f'):
                st['frozen'] = None if st['frozen'] is not None else frame.copy()
            elif k == ord('s'):
                json.dump(cfg, open(CONFIG_PATH, 'w', encoding='utf-8'), indent=1)
                print('saved', CONFIG_PATH)
    finally:
        if robot:
            robot.send('O')
            time.sleep(0.1)
            robot.close()
        if close:
            close()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
