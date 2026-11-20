"""Factory floor hazard check with MechDog head-lamp feedback. Rule based (OpenCV only).

  black obstacle alone                         -> RED   (hazard, not secured)
  black obstacle surrounded by >= 4 white cones -> GREEN (secured)
  no obstacle / camera lost                    -> BLUE  (idle)

    py hazard_check.py                   # ESP32-S3 camera + robot lamp (joins camera Wi-Fi once, never fights a switch)
    py hazard_check.py --no-robot        # screen only
    py hazard_check.py --source img.jpg  # still image      --selftest   synthetic test

Setup (fixed camera, recommended): clear the table, press b once -> hazard_bg.png (empty-scene background).
  Re-press b whenever the camera or the lighting moves.

Background mode (used when hazard_bg.png exists):
  changed  = |frame - background| > DIFF_T (auto-exposure compensated)
  obstacle = biggest changed + dark blob standing on the table (bottom below the table's back edge)
  cones    = changed + white pixels; overlapping cups (side view) are split by their bottom rims:
             each cup touches the table at its own height -> a jump in the bottom edge = next cup.
             White highlights on the obstacle itself and anything behind the table (people, wall) are ignored.
  SECURED  = >= MIN_CONES cones with at least one left and one right of the obstacle.
  Replay check on the 2026-11-20 session: 13/14 labelled frames correct (miss = cups still being placed).

Fallback without background: dark blob = obstacle, white blobs near its base = cones. Fails on a dark
  table / dark background (the obstacle merges with them) -> use the background mode.
Side camera: a cone completely hidden behind the obstacle or another cone cannot be counted.
Keys: b save empty-scene background, f freeze, q/ESC quit."""
import argparse
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, 'tools'))
from vest_check import Stabilizer, open_source, SCALE, STALE_S  # noqa: E402

DARK_V = 70          # obstacle: HSV V at most this
MIN_OBST = 0.03      # obstacle area / image area
WHITE_S = 55         # cone: saturation at most this
WHITE_V = 170        # cone: brightness at least this
MIN_CONE = 0.0012    # cone area / image area
CONE_REACH = 2.0     # cones within this many obstacle widths left/right of the obstacle
MIN_CONES = 4
STABLE_S = 1.5
LAMP = {'SECURED': 'G', 'HAZARD': 'E', 'CLEAR': 'B'}
COL = {'SECURED': (0, 200, 0), 'HAZARD': (0, 0, 255), 'CLEAR': (255, 120, 0)}


BG_PATH = os.path.join(HERE, 'hazard_bg.png')   # empty scene for the fixed camera view (key b), gitignored
DIFF_T = 32          # per-pixel change vs background (max over B,G,R after blur)
NEW_DARK_V = 100     # changed + this dark -> obstacle
MIN_OBST_BG = 0.012  # obstacle area / image (background mode)
MIN_CONE_BG = 0.004  # cone area / image (background mode)


def _blobs(mask, min_area):
    n, _, st, _ = cv2.connectedComponentsWithStats(mask)
    return [tuple(int(v) for v in s[:4]) for s in sorted(st[1:], key=lambda s: -s[4]) if s[4] >= min_area]


def split_by_base(mask, blobs, hz, jump=15, min_w=14):
    """Side view: overlapping cups merge into one white blob, but each cup touches the table at its own
    height. Walk the blob's bottom edge column by column; a jump > `jump` px starts a new cup.
    Columns whose bottom is above the table edge (wall, people) are ignored."""
    out = []
    for x, y, w, h in blobs:
        sub = mask[y:y + h, x:x + w] > 0
        seg = None
        for i in range(w):
            rows = np.where(sub[:, i])[0]
            bot = y + rows.max() if rows.size else -1
            if bot < hz:
                if seg:
                    out.append(seg)
                seg = None
                continue
            if seg and abs(bot - seg[3]) <= jump:
                seg = [seg[0], seg[1], i + x - seg[0] + 1, bot]
            else:
                if seg:
                    out.append(seg)
                seg = [x + i, y, 1, bot]
        if seg:
            out.append(seg)
    # (x, y, w, h) boxes from top of blob to the cup's base
    return [(s[0], s[1], s[2], s[3] - s[1]) for s in out if s[2] >= min_w]


_edge_cache = {}


def table_edge(bg):
    """Row of the table's back edge in the empty background: per column, the lowest bright (wall) pixel
    above the dark table; median over columns. Falls back to 40 % of the height."""
    key = id(bg)
    if key not in _edge_cache:
        v = cv2.medianBlur(cv2.cvtColor(bg, cv2.COLOR_BGR2HSV)[:, :, 2], 5)
        H = v.shape[0]
        rows = []
        for x in range(0, v.shape[1], 4):
            col = v[:, x]
            # first row (from the top) where bright wall ends and the dark table starts (door columns give none)
            for y in range(5, H - 15):
                if col[y - 5:y].min() > 140 and col[y:y + 15].max() < 100:   # bright wall right above, dark table below
                    rows.append(y)
                    break
        _edge_cache.clear()
        _edge_cache[key] = int(np.median(rows)) if rows else int(0.4 * H)
    return _edge_cache[key]


def evaluate_bg(frame, bg):
    """Fixed camera + empty-scene background: new dark blob = obstacle, new white blobs = cones."""
    H, W = frame.shape[:2]
    a = cv2.GaussianBlur(frame, (5, 5), 0)
    b = cv2.GaussianBlur(bg, (5, 5), 0)
    # cancel auto-exposure drift (white cups darken the picture): median per-pixel ratio on mid-tones
    ga, gb = a.mean(axis=2), b.mean(axis=2)
    sel = (gb > 40) & (gb < 220)
    ratio = ga[sel] / gb[sel] if sel.any() else np.array([1.0])
    gain = float(np.median(ratio[(ratio > 0.5) & (ratio < 2.0)])) if ((ratio > 0.5) & (ratio < 2.0)).any() else 1.0
    b = np.clip(b.astype(np.float32) * gain, 0, 255).astype(np.uint8)
    changed = (cv2.absdiff(a, b).max(axis=2) > DIFF_T).astype(np.uint8) * 255
    k = np.ones((5, 5), np.uint8)
    changed = cv2.morphologyEx(cv2.morphologyEx(changed, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    white = cv2.inRange(hsv, (0, 0, WHITE_V - 10), (179, WHITE_S + 5, 255))
    dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, NEW_DARK_V))
    obst = cv2.morphologyEx(changed & dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    cones = cv2.morphologyEx(changed & white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    res = {'obstacle': None, 'cones': [], 'verdict': 'CLEAR', 'why': '', 'mask_dark': obst, 'mask_white': cones}
    # objects must STAND ON the table: blob bottom below the table's back edge (+ margin).
    # Removes people / walls / doors behind the table and exposure changes up there.
    hz = table_edge(bg) + 15
    res['horizon'] = hz
    ob = [o for o in _blobs(obst, MIN_OBST_BG * H * W) if o[1] + o[3] >= hz]
    res['cones'] = split_by_base(cones, _blobs(cones, MIN_CONE_BG * H * W), hz)
    if not ob:
        return res
    res['obstacle'] = ob[0]
    ox, oy, ow, oh = ob[0]
    ocx = ox + ow / 2
    # white highlights ON the obstacle's plastic are not cones
    res['cones'] = [c for c in res['cones'] if not (ox <= c[0] and c[0] + c[2] <= ox + ow and c[1] + c[3] <= oy + oh + 5)]
    left = sum(1 for x, y, w, h in res['cones'] if x + w / 2 < ocx)
    right = len(res['cones']) - left
    if len(res['cones']) >= MIN_CONES and left and right:
        res['verdict'] = 'SECURED'
    else:
        res['verdict'] = 'HAZARD'
        res['why'] = f'cones {len(res["cones"])}/{MIN_CONES} (L{left} R{right})'
    return res

def evaluate(frame, bg=None):
    if bg is not None:
        return evaluate_bg(frame, bg)
    H, W = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    k = np.ones((5, 5), np.uint8)
    res = {'obstacle': None, 'cones': [], 'verdict': 'CLEAR', 'why': ''}

    dark = cv2.inRange(hsv, (0, 0, 0), (179, 255, DARK_V))
    dark = cv2.morphologyEx(cv2.morphologyEx(dark, cv2.MORPH_OPEN, k), cv2.MORPH_CLOSE, k)
    cnts, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = [c for c in cnts if cv2.contourArea(c) >= MIN_OBST * H * W]
    res['mask_dark'] = dark
    if not cnts:
        return res
    ox, oy, ow, oh = cv2.boundingRect(max(cnts, key=cv2.contourArea))
    res['obstacle'] = (ox, oy, ow, oh)
    ocx, obot = ox + ow / 2, oy + oh

    white = cv2.inRange(hsv, (0, 0, WHITE_V), (179, WHITE_S, 255))
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    res['mask_white'] = white
    wc, _ = cv2.findContours(white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in wc:
        a = cv2.contourArea(c)
        x, y, w, h = cv2.boundingRect(c)
        if a < MIN_CONE * H * W or a > 0.6 * ow * oh or w > ow:   # cones are smaller than the obstacle
            continue
        if x <= 1 or y <= 1 or x + w >= W - 1 or y + h >= H - 1:  # paper / glare / wall run off the image
            continue
        base = y + h
        # base on the floor around the obstacle: from mid-obstacle (cones behind it) to below it (in front)
        if not (oy + 0.1 * oh <= base <= obot + 0.8 * oh):
            continue
        if not (ox - CONE_REACH * ow <= x + w / 2 <= ox + ow + CONE_REACH * ow):
            continue
        cx, cy = x + w / 2, y + h / 2
        if ox <= cx <= ox + ow and oy <= cy <= obot and base < obot:   # tape / label ON the obstacle
            continue
        res['cones'].append((x, y, w, h))
    left = sum(1 for x, y, w, h in res['cones'] if x + w / 2 < ocx)
    right = len(res['cones']) - left
    if len(res['cones']) >= MIN_CONES and left and right:
        res['verdict'] = 'SECURED'
    else:
        res['verdict'] = 'HAZARD'
        res['why'] = f'cones {len(res["cones"])}/{MIN_CONES} (L{left} R{right})'
    return res


def draw(frame, res, shown):
    out = cv2.resize(frame, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_NEAREST)
    if res['obstacle']:
        x, y, w, h = [v * SCALE for v in res['obstacle']]
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 0, 255), 2)
    if res.get('horizon'):
        cv2.line(out, (0, res['horizon'] * SCALE), (out.shape[1], res['horizon'] * SCALE), (255, 255, 0), 1)
    for x, y, w, h in res['cones']:
        x, y, w, h = x * SCALE, y * SCALE, w * SCALE, h * SCALE
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.putText(out, f'{shown or "..."}  {res["why"]}', (8, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                COL.get(shown, (200, 200, 200)), 2)
    return out


def selftest():
    def scene(n_cones):
        img = np.full((240, 320, 3), (60, 100, 140), np.uint8)              # brown floor
        cv2.ellipse(img, (160, 150), (55, 40), 0, 0, 360, (20, 20, 20), -1)  # black obstacle
        spots = [(70, 190), (250, 190), (80, 130), (240, 130)]
        for cx, cy in spots[:n_cones]:
            pts = np.array([[cx - 10, cy], [cx + 10, cy], [cx, cy - 28]])
            cv2.fillPoly(img, [pts], (235, 235, 235))
        return img
    cases = [(0, 'HAZARD'), (2, 'HAZARD'), (4, 'SECURED')]
    ok = True
    for n, want in cases:
        r = evaluate(scene(n))
        ok &= r['verdict'] == want
        print(f'{n} cones -> {r["verdict"]:8s} {r["why"]:18s} {"ok" if r["verdict"] == want else "MISMATCH"}')
    r = evaluate(np.full((240, 320, 3), (60, 100, 140), np.uint8))
    print('empty    ->', r['verdict'], 'ok' if r['verdict'] == 'CLEAR' else 'MISMATCH')
    return 0 if ok and r['verdict'] == 'CLEAR' else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', default='http://192.168.5.1:81/stream')
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--no-robot', action='store_true')
    ap.add_argument('--selftest', action='store_true')
    args = ap.parse_args()
    if args.selftest:
        return selftest()

    stab = Stabilizer(STABLE_S)
    robot = close = None
    frozen = None
    camera_ok = True
    bg = cv2.imread(BG_PATH) if os.path.exists(BG_PATH) else None
    print('background:', BG_PATH if bg is not None else 'NONE - clear the table and press b')
    try:
        if not args.no_robot:
            from line_follow_pc_v2 import Robot
            robot = Robot(args.port)
            robot.start(init=False)          # lamp only, robot does not stand up / move
            robot.send('B')
            print('Robot lamp ready (blue = idle).')
        get_frame, close = open_source(args.source, print)
        snap_dir = os.path.join(HERE, 'logs', time.strftime('hazard_%Y%m%d_%H%M%S'))
        os.makedirs(snap_dir, exist_ok=True)
        last_snap = 0.0
        while True:
            frame, age = (frozen, 0.0) if frozen is not None else get_frame()
            if robot:
                robot.poll()
            if age > STALE_S:
                if camera_ok:
                    camera_ok = False
                    print(time.strftime('%H:%M:%S'), 'NO CAMERA')
                    stab = Stabilizer(STABLE_S)
                    if robot:
                        robot.send('B')
                if cv2.waitKey(30) & 0xFF in (ord('q'), 27):
                    break
                continue
            camera_ok = True
            if bg is not None and bg.shape != frame.shape:
                bg = None
            res = evaluate(frame, bg)
            if stab.update(res['verdict'], time.monotonic()):
                print(time.strftime('%H:%M:%S'), stab.value, res['why'])
                if robot:
                    robot.send(LAMP[stab.value])
            shown = draw(frame, res, stab.value)
            if bg is None:
                cv2.putText(shown, 'no background: clear table, press b', (8, shown.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.imshow('hazard check', shown)
            if time.monotonic() - last_snap > 2:
                last_snap = time.monotonic()
                cv2.imwrite(os.path.join(snap_dir, time.strftime('%H%M%S_raw.jpg')), frame)
                cv2.imwrite(os.path.join(snap_dir, time.strftime('%H%M%S_view.jpg')), shown)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('b'):
                bg = frame.copy()
                cv2.imwrite(BG_PATH, bg)
                print(time.strftime('%H:%M:%S'), 'background saved', BG_PATH)
                stab = Stabilizer(STABLE_S)
            if k == ord('f'):
                frozen = None if frozen is not None else frame.copy()
    finally:
        if robot:
            robot.send('B')
            time.sleep(0.1)
            robot.close()
        if close:
            close()
        cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
