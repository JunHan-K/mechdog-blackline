"""Black tape detector tuned for the MechDog ESP32-S3 camera (2026-11-20 measurements).

Why not a plain "V < 70" threshold: on the real floor the camera shows the black tape as
dark-grey/bluish (V ~ 100) while the floor has a strong lighting gradient (V 90..240), so
no single brightness threshold separates them. Instead:

  channel T = V + COLOR_W * (R - B)      tape: dark AND bluish -> low T; warm floor -> high T
  For several horizontal strips: column profile of T -> find a falling edge (floor->tape)
  and a rising edge (tape->floor) with tape-like width; the tape must be darker than the
  floor on both sides. One-sided (tape touching image border) is allowed with lower score.
  Candidates near the previous line position are preferred (tracking).

Positions are normalised: -1 = left edge, 0 = centre, +1 = right edge of the image.
"""
import cv2
import numpy as np

COLOR_W = 1.5            # weight of (R-B); set 0 for pure brightness
STRIP_ROWS = (0.45, 1.0)  # use the bottom 55 % of the image
N_STRIPS = 6
EDGE_K = 8               # gradient half-width in px (for 320 px wide frames)
MIN_EDGE = 8             # minimum edge strength (T units)
MIN_CONTRAST = 10        # floor must be this much brighter than tape (each side)
WIDTH_RANGE = (0.06, 0.6)  # tape width as fraction of image width
TRACK_W = 40             # score penalty per unit distance from previous position


def _strip_line(prof, prefer_x):
    w = prof.size
    p = np.convolve(prof, np.ones(5) / 5, mode='same')
    k = max(3, int(EDGE_K * w / 320))
    g = np.zeros(w)
    g[k:w - k] = p[2 * k:] - p[:w - 2 * k]
    lefts = [x for x in range(k, w - k) if g[x] < -MIN_EDGE and g[x] == g[max(0, x - 6):x + 7].min()]
    rights = [x for x in range(k, w - k) if g[x] > MIN_EDGE and g[x] == g[max(0, x - 6):x + 7].max()]
    best = None
    side = max(6, w // 16)
    for L in [None] + lefts:
        for R in rights + [None]:
            if L is None and R is None:
                continue
            l0, r0 = (L if L is not None else 0), (R if R is not None else w - 1)
            if not (WIDTH_RANGE[0] * w <= r0 - l0 <= WIDTH_RANGE[1] * w):
                continue
            inside = np.median(p[l0 + 3:max(l0 + 4, r0 - 2)])
            outs = []
            if L is not None:
                outs.append(np.median(p[max(0, L - side):L]))
            if R is not None:
                outs.append(np.median(p[R + 1:R + 1 + side]))
            c = min(o - inside for o in outs)
            if c < MIN_CONTRAST:
                continue
            x = (l0 + r0) / 2 / (w / 2) - 1.0
            score = (-g[L] if L is not None else 0) + (g[R] if R is not None else 0) \
                + min(c, 30) - TRACK_W * abs(x - prefer_x)
            if L is None or R is None:
                score -= 15
            if best is None or score > best[0]:
                best = (score, x, l0, r0)
    return best


class LineDetector:
    def __init__(self):
        self.last_x = 0.0

    def detect(self, frame):
        """Returns dict(found, x_near, x_far, error, overlay, mask)."""
        h, w = frame.shape[:2]
        f = frame.astype(np.float32)
        v = f.max(axis=2)                                  # HSV V
        t = v + COLOR_W * (f[:, :, 2] - f[:, :, 0])
        y_top, y_bot = int(h * STRIP_ROWS[0]), int(h * STRIP_ROWS[1])
        edges = np.linspace(y_top, y_bot, N_STRIPS + 1).astype(int)
        out = frame.copy()
        mask = np.zeros((h - y_top, w), np.uint8)
        pts = []                                           # bottom strip first
        prefer = self.last_x
        for i in range(N_STRIPS - 1, -1, -1):
            y0, y1 = edges[i], edges[i + 1]
            best = _strip_line(np.median(t[y0:y1], axis=0), prefer)
            if best:
                _, x, l0, r0 = best
                pts.append((x, (y0 + y1) / 2))
                prefer = x
                mask[y0 - y_top:y1 - y_top, int(l0):int(r0) + 1] = 255
                cv2.rectangle(out, (int(l0), y0), (int(r0), y1 - 1), (0, 255, 0), 1)
                cv2.circle(out, (int((x + 1) * w / 2), int((y0 + y1) / 2)), 3, (0, 0, 255), -1)
        res = {'found': False, 'x_near': None, 'x_far': None, 'error': None, 'strips': len(pts)}
        if len(pts) >= 2:
            near = [x for x, _ in pts[:N_STRIPS // 2]] or [pts[0][0]]
            far = [x for x, _ in pts[N_STRIPS // 2:]] or near
            res['found'] = True
            res['x_near'] = float(np.median(near))
            res['x_far'] = float(np.median(far))
            res['error'] = 0.6 * res['x_near'] + 0.4 * res['x_far']
            self.last_x = res['x_near']
        cv2.line(out, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
        cv2.line(out, (0, y_top), (w, y_top), (200, 200, 0), 1)
        res['overlay'] = out
        res['mask'] = mask
        return res
