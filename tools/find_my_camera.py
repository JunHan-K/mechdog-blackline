"""Grab one frame from every HW_ESP32S3CAM access point nearby -> logs\\cam_<bssid>.jpg,
then return to the previous Wi-Fi. Look at the pictures, then save yours:
    py tools\\find_my_camera.py
    py tools\\find_my_camera.py --set 8c:bf:ea:87:f1:29     # writes cam_bssid.txt (local, gitignored)
"""
import os
import subprocess
import sys

import cv2

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(HERE, 'tools'))
import wifi_bssid  # noqa: E402

BSSID_FILE = os.path.join(HERE, 'cam_bssid.txt')
URL = 'http://192.168.5.1:81/stream'


def grab(timeout=8):
    for _ in range(int(timeout / 0.5)):
        if 'TTL=' in subprocess.run(['ping', '-n', '1', '-w', '500', '192.168.5.1'],
                                    capture_output=True, text=True).stdout:
            break
    cap = cv2.VideoCapture(URL)
    frame = None
    for _ in range(5):                       # skip first frames
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    return frame


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--set':
        open(BSSID_FILE, 'w').write(wifi_bssid.norm(sys.argv[2]) + '\n')
        print('saved', BSSID_FILE)
        return 0
    prev = wifi_bssid.current()[0]
    aps = wifi_bssid.list_bssids()
    print('cameras:', aps)
    try:
        for b, sig in aps:
            ok = wifi_bssid.connect(b)
            f = grab() if ok else None
            path = os.path.join(HERE, 'logs', 'cam_%s.jpg' % b.replace(':', ''))
            if f is not None:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                cv2.imwrite(path, f)
            print(b, sig, '% connected' if ok else '% CONNECT FAILED', '->', path if f is not None else 'no frame')
    finally:
        if prev and prev != wifi_bssid.CAM_SSID:
            subprocess.run(['netsh', 'wlan', 'connect', 'name=' + prev, 'ssid=' + prev], capture_output=True)
            print('restored Wi-Fi', prev)
    return 0


if __name__ == '__main__':
    sys.exit(main())
