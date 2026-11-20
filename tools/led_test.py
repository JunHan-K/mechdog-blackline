"""Head lamp test: green 2 s -> red 2 s -> off. Receiver in RAM, MechDog NOT initialised (no motors).
    py tools\\led_test.py [--port COM9]
"""
import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, 'tools'))
from line_follow_pc_v2 import Robot  # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument('--port', default='COM9')
args = ap.parse_args()
robot = Robot(args.port)
ok = True
try:
    robot.start(init=False)
    for c, name in (('G', 'green'), ('E', 'red'), ('O', 'off')):
        robot.send(c)
        got = robot.wait('LED ' + c, 2)
        print(name, 'OK' if got else 'NO REPLY', robot.buf[-120:] if not got else '')
        ok &= got
        time.sleep(2 if c != 'O' else 0.2)
finally:
    robot.close()
print('log:', robot.log_path)
sys.exit(0 if ok else 1)
