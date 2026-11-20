"""Switch what the robot runs at boot. Never deletes Hiwonder files - only renames them.

IMPORTANT finding (2026-11-20): if /main.py is MISSING, the firmware regenerates its default
APP main.py (BLE+WiFi) at boot. So "disabling" means installing a tiny placeholder /main.py.

    py tools\\robot_main_switch.py --status
    py tools\\robot_main_switch.py --idle      # current /main.py -> saved name, install IDLE placeholder
    py tools\\robot_main_switch.py --restore   # remove IDLE placeholder, BLE app back to /main.py
    add --reset to machine.reset() the board once afterwards.

Saved names (on the robot):
    /main_hiwonder_ble_20261120.py          original BLE app main.py (9996 B)
    /main_hiwonder_wifi_default_20261120.py firmware-regenerated BLE+WiFi main.py (10617 B)
"""
import argparse
import sys

from mdlink import Board, new_log_path

MARK = 'MECHDOG_IDLE_MAIN'
PLACEHOLDER = (
    "# MECHDOG_IDLE_MAIN - placeholder so the firmware does not regenerate its default app.\n"
    "# Does nothing: the board stays at the REPL. Restore: py tools\\robot_main_switch.py --restore\n"
    "print('MAIN_IDLE')\n"
)

COMMON = r'''
import os
MARK = %r
def has(p):
    return p.lstrip('/') in os.listdir('/')
def size(p):
    return os.stat(p)[6]
def is_idle(p):
    with open(p) as f:
        return MARK in f.read(200)
print('BEFORE', sorted(os.listdir('/')))
'''

IDLE = r'''
res = 'NOTHING'
if has('/main.py') and is_idle('/main.py'):
    res = 'ALREADY_IDLE'
else:
    if has('/main.py'):
        s = size('/main.py')
        dst = {9996: '/main_hiwonder_ble_20261120.py', 10617: '/main_hiwonder_wifi_default_20261120.py'}.get(s)
        if dst is None:
            res = 'UNKNOWN_MAIN_SIZE %%d (not touched)' %% s
        elif has(dst):
            res = 'TARGET_EXISTS ' + dst
        else:
            os.rename('/main.py', dst)
    if not has('/main.py'):
        with open('/main.py', 'w') as f:
            f.write(%r)
        res = 'IDLE_INSTALLED %%d' %% size('/main.py')
print('RESULT', res)
print('AFTER', sorted(os.listdir('/')))
'''

RESTORE = r'''
SRC = '/main_hiwonder_ble_20261120.py'
if not has(SRC):
    res = 'NO_BLE_BACKUP'
elif has('/main.py') and not is_idle('/main.py'):
    res = 'MAIN_NOT_IDLE (not touched)'
else:
    if has('/main.py'):
        os.remove('/main.py')          # only ever removes our own placeholder
    os.rename(SRC, '/main.py')
    res = 'RESTORED %%d' %% size('/main.py')
print('RESULT', res)
print('AFTER', sorted(os.listdir('/')))
'''


def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--idle', action='store_true')
    g.add_argument('--restore', action='store_true')
    g.add_argument('--status', action='store_true')
    ap.add_argument('--reset', action='store_true', help='machine.reset() once afterwards')
    ap.add_argument('--port', default='COM9')
    a = ap.parse_args()
    b = Board(a.port, log_path=new_log_path('switch'))
    try:
        b.enter_raw()
        code = COMMON % MARK
        if a.idle:
            code += IDLE % PLACEHOLDER
        elif a.restore:
            code += RESTORE
        else:
            code += "print('MAIN_IS_IDLE', has('/main.py') and is_idle('/main.py'))\n"
        out, err = b.exec_raw(code)
        print(out, err)
        if a.reset and 'RESULT' in out and 'NOT' not in out.split('RESULT')[1].split('\n')[0]:
            print('Resetting board once...')
            b.send_code(b"import machine\nmachine.reset()\n")
            print(b.read_for(8).decode('utf-8', 'replace')[-700:])
        else:
            b.exit_raw()
    finally:
        b.close()


if __name__ == '__main__':
    sys.exit(main())
