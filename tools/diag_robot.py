"""STEP 1 - read-only diagnosis of the MechDog main ESP32 (no motor commands, no file writes on robot).

Usage (from the project folder, close every other program using COM9 first):
    py tools\\diag_robot.py                  # passive listen + read-only info + copy robot files to PC
    py tools\\diag_robot.py --listen-only    # only listen; sends nothing at all
    py tools\\diag_robot.py --no-backup      # info only, no file copy

What it does:
1. Opens COM9 with DTR/RTS released, listens passively (default 6 s) and records whether the board
   reset, printed a boot log, printed ROBOT_READY, or is flooding 'i2c ... malloc error'.
2. If a malloc-error flood is seen, it STOPS here and sends nothing.
3. Otherwise sends Ctrl-C (stops a foreground program), enters raw REPL and runs read-only snippets:
   file list, which modules/globals are loaded (= what /main.py did at boot), heap numbers, reset cause.
4. Copies every robot file (<= --max-bytes) to robot_backup\\<timestamp>\\ on the PC (new folder each run).
5. Leaves the board at the normal >>> prompt. It does NOT soft-reset or reboot.
"""
import argparse
import base64
import os
import sys
import time

from mdlink import Board, BoardError, new_log_path

INFO_CODE = r'''
import sys, gc, os
_g = sorted(k for k in globals().keys() if not k.startswith('__'))
print('GLOBALS', _g)
print('MODULES', sorted(sys.modules.keys()))
def _walk(p):
    for e in os.ilistdir(p):
        full = (p.rstrip('/') + '/' + e[0])
        if e[1] == 0x4000:
            print('D', full)
            _walk(full)
        else:
            sz = e[3] if len(e) > 3 else os.stat(full)[6]
            print('F', sz, full)
_walk('/')
gc.collect()
print('GC_FREE', gc.mem_free(), 'GC_ALLOC', gc.mem_alloc())
try:
    import esp32
    for h in esp32.idf_heap_info(esp32.HEAP_DATA):
        print('IDF_HEAP total,free,largest,minfree', h)
except Exception as e:
    print('IDF_HEAP_ERR', repr(e))
try:
    import machine
    print('RESET_CAUSE', machine.reset_cause(), 'FREQ', machine.freq())
except Exception as e:
    print('MACHINE_ERR', repr(e))
print('IMPL', sys.implementation, sys.platform)
try:
    st = os.statvfs('/')
    print('FS_FREE_BYTES', st[0] * st[3])
except Exception as e:
    print('STATVFS_ERR', repr(e))
'''

DUMP_CODE = r'''
import ubinascii
f = open(%r, 'rb')
n = 0
while True:
    b = f.read(384)
    if not b:
        break
    n += len(b)
    print(ubinascii.b2a_base64(b).decode().strip())
f.close()
print('END', n)
'''


def classify_listen(data):
    notes = []
    if b'rst:' in data or b'ets ' in data or b'MicroPython v' in data:
        notes.append('BOOT_LOG_SEEN (opening the port may have reset the board, or it rebooted by itself)')
    if b'ROBOT_READY' in data:
        notes.append('ROBOT_READY seen -> the line-tracing main.py ran at boot')
    if b'wifi connecting' in data:
        notes.append("'wifi connecting' seen -> Hiwonder original APP main.py (BLE+WiFi) ran at boot")
    if b'Traceback' in data:
        notes.append('Traceback seen during listen (see log)')
    if b'>>>' in data:
        notes.append('>>> prompt seen -> REPL is active (bytes sent now are typed into the REPL)')
    return notes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--listen', type=float, default=6.0)
    ap.add_argument('--listen-only', action='store_true')
    ap.add_argument('--no-backup', action='store_true')
    ap.add_argument('--max-bytes', type=int, default=200_000)
    args = ap.parse_args()

    log_path = new_log_path('diag')
    print('Raw serial log ->', log_path)
    b = Board(args.port, log_path=log_path)
    try:
        print(f'[1] Passive listen {args.listen:.0f}s (nothing is sent)...')
        data = b.read_for(args.listen)
        print(f'    received {len(data)} bytes, malloc errors: {b.malloc_errors}')
        for n in classify_listen(data):
            print('    -', n)
        tail = data[-400:].decode('utf-8', 'replace')
        if tail.strip():
            print('    --- last output ---')
            print('    ' + tail.replace('\n', '\n    '))
        if b.malloc_errors >= 3:
            print('\n!! i2c malloc-error flood is active. Sending nothing. Power the robot OFF,')
            print('   wait 5 s, power ON, then run this again (send me the log either way).')
            return 2
        if args.listen_only:
            return 0

        print('[2] Ctrl-C + raw REPL (stops a foreground program; does not reboot)...')
        b.enter_raw()
        out, err = b.exec_raw(INFO_CODE, timeout=20)
        print(out)
        if err.strip():
            print('STDERR:', err)

        files = []
        for line in out.splitlines():
            parts = line.split(' ', 2)
            if parts[0] == 'F' and len(parts) == 3:
                files.append((int(parts[1]), parts[2].strip()))

        if not args.no_backup and files:
            dest = os.path.join('robot_backup', time.strftime('%Y%m%d_%H%M%S'))
            os.makedirs(dest, exist_ok=False)   # new folder every run -> never overwrites
            print(f'[3] Copying robot files to {dest} (read-only on robot)')
            for size, path in files:
                if size > args.max_bytes:
                    print(f'    SKIP (too big) {path} {size} B')
                    continue
                timeout = 5 + size * 1.5 / 8000
                o, e = b.exec_raw(DUMP_CODE % path, timeout=timeout)
                lines = o.split()
                if e.strip() or 'END' not in lines:
                    print(f'    FAIL {path}: {e.strip()[-120:]}')
                    continue
                raw = base64.b64decode(''.join(lines[:lines.index('END')]))
                expected = int(lines[lines.index('END') + 1])
                local = os.path.join(dest, path.lstrip('/').replace('/', os.sep))
                os.makedirs(os.path.dirname(local) or dest, exist_ok=True)
                with open(local, 'wb') as f:
                    f.write(raw)
                status = 'OK ' if len(raw) == expected == size else 'SIZE_MISMATCH'
                print(f'    {status} {path} {len(raw)} B')
        print(f'\nmalloc errors during whole session: {b.malloc_errors}')
        b.exit_raw()
        print('Done. Board left at normal >>> prompt (no reboot).')
        return 0
    except BoardError as e:
        print('BOARD ERROR:', e)
        return 1
    finally:
        b.close()
        print('Log:', log_path)


if __name__ == '__main__':
    sys.exit(main())
