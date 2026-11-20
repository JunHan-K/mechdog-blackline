"""STEP 3 - run robot/pong_test.py from RAM (no flash write, no motor import) and check P -> PONG.

Usage:  py tools\\run_pong_test.py [--port COM9]
"""
import argparse
import os
import sys
import time

from mdlink import Board, BoardError, new_log_path

HERE = os.path.dirname(os.path.abspath(__file__))
CODE_PATH = os.path.join(HERE, '..', 'robot', 'pong_test.py')


def expect_line(b, token, timeout):
    t0 = time.monotonic()
    ok, data = b.read_until(token.encode(), timeout)
    return ok, (time.monotonic() - t0) * 1000, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM9')
    args = ap.parse_args()
    log_path = new_log_path('pong')
    b = Board(args.port, log_path=log_path)
    results = []
    try:
        b.enter_raw()
        b.send_code(open(CODE_PATH, 'rb').read())
        ok, _, data = expect_line(b, 'PONGTEST_READY', 3)
        results.append(('receiver started', ok))
        if not ok:
            print('No PONGTEST_READY:', data[-300:])
            return 1

        lat = []
        for _ in range(20):
            b.write(b'P')
            ok, ms, data = expect_line(b, 'PONG\r\n', 1.0)
            if not ok:
                break
            lat.append(ms)
        results.append((f'20 single P -> PONG ({len(lat)}/20)', len(lat) == 20))
        if lat:
            print(f'latency ms: min {min(lat):.0f} avg {sum(lat)/len(lat):.0f} max {max(lat):.0f}')

        b.write(b'PPPPP')
        data = b.read_for(1.0)
        results.append((f'burst PPPPP -> {data.count(b"PONG")} PONG', data.count(b'PONG') == 5))
        echo = b'PPPPP' in data or b'>>>' in data
        results.append(('no REPL echo / prompt', not echo))

        b.write(b'Z')
        ok, _, _ = expect_line(b, 'UNK 90', 1.0)
        results.append(('unknown byte reported', ok))

        ok, _, _ = expect_line(b, 'ALIVE', 3.0)
        results.append(('ALIVE heartbeat (loop not stuck)', ok))

        b.write(b'X')
        ok, _, _ = expect_line(b, 'PONGTEST_EXIT', 1.5)
        results.append(('clean exit with X', ok))
        b.read_until(b'>', 2.0)
        b.exit_raw()
    except BoardError as e:
        print('BOARD ERROR:', e)
        try:
            b.interrupt()
        except Exception:
            pass
        return 1
    finally:
        print()
        for name, ok in results:
            print(('PASS ' if ok else 'FAIL ') + name)
        print('malloc errors seen:', b.malloc_errors)
        b.close()
        print('Log:', log_path)
    return 0 if all(ok for _, ok in results) else 1


if __name__ == '__main__':
    sys.exit(main())
