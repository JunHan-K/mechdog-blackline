"""STEP 4a - create MechDog() once (NO move() call) and measure heap / time / I2C errors.

    py tools\\init_test.py [--watch 10]

The robot may stand up (servos energise). The object is kept as global `dog` on the board,
so later RAM-run code must reuse it instead of creating a second MechDog().
"""
import argparse
import sys

from mdlink import Board, new_log_path

HEAP = r'''
import gc, time
try:
    import esp32
    def idf():
        return sum(h[1] for h in esp32.idf_heap_info(esp32.HEAP_DATA))
except ImportError:
    def idf():
        return -1
def report(tag):
    gc.collect()
    print(tag, 'GC_FREE', gc.mem_free(), 'IDF_FREE', idf())
'''

CREATE = r'''
report('BEFORE')
if 'dog' in globals():
    print('DOG_ALREADY_EXISTS')
else:
    t = time.ticks_ms()
    from HW_MechDog import MechDog
    dog = MechDog()
    print('INIT_MS', time.ticks_diff(time.ticks_ms(), t))
report('AFTER_INIT')
print('DOG_DIR', [n for n in dir(dog) if not n.startswith('__')])
import HW_MechDog
print('MODULE_DIR', [n for n in dir(HW_MechDog) if not n.startswith('__')])
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='COM9')
    ap.add_argument('--watch', type=float, default=10.0)
    a = ap.parse_args()
    log = new_log_path('init')
    b = Board(a.port, log_path=log)
    try:
        b.enter_raw()
        out, err = b.exec_raw(HEAP + CREATE, timeout=20)
        print(out, err)
        print(f'Watching {a.watch:.0f}s for background errors (nothing sent)...')
        b.exit_raw()
        data = b.read_for(a.watch)
        if data.strip():
            print(data.decode('utf-8', 'replace')[-800:])
        b.enter_raw()
        out, err = b.exec_raw("report('AFTER_WATCH')")
        print(out, err)
        b.exit_raw()
        print('malloc errors seen:', b.malloc_errors)
    finally:
        b.close()
        print('Log:', log)


if __name__ == '__main__':
    sys.exit(main())
