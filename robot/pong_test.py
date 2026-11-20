# STEP 3 - minimal USB receiver for the MechDog main ESP32. NO motor library is imported.
# Run from RAM with tools\run_pong_test.py (nothing is written to the robot flash).
# Protocol: 'P' -> 'PONG', 'X' -> exit, CR/LF/space ignored, other bytes -> 'UNK <code>'.
import sys
import time
try:
    import uselect as select
except ImportError:
    import select

# Read raw bytes: text-mode read(1) can block waiting for UTF-8 continuation bytes.
src = sys.stdin.buffer if hasattr(sys.stdin, 'buffer') else sys.stdin
poller = select.poll()
poller.register(sys.stdin, select.POLLIN)

print('PONGTEST_READY')
count = 0
alive_at = time.ticks_ms()
running = True
while running:
    for _ in poller.poll(20):
        b = src.read(1)
        if not b:
            continue
        c = b[0] if isinstance(b, (bytes, bytearray)) else ord(b)
        count += 1
        if c == 0x50:          # 'P'
            print('PONG')
        elif c == 0x58:        # 'X'
            print('PONGTEST_EXIT', count)
            running = False
            break
        elif c in (10, 13, 32):
            pass
        else:
            print('UNK', c)
    if time.ticks_diff(time.ticks_ms(), alive_at) >= 2000:
        alive_at = time.ticks_ms()
        print('ALIVE', count)
