"""PC-side helper for talking to the MechDog MicroPython board over USB serial.

Design rules:
- Open the port WITHOUT toggling DTR/RTS so the ESP32 is (ideally) not reset.
- Never write to the robot filesystem from this module.
- Raw REPL (Ctrl-A) is used to run code from RAM, the same way mpremote/pyboard.py do.
- Everything received is appended to a log file for later inspection.
"""
import os
import time

import serial

MALLOC_MARK = b'malloc error'


class BoardError(RuntimeError):
    pass


class Board:
    def __init__(self, port='COM9', baud=115200, log_path=None):
        s = serial.Serial()
        s.port = port
        s.baudrate = baud
        s.timeout = 0.05
        s.write_timeout = 2
        # Keep DTR/RTS released before open -> avoids the auto-reset circuit on most ESP32 boards.
        s.dtr = False
        s.rts = False
        s.open()
        self.s = s
        self.log = open(log_path, 'ab') if log_path else None
        self.malloc_errors = 0
        self._log_mark(f'OPEN {port} {baud}')

    # ---------------- low level ----------------
    def _log_mark(self, text):
        if self.log:
            self.log.write(f'\n===== {time.strftime("%H:%M:%S")} {text} =====\n'.encode())
            self.log.flush()

    def _got(self, data):
        if data:
            self.malloc_errors += data.count(MALLOC_MARK)
            if self.log:
                self.log.write(data)
                self.log.flush()
        return data

    def write(self, data):
        if self.log:
            self.log.write(b'\n[PC->] ' + repr(data).encode() + b'\n')
        self.s.write(data)
        self.s.flush()

    def read_available(self):
        n = self.s.in_waiting
        return self._got(self.s.read(n or 1))

    def read_for(self, seconds):
        end = time.monotonic() + seconds
        buf = bytearray()
        while time.monotonic() < end:
            buf += self.read_available()
        return bytes(buf)

    def read_until(self, token, timeout):
        """Return (found, data). data includes the token when found."""
        end = time.monotonic() + timeout
        buf = bytearray()
        while time.monotonic() < end:
            chunk = self.read_available()
            if chunk:
                buf += chunk
                i = buf.find(token)
                if i >= 0:
                    return True, bytes(buf)
        return False, bytes(buf)

    def close(self):
        try:
            self._log_mark('CLOSE')
            self.s.close()
        finally:
            if self.log:
                self.log.close()

    # ---------------- REPL helpers ----------------
    def interrupt(self):
        """Ctrl-C twice. Stops a foreground program (our receivers stop motors on exit)."""
        self.write(b'\r\x03')
        time.sleep(0.1)
        self.write(b'\x03')
        return self.read_for(0.6)

    def enter_raw(self):
        self.interrupt()
        for _ in range(3):
            self.s.reset_input_buffer()
            self.write(b'\r\x01')
            ok, data = self.read_until(b'raw REPL; CTRL-B to exit\r\n>', 2.0)
            if ok:
                return
            time.sleep(0.3)
        raise BoardError('Could not enter raw REPL. Last data: %r' % data[-200:])

    def exit_raw(self):
        """Back to the normal >>> REPL. Does NOT soft-reset (so /main.py is not re-run)."""
        self.write(b'\x02')
        self.read_for(0.3)

    def _read_exact(self, n, timeout=2.0):
        end = time.monotonic() + timeout
        buf = bytearray()
        while len(buf) < n and time.monotonic() < end:
            buf += self._got(self.s.read(n - len(buf)))
        return bytes(buf)

    def send_code(self, code):
        """Send code in raw REPL and start it (flow-controlled raw-paste mode, like mpremote).

        Returns any program output already received. Falls back to slow chunked
        raw mode if the firmware does not support raw-paste.
        """
        if isinstance(code, str):
            code = code.encode()
        code = code.replace(b'\r\n', b'\n')
        self.write(b'\x05A\x01')
        resp = self._read_exact(2)
        if resp == b'R\x01':
            window = int.from_bytes(self._read_exact(2), 'little')
            remain = window
            i = 0
            while i < len(code):
                while remain == 0 or self.s.in_waiting:
                    c = self._read_exact(1)
                    if c == b'\x01':
                        remain += window
                    elif c == b'\x04':          # board aborted the paste
                        self.write(b'\x04')
                        raise BoardError('Board aborted raw-paste')
                    elif not c:
                        raise BoardError('Raw-paste flow control timeout')
                    else:
                        raise BoardError('Unexpected raw-paste byte %r' % c)
                chunk = code[i:i + remain]
                self.write(chunk)
                remain -= len(chunk)
                i += len(chunk)
            self.write(b'\x04')
            ok, data = self.read_until(b'\x04', 5.0)   # end-of-data ack
            if not ok:
                raise BoardError('No raw-paste end ack: %r' % data[-200:])
            return data[data.find(b'\x04') + 1:]
        # Fallback: old raw REPL, small slow chunks
        self.read_until(b'>', 1.0)
        for i in range(0, len(code), 32):
            self.write(code[i:i + 32])
            time.sleep(0.03)
        self.write(b'\x04')
        ok, data = self.read_until(b'OK', 3.0)
        if not ok:
            raise BoardError('Board did not accept code: %r' % data[-200:])
        return data[data.find(b'OK') + 2:]

    def exec_raw(self, code, timeout=10.0):
        """Run a short snippet to completion; returns (stdout, stderr) as text."""
        head = self.send_code(code)
        ok, out = self.read_until(b'\x04', timeout)
        out = head + out
        if not ok and b'\x04' not in out:
            raise BoardError('Timeout waiting for snippet output: %r' % out[-300:])
        stdout, _, rest = out.partition(b'\x04')
        if b'\x04' not in rest:
            _, more = self.read_until(b'\x04', 3.0)
            rest += more
        stderr = rest.partition(b'\x04')[0]
        self.read_until(b'>', 1.0)
        return (stdout.decode('utf-8', 'replace'), stderr.decode('utf-8', 'replace'))


def new_log_path(prefix):
    os.makedirs('logs', exist_ok=True)
    return os.path.join('logs', f'{prefix}_{time.strftime("%Y%m%d_%H%M%S")}.log')
