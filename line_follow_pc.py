"""Run on Windows Python, not on MechDog.
Press G to drive (5 seconds max), S/space to stop, Q to quit.
"""
import time
import cv2
import serial

CAMERA_URL = 'http://192.168.5.1:81/stream'
PORT = 'COM9'     # Change if the main MechDog ESP32 uses another COM port
BAUD = 115200
RUN_SECONDS = 5.0
HEARTBEAT_SECONDS = 0.12
INVERT_STEERING = False  # Change to True if robot steers away from the line


def wait_for(ser, text, timeout):
    deadline = time.monotonic() + timeout
    received = bytearray()
    while time.monotonic() < deadline:
        block = ser.read(ser.in_waiting or 1)
        if block:
            received.extend(block)
            if text.encode() in received:
                return True
            if len(received) > 2000:
                received = received[-1000:]
    print('Board response:', received.decode('utf-8', errors='replace')[-250:])
    return False


def connect_robot():
    ser = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.3)
    # Serial opening may restart the ESP32 on some boards.
    time.sleep(1.5)
    for _ in range(12):
        ser.write(b'P')
        if wait_for(ser, 'PONG', 0.5):
            print('MechDog receiver: PONG')
            return ser
    ser.close()
    raise RuntimeError('No PONG. Check the robot main.py, board boot and COM port.')


def black_line(frame):
    h, w = frame.shape[:2]
    y0 = int(0.6 * h)
    roi = frame[y0:, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 70))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    direction = 'NO LINE'
    if contours:
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if 180 < area < mask.size * 0.4:
            m = cv2.moments(contour)
            if m['m00']:
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00']) + y0
                error = cx - w // 2
                threshold = max(12, int(0.08 * w))
                if error < -threshold:
                    direction = 'LEFT'
                elif error > threshold:
                    direction = 'RIGHT'
                else:
                    direction = 'CENTER'
                cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
                cv2.drawContours(frame[y0:, :], [contour], -1, (0, 255, 0), 2)
    cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
    return direction, mask


def main():
    ser = None
    cap = None
    armed = False
    last_send = 0.0
    arm_time = 0.0
    try:
        cap = cv2.VideoCapture(CAMERA_URL)
        if not cap.isOpened():
            raise RuntimeError('Camera stream failed; connect to HW_ESP32S3CAM Wi-Fi.')
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # May be ignored by some backends
        ser = connect_robot()
        ser.write(b'D')
        print('Ready: G=drive for 5s, S/space=stop, Q=quit')
        while True:
            ok, frame = cap.read()
            if not ok:
                print('Camera frame lost -> stop')
                if ser:
                    ser.write(b'D')
                break
            direction, mask = black_line(frame)
            status = 'DRIVING' if armed else 'STOPPED'
            cv2.putText(frame, f'{direction} | {status}', (8, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
            cv2.putText(frame, 'G=start  S/SPACE=stop  Q=quit', (8, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.imshow('MechDog Black Line', frame)
            cv2.imshow('Black Mask', mask)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key in (ord('s'), 32):
                ser.write(b'D')
                armed = False
                print('Manual stop')
            if key == ord('g') and not armed and direction != 'NO LINE':
                ser.reset_input_buffer()
                ser.write(b'A')
                if wait_for(ser, 'ARMED', 0.35):
                    armed = True
                    arm_time = time.monotonic()
                    last_send = 0.0
                    print('Armed for 5 seconds')
                else:
                    ser.write(b'D')
                    print('Arm not confirmed; remaining stopped')
            if not armed:
                continue
            now = time.monotonic()
            if direction == 'NO LINE' or now - arm_time >= RUN_SECONDS:
                ser.write(b'D')
                armed = False
                print('Line lost or time limit -> stopped (press G to restart)')
                continue
            if now - last_send >= HEARTBEAT_SECONDS:
                if direction == 'CENTER':
                    command = b'F'
                elif direction == 'LEFT':
                    command = b'R' if INVERT_STEERING else b'L'
                else:
                    command = b'L' if INVERT_STEERING else b'R'
                ser.write(command)
                last_send = now
    finally:
        if ser is not None:
            try:
                ser.write(b'D')
                ser.flush()
            except (OSError, serial.SerialException):
                pass
            ser.close()
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
