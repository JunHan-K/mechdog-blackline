# MechDog 검은색 라인트레이싱 — Claude Code 인수인계 README

> **현재 작업 우선순위: 오류 복구 → USB 명령 수신 확인 → 모터와 비전 통합.**  
> 이 문서는 실제 진행 기록과 현재 제공된 소스 파일을 근거로 작성됐다. 추정 사항과 검증된 사항을 구분한다.

## 0. Claude Code에게 요청하는 작업

이 프로젝트는 **Hiwonder MechDog + ESP32-S3 카메라 1개 + Windows PC**를 이용해 검은색 바닥 테이프를 따라 걷는 로봇독을 구현하는 것이다. 현재 영상 검출과 개별 모터 제어는 성공했지만 **연속 제어 프로그램은 정상 동작하지 않았다.** 최근에는 ESP32가 `i2c: i2c command link malloc error`를 연속 출력했다.

**먼저 소스 전체를 읽고 원인을 분석하라.** 곧바로 주행하거나 펌웨어를 갈아엎지 말고 아래 순서로 진행해 달라.

1. 로봇 내부에서 **현재 어떤 `/main.py`가 실행되는지** 판별하는 안전한 진단 절차 제시.
2. 반복 I2C 메모리 오류를 일으킬 수 있는 `MechDog()` 초기화, `move()` 호출, Wi-Fi/스레드, 중복 작업과 펌웨어 특성을 검토.
3. **모터 구동 없이** PC→USB→MicroPython `P` / `PONG` 통신을 검증하는 최소 예제부터 제안.
4. 안정적이고 끊김에 안전한 제어 수신 코드를 수정하고, 변경 이유와 테스트 방법 제공.
5. 기존 OpenCV 검은색 라인 검출 프로그램과 통합. 단계별 시험은 **정지 상태→짧은 수동 전진→방향 확인→제한된 자동주행**으로 진행.
6. 실제 하드웨어에 대한 가정은 확인 질문 또는 진단 로그로 검증. **코드 수정/덮어쓰기 전 백업 절차** 명시.

**금지:** `erase_flash`, 공장 초기화, 메인 ESP32 Arduino 플래싱, 서보 보정값 삭제, 기존 백업 파일 덮어쓰기, 실패한 상태에서 무제한 자동주행, 무조건적인 재부팅 반복.

## 1. 환경 및 구성

| 구분 | 확인 상태 |
|---|---|
| PC | Windows / VS Code / PowerShell, 프로젝트 경로 `C:\Users\User\Desktop\mechdog\mechdog_blackline` |
| PC Python | Python 3.14 환경에서 `pyserial 3.5` 확인, OpenCV 검은색 검출 성공 |
| 로봇 메인 | Hiwonder MechDog ESP32, MicroPython `v1.21.0-dirty` (2024-12-17 빌드 메시지) |
| USB 통신 | CH340, 현재 확인된 포트 `COM9` (카메라 USB와 메인보드 USB 연결에 따라 달라질 수 있음) |
| 카메라 | Hiwonder ESP32-S3 비전 모듈 1개, 기존 Wi-Fi 영상 스트리밍 성공 |
| 카메라 Wi-Fi | `HW_ESP32S3CAM` (AP 모드) |
| 영상 주소 | `http://192.168.5.1:81/stream` |
| 카메라 연결 | 메인보드의 IIC 1번 확장 포트에 4핀 케이블로 연결됨 |
| 주행 방식 | ROS 2 없음. PC OpenCV에서 계산 → 메인 ESP32에 USB 직렬로 보행 지시하는 방식 |

**중요:** 카메라의 Wi-Fi 연결과 메인 ESP32의 `wifi connecting... wifi connect timeout` 출력은 서로 별개의 네트워크 문제일 수 있다. 메인 ESP32의 기본 프로그램에는 `WIFI_NAME = "hiwonder_node"`가 있었으며, 이 로그만으로 카메라 연결 실패를 뜻하지 않는다.

## 2. 기존에 성공한 테스트

- Hiwonder Python Editor로 메인보드 연결 및 MicroPython REPL 실행 성공.
- `from HW_MechDog import MechDog`, `mechdog = MechDog()` 성공.
- 기존 자동 실행 프로그램을 일시 비활성화한 후 `mechdog.action_run("nodding_motion")` 동작 성공. 그 전에 `OSError: can't create thread`가 발생했음.
- `mechdog.move(30, 0); time.sleep(0.5); mechdog.move(0, 0)`로 짧은 전진 및 정지 성공.
- PC `pyserial`을 이용한 단발성 `motor_test.py`의 약 0.5초 전진·정지 실행 성공.
- ESP32-S3 카메라 웹 영상 확인 성공. VS Code + OpenCV에서 **검은색 테이프를 LEFT / CENTER / RIGHT / NO LINE으로 검출하는 테스트 성공**.
- `py -m serial.tools.miniterm COM9 115200`로 MicroPython REPL 접속 및 `print("MECHDOG_USB_OK")` 출력 성공.

## 3. 제공된 소스 파일 — 반드시 실제 코드를 읽을 것

이 README와 함께 첨부한 프로젝트 폴더 파일은 다음과 같다.

```text
mechdog_blackline/
├── main.py             # [ESP32 MicroPython] PC에서 1바이트 명령 받는 실험 코드
├── line_follow_pc.py   # [Windows] 영상 검출 + COM9 명령 전송
└── README_KO.txt       # 초기 사용법
```

### `main.py` (로봇용, 제공된 사본)

- `HW_MechDog.MechDog()` 객체 생성.
- `uselect.poll()` / `sys.stdin.read(1)`로 USB REPL 직렬 입력을 읽도록 구현.
- 통신 프로토콜: `P` → `PONG`, `A` → `ARMED`, `F` = 전진, `L` = 좌방향 수정, `R` = 우방향 수정, `D` 또는 `S` = 정지/해제.
- 보폭 `STEP_MM=30`, 회전 설정 `TURN_DEG=15`.
- 수신 타임아웃 `WATCHDOG_MS=600`, 로봇 자체 주행 제한 `MAX_RUN_MS=6000`.
- 부팅 후 `stop()`을 호출하고 `ROBOT_READY`를 출력하도록 작성됨.
- **실제 로봇에 이 코드가 업로드 및 실행되었다는 증거는 아직 없다.**

### `line_follow_pc.py` (PC용, 제공된 사본)

- `CAMERA_URL = 'http://192.168.5.1:81/stream'`, `PORT = 'COM9'`, `BAUD = 115200`.
- 영상 하단 40% ROI, HSV 검정 범위 `(0,0,0)`~`(180,255,70)`, 형태학적 열기, 최대 컨투어 중심점.
- 라인 위치를 LEFT/CENTER/RIGHT/NO LINE으로 구분.
- `connect_robot()`이 `P`를 전송하여 `PONG` 응답을 기다림(12회 재시도).
- `G`로 최대 5초 동작, `S`/Space 정지, `Q` 종료; 0.12초마다 방향 명령 반복; `INVERT_STEERING` 설정 있음.
- `cap.read()`가 블로킹될 수 있고, UART 버퍼·REPL 입력 처리와 watchdog 신뢰성은 실제 테스트 필요.
- 기본 검정 마스크는 그림자도 검출할 수 있음. 밝은 바닥 + 검은 테이프에서 테스트할 것.

## 4. 실제 실패 기록과 현재 핵심 문제

### 4-A. `No PONG`

```text
Board response: P
Board response: P
... (반복)
RuntimeError: No PONG. Check the robot main.py, board boot and COM port.
```

그 뒤 `miniterm`에서 다음 로그가 관찰됨.

```text
NameError: name 'PPPPPPPPPPPP' isn't defined
```

**가장 가능성 높은 해석:** `P` 문자가 로봇의 명령 처리 루프가 아니라 일반 MicroPython REPL로 전달되어 코드처럼 해석됨. 즉, 로봇용 실험 `main.py`가 실행되지 않았거나 시리얼 입력 경로가 의도대로 작동하지 않음. 단, 정확한 원인은 로봇 내부 실제 파일/부팅 로그와 함께 검증해야 함.

### 4-B. 실제 `/main.py`가 기존 Hiwonder 코드였음

사용자가 로봇에서 실행한 명령:

```python
print(open('/main.py').read()[:350])
```

당시 출력 시작 부분:

```python
# Hiwonder MechDog
# MicroPython - APP Bluetooth and WIFI control
import Hiwonder
import time
import Hiwonder_IIC
from Hiwonder_BLE import BLE
from Hiwonder_WIFI import WIFI_CL
from HW_MechDog import MechDog
import machine
WIFI_NAME = "hiwonder_node" #WiFi SSID
```

따라서 **적어도 그 시점의 로봇 `/main.py`는 제공된 라인트레이싱 실험 `main.py`가 아니었음**. 이후 사용자가 파일을 성공적으로 교체했는지 불명확하다. 원본 `main.py`는 BLUETOOTH+WIFI 제어 코드다.

### 4-C. 재부팅 로그

```text
robot_instance: ...
QMI8658 init.
wifi connecting...
wifi connect timeout
MicroPython v1.21.0-dirty on 2024-12-17; MechDog with ESP32
Type "help()" for more information.
>>>
```

위 로그에 로봇 실험 코드의 `ROBOT_READY`가 없었음. 단, 출력 타이밍과 실제 설치 상태의 재확인이 필요함.

### 4-D. 심각한 I2C 할당 오류 반복

```text
E (189387) i2c: i2c command link malloc error
E (189397) i2c: i2c command link malloc error
E (189407) i2c: i2c command link malloc error
... 약 10ms 간격으로 연속 출력 ...
```

사용자는 당시 `Ctrl+C`로 실행을 중단하기 어려웠으며 `miniterm`에서 `--- exit ---`로 나옴. `--- exit ---`는 **PC 터미널 종료이지 로봇 측 프로세스 중지의 증거가 아님**.

원인 **미확정**: 연속 I2C 호출, 메모리 고갈/단편화, 모터 제어 라이브러리 내부 상태, 자동 실행 중복, 부팅 코드/백그라운드 작업, 특정 펌웨어 드라이버 문제 등 가능. 새 코드가 직접 원인이라고 단정하지 말고 **현재 실행 파일과 정상 부팅 여부부터 조사**해야 함.

### 4-E. 카메라 I2C 스캔

```python
Hiwonder_IIC.IIC(2).scan()  # []
Hiwonder_IIC.IIC(1).scan()  # [106, 119]
```

카메라 4핀 IIC 1번 확장 포트 연결은 사용자가 확인. Wi-Fi 카메라 영상은 정상 작동함. **현재 카메라 펌웨어는 I2C 라인트레이싱 인터페이스를 제공한다고 확인되지 않았음.** 이번 계획은 카메라 I2C가 아닌 **Wi-Fi 스트림 + PC OpenCV**에 의존한다.

## 5. 안전한 복구 및 진단 순서 (최우선)

1. 로봇을 평평한 바닥의 안전한 곳에 두고 이동 명령을 중단. 필요 시 전원 OFF. USB 케이블이 다리에 걸리지 않도록 확보.
2. Hiwonder Python Editor / Arduino IDE / VS Code 시리얼 모니터 / miniterm 중 **COM9를 점유하는 프로그램은 한 번에 하나만** 실행.
3. 기존 `main_ble_backup.py`, `main_wifi_backup.py`, `main_original.py`, `boot_original.py` 등 백업은 **있을 수도 있으므로 실제 파일 목록 확인**. 존재 여부를 가정하지 말 것.
4. 전원 재인가 후 가능한 경우 MicroPython REPL(`py -m serial.tools.miniterm COM9 115200`) 접속. 계속 오류가 쏟아지면 비정상 주행을 멈추고 무리한 명령 입력 금지.
5. REPL이 안전하게 응답할 때 **읽기 전용**으로 `import os; print(os.listdir('/'))`, `print(open('/main.py').read()[:300])` 등 확인. `/main.py`가 없어도 예외 처리.
6. 문제가 되는 자동 실행 `main.py`가 명확히 확인되면 **기존 백업명과 충돌하지 않는 이름으로 보존** 후 재부팅해 오류가 지속되는지 비교. 로봇 기동에 필요한 `boot.py`·펌웨어·보정 데이터는 변경 금지.
7. 필요 시 `gc.mem_free()` 등 사용 가능한 MicroPython 메모리 지표를 **최소 실행 환경**에서 측정. I2C를 반복 호출하는 기존 코드는 원인 확인 전 재실행하지 않음.
8. **통신만 하는 최소 수신기**를 먼저 검증: 모터 라이브러리를 import하지 않고 `P` → `PONG`를 확인. REPL 에코와 프로토콜 응답을 혼동하지 않을 것.
9. 그 다음 `MechDog()`과 `move()`를 한 번씩 추가하고, 모터는 반드시 제한 시간과 정지 보장을 갖춘 상태에서만 시험.
10. 이상 없을 때만 PC 영상 검출을 연결. 통신 끊김, 영상 끊김, 라인 소실, 타임아웃 시 안전 정지 검증.

**안전 주의:** MicroPython 코드 레벨 watchdog은 하드웨어 비상정지 장치가 아니다. 제어 기능을 시험할 때는 즉시 전원을 끌 수 있어야 한다. 문제가 있는 상태에서 주행시키지 말 것.

## 6. Claude에게 특히 검토받고 싶은 기술 문제

- `sys.stdin` + `uselect.poll()`을 현재 Hiwonder MicroPython `v1.21.0-dirty` USB-CH340 UART0 콘솔에서 안전하게 사용할 수 있는가?
- UART0 REPL이 사용자 프로그램과 경쟁하는 경우, `dupterm` 또는 **별도 UART/USB-TTL**이 필요한가? 문서/실측에 맞춰 판단.
- `main.py`의 `stop()` 및 `motion()`에서 `HW_MechDog.move()`의 반복 호출을 줄이거나, 단일 실행 흐름·큐·속도 제한을 두어야 하는가?
- `MechDog()` 생성, Wi-Fi 초기화, 스레드 및 I2C 관련 할당량을 어떻게 점검하는가?
- 600ms watchdog이 블로킹 `move()`, 시리얼 수신, 영상 프레임 중단 상황에서 **실제로** 정지를 보장하는가? 보장되지 않으면 한계를 설명하고 개선할 것.
- `line_follow_pc.py`는 연결 확인 전 모터 동작을 금지하고, 카메라에서 오래된 프레임을 받거나 네트워크 끊김 시 즉시 정지해야 한다.
- 매 프레임마다 REPL에 `exec(...)`를 전송하는 방식은 사용하지 말 것. 지속적인 명령 수신 또는 다른 검증된 통신 구조를 사용.

## 7. 성공 판정 기준

- [ ] 문제의 I2C 메모리 오류가 부팅 후 반복되지 않음
- [ ] 로봇 내부 현재 파일 목록과 부팅 실행 파일을 확인·백업함
- [ ] 모터 비활성 상태에서 `P` → `PONG` 정상 응답
- [ ] `A/D/F/L/R` 명령이 정해진 절차로만 동작하고, 정지 동작 확인
- [ ] PC가 연결 해제되거나 명령이 끊기면 로봇이 정지하는지 실측
- [ ] OpenCV 검은색 라인 검출 결과에 따라 방향 변경
- [ ] 라인을 잃거나 영상 수신이 끊기면 정지
- [ ] 5초 제한 주행을 반복 재현, 테스트 끝나면 원본 프로그램 복원 가능

## 8. Claude Code 첫 요청 예시

> 이 README 및 프로젝트의 `main.py`, `line_follow_pc.py`를 전부 읽어줘. 현재 영상 검출과 단발 모터 제어는 성공했지만, `P`→`PONG` 통신이 실패하고 `i2c command link malloc error`가 연속 발생했어. 실제 로봇의 `/main.py`가 Hiwonder 기본 APP Bluetooth+WIFI 코드였던 증거도 있어. 1) 원인 가설을 증거별로 분리하고 2) 안전한 비파괴 진단 순서부터 정리해줘. 이후 모터를 건드리지 않는 최소 USB 수신기 테스트와 신뢰성 있는 명령 수신 구조를 코딩해 줘. 각 단계는 내가 결과를 보내면 진행하고, 원본/서보 보정/펌웨어를 지우지 마. Arduino IDE, ROS 2는 사용하지 않는 방향으로 해줘.

---

작성 근거: 사용자와의 실제 디버깅 로그 + 함께 전달하는 `MechDog_Black_Line_Follower.zip`의 `main.py`, `line_follow_pc.py`, `README_KO.txt`. **사용자 PC의 최신 수정본이나 로봇 내부 현재 파일 상태는 직접 읽지 못했으므로 Claude가 첫 단계에서 반드시 확인할 것.**
