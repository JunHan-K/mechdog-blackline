# MechDog 검은색 라인트레이싱

Hiwonder MechDog + ESP32-S3 카메라(Wi-Fi 스트림) + Windows PC(OpenCV)로 바닥의 검은 테이프를 따라 걷는 프로젝트.
PC가 영상을 처리하고 USB 시리얼(COM 포트)로 로봇에 보행 명령을 보낸다.

## 빠른 시작

```powershell
py -m pip install opencv-python pyserial numpy
py line_follow_auto.py            # 실행하면 바로 라인 추종 (창: SPACE 일시정지, Q 종료)
py line_follow_auto.py --calibrate   # 카메라 위치를 바꿨으면 방향 재보정
```

- 로봇 USB를 PC에 연결(기본 `COM9`, 다르면 `--port COMx`).
- 카메라 Wi-Fi `HW_ESP32S3CAM`이 저장돼 있으면 프로그램이 자동 전환하고 종료 시 원래 Wi-Fi로 복귀한다.
- 카메라는 로봇 앞 바닥을 내려다보게, 밝은 바닥 + 검은 테이프 위에 로봇을 둔다.
- 첫 실행 시 로봇이 선 위에서 잠깐 회전해 좌우 방향을 자동 보정한다(`steer_calib.json`).

## 구성

| 파일 | 역할 |
|---|---|
| `line_follow_auto.py` | 자율 주행 메인. 비례 조향, 커브 감속, 라인 소실 시 탐색 후 정지·자동 재출발 |
| `linevision.py` | 테이프 검출(밝기+색 엣지 기반, 조명 기울기/그림자에 강함) |
| `robot/mechdog_rx.py` | 로봇 측 수신기. **실행 시 RAM에 업로드**(플래시 미변경). 워치독 500 ms, 구간 제한 |
| `line_follow_pc_v2.py` | 수동 시험용(W/A/D 펄스, G 5초 자동) + 공용 Camera/Robot 클래스 |
| `tools/mdlink.py` | 시리얼/raw-paste REPL 도우미(DTR/RTS 미토글 → 보드 리셋 방지) |
| `tools/diag_robot.py` | 읽기 전용 진단 + 로봇 파일 PC 백업 |
| `tools/run_pong_test.py`, `tools/rx_test.py`, `tools/init_test.py` | 통신·수신기(DRY/모터)·초기화 단계별 시험 |
| `tools/robot_main_switch.py` | 로봇 자동실행 `main.py` 대기/복원 |
| `main.py`, `line_follow_pc.py`, `README_KO.txt` | 초기 버전(참고용) |
| `CLAUDE.md` | 디버깅 경위·안전 수칙 |

## 로봇 측 주의 (중요)

- **`/main.py`가 없으면 펌웨어가 기본 앱(BLE+WiFi) `main.py`를 재생성한다.** 이 앱은 ESP-IDF 힙을 거의 다 써서
  추가 `MechDog()`/스레드 시 `can't create thread`, `i2c command link malloc error`를 유발했다.
- 그래서 로봇에는 아무 것도 안 하는 대기용 `main.py`를 설치해 둔다:
  ```powershell
  py tools\robot_main_switch.py --status
  py tools\robot_main_switch.py --idle --reset      # 대기용 설치 (원본은 이름 바꿔 보존)
  py tools\robot_main_switch.py --restore --reset   # Hiwonder 앱 복원
  ```
- 서보 보정(`set_offset`/`save_offset`), `erase_flash`, 펌웨어 재설치는 하지 않는다.

## 프로토콜 (PC → 로봇, 1바이트)

`P` PONG · `I` MechDog 초기화 · `A` 시동 · `D`/`S` 정지 · `F`/`L`/`R` 고정 이동 ·
`0`~`8` 비례 조향(4=직진) · `Q` 상태 · `X` 수신기 종료. 명령이 500 ms 끊기면 로봇 자체 정지.

## 안전

소프트웨어 정지는 비상정지 장치가 아니다. 시험 중엔 전원 스위치를 바로 끌 수 있게 하고, USB 케이블이 다리에 걸리지 않게 한다.
