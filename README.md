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

## 비전 판정 + 헤드램프 피드백 (로봇은 걷지 않음, 램프만 사용)

헤드램프 = 초음파 센서 RGB. **파랑 = 대기**, **초록 = 통과/안전**, **빨강 = 불량/위험**. 판정이 1~1.5초 유지돼야 바뀐다.

```powershell
py vest_check.py                # 안전조끼 착용 판정 (조끼 입은 사람 사진 → 초록, 안 입은 사람 → 빨강)
py hazard_check.py              # 장애물 판정 (검은 장애물만 → 빨강, 흰 라바콘 4개로 둘러쌈 → 초록)
py vest_check.py --no-robot     # 화면만 / --source 0 (웹캠) / --source 사진.jpg / --selftest
```

- **조끼(`vest_check.py`)**: 형광 연두·노랑·주황 색 영역 합계 ≥ 화면 3% = 조끼. 얼굴(OpenCV YuNet,
  `models/`)이 조끼 바로 위에 있으면 폭도 확인. 카메라에 바짝 붙은 큰 얼굴(조작자)은 무시 → 파랑.
  조끼 색이 다르면 창에서 `v` + 조끼 클릭, `s` 저장(`vest_config.json`). `--zones`: 공구 색 태그 A~D 위치 검사.
- **장애물(`hazard_check.py`)**: 카메라 고정 후 **빈 책상에서 `b` 한 번**(배경 저장, `hazard_bg.png`).
  이후 배경 대비 새로 생긴 검은 물체 = 장애물, 흰 물체 = 라바콘. 겹친 컵은 바닥 테두리 높이로 분리해서 센다.
  책상 뒤 사람·벽은 무시. 카메라나 조명이 바뀌면 `b`를 다시 누른다.
- 2초마다 화면이 `logs/vest_*`, `logs/hazard_*`에 저장된다(판정이 이상하면 이걸 보면 된다).

## 카메라 Wi-Fi (주의)

- 주변 팀 카메라도 이름이 전부 `HW_ESP32S3CAM`이다. 내 카메라 BSSID를 한 번 지정해 둔다:
  ```powershell
  py tools\find_my_camera.py                      # 카메라별 화면을 logs\cam_<bssid>.jpg 로 저장
  py tools\find_my_camera.py --set 8c:bf:ea:87:28:25   # 내 카메라 저장 (cam_bssid.txt, 각자 로컬)
  ```
- 프로그램은 **시작할 때 한 번만** 그 카메라에 연결한다. 실행 중 사용자가 Wi-Fi를 바꾸면 건드리지 않고
  (화면 끊김 → 정지/파랑), 종료 시 아직 카메라에 붙어 있을 때만 원래 Wi-Fi로 되돌린다.
- 카메라 AP에는 인터넷이 없다(PC 인터넷 끊김). Windows가 수십 초마다 원래 Wi-Fi로 돌아갈 수 있다.

## 구성

| 파일 | 역할 |
|---|---|
| `line_follow_auto.py` | 자율 주행 메인. 비례 조향, 커브 감속, 라인 소실 시 탐색 후 정지·자동 재출발 |
| `linevision.py` | 테이프 검출(밝기+색 엣지 기반, 조명 기울기/그림자에 강함) |
| `vest_check.py` | 안전조끼 착용 판정 + 헤드램프 (공용 카메라 소스/안정화 코드 포함) |
| `hazard_check.py` | 장애물 + 라바콘 판정 + 헤드램프 (배경 비교 방식) |
| `robot/mechdog_rx.py` | 로봇 측 수신기. **실행 시 RAM에 업로드**(플래시 미변경). 워치독 500 ms, 구간 제한, 헤드램프 |
| `line_follow_pc_v2.py` | 수동 시험용(W/A/D 펄스, G 5초 자동) + 공용 Camera/Robot 클래스 |
| `models/face_detection_yunet_2023mar.onnx` | OpenCV 공식 얼굴 검출 모델(opencv_zoo) |
| `tools/mdlink.py` | 시리얼/raw-paste REPL 도우미(DTR/RTS 미토글 → 보드 리셋 방지) |
| `tools/wifi_bssid.py`, `tools/find_my_camera.py` | 같은 이름의 카메라 중 특정 BSSID에 연결 / 내 카메라 찾기 |
| `tools/led_test.py` | 헤드램프 초록→빨강→끔 시험(모터 없음) |
| `tools/diag_robot.py` | 읽기 전용 진단 + 로봇 파일 PC 백업 |
| `tools/run_pong_test.py`, `tools/rx_test.py`, `tools/init_test.py` | 통신·수신기(DRY/모터)·초기화 단계별 시험 |
| `tools/robot_main_switch.py` | 로봇 자동실행 `main.py` 대기/복원 |
| `main.py`, `line_follow_pc.py`, `README_KO.txt` | 초기 버전(참고용) |
| `CLAUDE.md` | 디버깅 경위·안전 수칙 |

## 문제 해결

- **주행 중 `ClearCommError ... 액세스가 거부` 후 종료**: 모터에 힘이 들어갈 때 보드가 재부팅돼 USB가 끊긴 것.
  거의 항상 **배터리 부족 또는 전원 스위치 OFF**(USB 전원만으로는 서보 구동 불가). 충전된 배터리로 교체.
- 같은 COM 포트를 쓰는 프로그램(miniterm, Hiwonder 에디터)은 한 번에 하나만.

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
헤드램프: `G` 초록 · `E` 빨강 · `B` 파랑 · `O` 끔 (초기화 `I` 없이 사용 가능, 모터 무관).

## 안전

소프트웨어 정지는 비상정지 장치가 아니다. 시험 중엔 전원 스위치를 바로 끌 수 있게 하고, USB 케이블이 다리에 걸리지 않게 한다.
