MechDog 검은색 라인트레이싱 - USB-C + Wi-Fi 방식

0. 주의: 카메라 ESP32-S3에는 이 main.py를 절대 올리지 않습니다. MechDog 메인 ESP32에만 올립니다.
1. 기존 로봇 파일 main_ble_backup.py 및 boot.py와 PC 백업본을 보존합니다.
2. MechDog 메인 ESP32에 현재 /main.py가 없는지 확인합니다.
3. Hiwonder Python Editor로 이 폴더의 main.py를 메인보드에 업로드합니다.
4. 로봇을 넓고 평평한 바닥에 놓고 재부팅합니다. 시작 즉시 로봇은 정지 상태입니다.
5. Hiwonder Python Editor 연결을 해제하고 프로그램을 종료합니다.
6. PC를 HW_ESP32S3CAM Wi-Fi에 연결, 카메라 영상 스트림 주소 확인.
7. MechDog 메인보드 USB-C를 PC에 연결. COM9가 맞는지 확인합니다.
8. VS Code PowerShell: py -m pip install opencv-python pyserial
9. VS Code PowerShell: py line_follow_pc.py
10. 창에서 G: 최대 5초 주행 / S 또는 스페이스: 정지 / Q: 종료

안전: 초기에는 로봇 다리를 공중에 띄워 명령 방향만 테스트한 다음 바닥에 둡니다.
로봇에 USB 케이블이 연결된 유선 테스트입니다. 케이블을 걸거나 당기지 마세요.
정지하지 않으면 즉시 로봇 전원을 끕니다. 자동 정지는 테스트용 소프트웨어 안전장치이지 산업용 비상정지 인증이 아닙니다.
검은색 선을 놓치면 즉시 정지/해제, PC 전송 정지 >600ms이면 로봇 자체가 정지/해제합니다.
주행은 시작 후 최대 5초 (로봇 자체 6초 제한). 재시작은 매번 G.

만약 PC에서 'No PONG'이 나오면:
- Hiwonder Python Editor/miniterm/Arduino 시리얼 모니터를 종료
- main.py가 MechDog 메인 ESP32에 저장됐는지, 부팅 시 오류가 없는지 확인
- 시리얼 열 때 ESP32가 재시작할 수 있으므로 포트가 맞는지 점검
- 해당 펌웨어의 sys.stdin + select.poll 호환성 점검

복원: 작업 종료 후 메인 ESP32에서 /main.py를 삭제 또는 이름 변경하고
/main_ble_backup.py를 /main.py로 복원한 뒤 재부팅합니다. 기존 백업을 덮어쓰지 마세요.
