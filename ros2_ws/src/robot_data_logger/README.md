# robot_data_logger

`robot_data_logger`는 로봇 로그를 수집/처리하기 위한 패키지입니다.

## 주요 기능
- `logger_node` : 로그 기록 담당
- `correction_node` : 수집된 데이터 기반 보정(correction) 로직 담당

## ROS2 엔트리포인트
`setup.py`의 `console_scripts`에 아래 노드가 등록되어 있습니다.
- `logger_node = robot_data_logger.logger_node:main`
- `correction_node = robot_data_logger.correction_node:main`

## 비고
현재 `package.xml` 및 `setup.py`의 description/license 등이 placeholder(`TODO`) 형태이므로, 실제 동작/토픽 스펙은 코드(`robot_data_logger/*.py`) 및 `logs/` 샘플을 참고하는 것을 권장합니다.

