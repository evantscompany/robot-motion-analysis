# phases

이 디렉터리는 실험을 단계적으로 수행하기 위한 phase 묶음입니다.

## 구조
- `phaseXX_*` 형태의 폴더들이 존재
- 각 phase 폴더 아래에 해당 단계의 컨트롤/시각화/bringup 관련 Python 코드가 배치되어 있음

## 포함된 주요 phase(현재 트리 기준)
- phase06_waypoint: waypoint controller
- phase07_rviz_path: RViz 경로 시각화/표시
- phase08_waypoint_marker: waypoint marker 표시
- phase09_active_waypoint: active waypoint 선택/표시
- phase10_heading_marker: heading marker 표시
- phase11_waypoint_route: waypoint route
- phase13_fake_robot_simulator: fake odom 생성(시뮬레이션)
- phase14_launch_system: bringup/런칭 통합
- phase15_16_waypoint_tracker: waypoint tracker
- phase17_multi_waypoint: multi waypoint tracker
- phase18_error_logger: 에러/튜닝 로그 기록
- phase19_analysis: 분석 스크립트
- phase20_kp_tuning: Kp 튜닝 비교
- phase21_rotate_then_go: rotate 후 이동
- phase22_pure_pursuit: pure pursuit
- phase23_pure_pursuit_path_tracker: pure pursuit path tracker

## 사용 방법(개략)
일반적으로 각 phase 디렉터리의 Python entry가 해당 단계에서 필요한 노드 조합을 구성합니다.
실행/파라미터는 각 phase 폴더의 코드(`bringup.py`, `*_tracker.py` 등)를 확인하세요.

