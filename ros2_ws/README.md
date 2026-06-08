# ROS2 Motion Analysis Workspace

이 저장소는 **ROS2 + (Isaac Sim 기반) 로봇 모션 분석/보정**을 위한 워크스페이스입니다.

## 구성 요소

### 1) `src/` (ROS2 패키지)
- `robot_motion`: 경로/트래킹 알고리즘 및 RViz/시각화 노드
- `robot_data_logger`: 로봇 로그 수집 및 보정 관련 노드
- `robot_serial_bridge` (있음): 로봇/외부 장치 브리지(해당 폴더가 워크스페이스에 존재)
- `odom_publish`: `/cmd_vel`로부터 단순 odom 계산 및 `/odom` 퍼블리시(예: `OdomPublisher`)

### 2) `phases/`
실험을 단계적으로 수행하기 위한 스크립트/노드 묶음입니다. 예:
- waypoint / rviz path / pure pursuit / stanley 등 컨트롤 방식이 단계별로 구현되어 있음
- phase별 launch/bringup 성격의 코드가 `phaseXX_*` 디렉터리 아래에 존재

### 3) `analysis/`
실험/로그 데이터를 분석하기 위한 파이썬 스크립트입니다.
- `correction_analysis.py`, `drift_analysis.py`
- `plot_trajectory.py`, `live_plot.py`
- `compare_correction.py` 등

### 4) `docs/`
분석 결과 이미지/리포트 자산(예: `correction_comparison.png`, `trajectory_plot.png`)이 포함됩니다.

## 빠른 시작(개략)
1. 워크스페이스 빌드
2. phase 또는 관련 노드를 실행하여 `/cmd_vel`, `/odom`, RViz marker 등을 확인
3. `analysis/`로 로그/궤적을 분석

> 주: 현재 각 패키지별 `package.xml`/설명 항목의 placeholder(`TODO`)가 존재하므로, 실제 실행 방법은 `phases/`와 해당 패키지의 entry script를 기준으로 확인하는 것을 권장합니다.

