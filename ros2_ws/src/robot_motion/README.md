# robot_motion

`robot_motion` 패키지는 경로 생성/시각화/트래킹 알고리즘을 제공하는 ROS2(a m e n t_python) 패키지입니다.

## 주요 기능

### 1) 경로/지도 시각화
- `grid_map_visualizer` : 고정된 grid map(0=free, 1=obstacle)을 RViz Marker로 표시

### 2) 경로 트래킹 컨트롤러
패키지 내에 여러 컨트롤러 구현이 있으며, 공통적으로 `/odom` 구독 후 `cmd_vel`을 발행하여 로봇을 유도합니다.

- `pure_pursuit_tracker`
  - 미리 정의된 path 상에서 가장 가까운 점을 찾고, lookahead offset만큼 전방의 goal point를 선택
  - 해당 goal을 기준으로 curvature를 계산하여 `Twist(linear.x, angular.z)`를 생성

- `stanley_controller`
  - path의 각 선분에 대해 projection point를 계산하고 cross-track error(부호 포함)를 추정
  - Stanley steering 법칙으로 heading + cte 보정을 반영하여 `cmd_vel` 생성

### 3) RViz/마커 및 상태 시각화
- `robot_marker` : `/odom` pose를 Arrow marker로 표시
- `trajectory_visualizer` : 누적되는 `/odom` pose를 `nav_msgs/Path`로 publish
- `cross_track_visualizer` : 가장 가까운 path 점과 로봇 위치를 선으로 표시
- `segment_cte_visualizer` : 가장 가까운 선분 projection과 로봇 위치를 선으로 표시

### 4) 시뮬레이션용
- `fake_robot_simulator`
  - `cmd_vel`을 구독하고 내부 적분으로 `/odom`을 생성하여 테스트/phase 실행을 돕는 역할
  - 일정 timeout 미수신 시 velocity를 0으로 만드는 로직이 포함되어 있어 phase 15~16 성격의 테스트에 활용

## ROS2 입출력(공통)
- 입력: `/odom` (`nav_msgs/Odometry`)
- 출력: `cmd_vel` (`geometry_msgs/Twist`)
- 시각화: RViz marker topics (예: `grid_map`, `/robot_marker`, `cross_track_error`, `/segment_cte` 등)

## 실행(개략)
`setup.py`의 `console_scripts` entry points를 기준으로 실행 가능합니다.

