# odom_publish

`odom_publish`는 입력으로 `cmd_vel`을 받아 단순 적분으로 odom을 추정하고 `/odom`을 퍼블리시하는 용도의 패키지(스크립트 묶음)입니다.

## 포함 노드

### 1) `odom_publisher.py` (`OdomPublisher`)
- 입력: `cmd_vel` (`geometry_msgs/Twist`)
- 출력: `/odom` (`nav_msgs/Odometry`)
- 동작:
  - `v = msg.linear.x`, `w = msg.angular.z`로부터
  - `x += v*cos(theta)*dt`, `y += v*sin(theta)*dt`, `theta += w*dt` 형태로 포즈를 적분
  - theta를 quaternion(z,w)로 변환해 odom pose orientation에 반영

### 2) `tf_broadcaster.py` (`TFBroadcaster`)
- 주기적으로 `odom -> base_link` 변환을 `tf2_ros.TransformBroadcaster`로 브로드캐스트
- translation은 0.0, rotation은 w=1.0(항등) 형태로 설정되어 있음(현재 예제/기본값 성격)

## 참고
`ros2_ws/src/odom_publish`에는 URDF(`urdf/robot.urdf`)가 포함되어 있습니다.

