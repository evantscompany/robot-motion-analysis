# GUI 없는 WSL 환경에서도 matplotlib 사용 가능하게 설정
import matplotlib
matplotlib.use('Agg')

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ============================================================
# CSV 데이터 읽기
# ============================================================

# 로봇 주행 데이터셋 로드
FILE_PATH = "/home/msm031/robot_motion_analysis/ros2_ws/src/robot_data_logger/robot_log_20260524_162137.csv"
df = pd.read_csv(FILE_PATH)

# numpy array 형태로 변환
x = df['pos_x'].to_numpy()
y = df['pos_y'].to_numpy()

# ============================================================
# Drift 분석
# ============================================================

# 마지막 y 값 = 최종 drift 라고 가정
final_drift = y[-1]

# correction coefficient
# 값이 클수록 강하게 보정
k = 0.5

# ============================================================
# 보정 trajectory 생성
# ============================================================

# drift를 점진적으로 줄이는 방식
#
# 예:
# 기존 y = 0.4
# 보정 후 y = 0.2
#
# 즉 trajectory를 중앙 방향으로 당김

corrected_y = y - (final_drift * k)

# ============================================================
# Plot 생성
# ============================================================

plt.figure(figsize=(8, 8))

# 원본 trajectory
plt.plot(
    x,
    y,
    marker='o',
    label='Original Trajectory'
)

# 보정 trajectory
plt.plot(
    x,
    corrected_y,
    marker='x',
    label='Corrected Trajectory'
)

# 시작점 표시
plt.scatter(x[0], y[0], s=120)

# 종료점 표시
plt.scatter(x[-1], y[-1], s=120)

# 그래프 정보
plt.xlabel('X Position')
plt.ylabel('Y Position')

plt.title('Trajectory Correction Comparison')

plt.axis('equal')
plt.grid(True)

# 범례 표시
plt.legend()

# 결과 저장
plt.savefig('../docs/correction_comparison.png')

print('Correction comparison plot saved!')