import matplotlib
matplotlib.use('Agg')

import pandas as pd
import matplotlib.pyplot as plt

# CSV 읽기
df = pd.read_csv(r'/home/msm031/robot_motion_analysis/ros2_ws/src/robot_data_logger/robot_log_20260524_162137.csv')

# 좌표 추출
x = df['pos_x'].to_numpy()
y = df['pos_y'].to_numpy()

# Plot
plt.figure(figsize=(8, 8))

plt.plot(x, y, marker='o')

# 시작점
plt.scatter(x[0], y[0], s=100)

# 종료점
plt.scatter(x[-1], y[-1], s=100)

plt.xlabel('X Position')
plt.ylabel('Y Position')

plt.title('Robot Trajectory')

plt.axis('equal')
plt.grid(True)

plt.savefig('../docs/trajectory_plot.png')

print('Plot Saved!')