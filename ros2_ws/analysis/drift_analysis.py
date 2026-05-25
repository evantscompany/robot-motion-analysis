# 얼마나 정확히 움직였는가 를 계산할 필요성이 있음. 

import pandas as pd

# CSV 읽기
FILE_PATH = "/home/msm031/robot_motion_analysis/ros2_ws/src/robot_data_logger/robot_log_20260524_162137.csv"
df = pd.read_csv(FILE_PATH)


# 최종위치
final_x = df['pos_x'].iloc[-1]
final_y = df['pos_y'].iloc[-1]

# drift 계산
drift = abs(final_y)

print('====Drift Analysis====')

print(f'Final  X Position: {final_x}')
print(f'Final  Y Position: {final_y}')
print(f'Drift error      : {drift:.3f}')

