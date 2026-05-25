import pandas as pd

FILE_PATH = "/home/msm031/robot_motion_analysis/ros2_ws/src/robot_data_logger/robot_log_20260524_162137.csv"
df= pd.read_csv(FILE_PATH)

# 최종 drift
final_y = df['pos_y'].iloc[-1]
drift = final_y

# 간단한 correction coefficient

k=0.1

correction_gain = 1+(k*drift)

print("====Correction Analysis====")
print(f"Drift           :{drift:.3f}")
print(f"Correction Gain :{correction_gain:.3f}")