import pandas as pd
import matplotlib.pyplot as plt

# =========================================================
# CSV 로드
# =========================================================

csv_path = '/home/msm031/robot_motion_analysis/ros2_ws/phases/phase18_error_logger/logs/error_log_20260531_131830.csv'

df = pd.read_csv(csv_path)

print("=== CSV Preview ===")
print(df.head())


# =========================================================
# 1. Distance Error Plot
# =========================================================

plt.figure(figsize=(10,5))

plt.plot(
    df['time'].values,
    df['distance_error'].values,
    label='Distance Error (m)'
)

plt.title('Distance Error over Time')
plt.xlabel('Time (s)')
plt.ylabel('Distance Error (m)')
plt.grid(True)
plt.legend()

plt.show()


# =========================================================
# 2. Angle Error Plot
# =========================================================

plt.figure(figsize=(10,5))

plt.plot(
    df['time'].values,
    df['angle_error'].values,
    label='Angle Error (rad)'
)

plt.title('Angle Error over Time')
plt.xlabel('Time (s)')
plt.ylabel('Angle Error (rad)')
plt.grid(True)
plt.legend()

plt.show()


# =========================================================
# 3. Combined Plot
# =========================================================

plt.figure(figsize=(10,8))

# Distance
plt.subplot(2,1,1)
plt.plot(df['time'].values, df['distance_error'].values, color='blue')
plt.title('Distance Error')
plt.grid(True)

# Angle
plt.subplot(2,1,2)
plt.plot(df['time'].values, df['angle_error'].values, color='red')
plt.title('Angle Error')
plt.grid(True)

plt.tight_layout()
plt.show()


# =========================================================
# Analysis Guide
# =========================================================

print("\n=== Analysis Guide ===")
print("1. Distance Error가 0으로 수렴하는지 확인")
print("2. Angle Error가 안정적으로 0 근처로 가는지 확인")
print("3. 진동이 크면 Kp 값이 과한 것")
print("4. 너무 느리면 Kp 값이 작은 것")