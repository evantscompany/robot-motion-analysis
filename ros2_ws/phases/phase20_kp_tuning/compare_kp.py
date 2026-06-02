import pandas as pd
import matplotlib.pyplot as plt

# =========================================================
# PHASE 20: Kp 비교 분석
# =========================================================
# 목적:
# - 서로 다른 Kp 값의 제어 성능 비교
# - distance_error / angle_error 수렴 속도 분석
# =========================================================

# ==============================
# CSV 로드
# ==============================

kp_5 = r"/home/msm031/robot_motion_analysis/ros2_ws/phases/phase18_error_logger/logs/tracker_kp_05.csv"
kp_10 = r"/home/msm031/robot_motion_analysis/ros2_ws/phases/phase18_error_logger/logs/tracker_kp_10.csv"
kp_20 = r"/home/msm031/robot_motion_analysis/ros2_ws/phases/phase18_error_logger/logs/tracker_kp_20.csv"

df_05 = pd.read_csv(kp_5)
df_10 = pd.read_csv(kp_10)
df_20 = pd.read_csv(kp_20)

# ==============================
# 1. Distance Error 비교
# ==============================

plt.figure(figsize=(10, 5))

plt.plot(
    df_05['time'].values,
    df_05['distance_error'].values,
    label='Kp=0.5'
)

plt.plot(
    df_10['time'].values,
    df_10['distance_error'].values,
    label='Kp=1.0'
)

plt.plot(
    df_20['time'].values,
    df_20['distance_error'].values,
    label='Kp=2.0'
)

plt.title('Distance Error Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Distance Error (m)')
plt.grid(True)
plt.legend()

plt.show()

# ==============================
# 2. Angle Error 비교
# ==============================

plt.figure(figsize=(10, 5))

plt.plot(
    df_05['time'].values,
    df_05['angle_error'].values,
    label='Kp=0.5'
)

plt.plot(
    df_10['time'].values,
    df_10['angle_error'].values,
    label='Kp=1.0'
)

plt.plot(
    df_20['time'].values,
    df_20['angle_error'].values,
    label='Kp=2.0'
)

plt.title('Angle Error Comparison')
plt.xlabel('Time (s)')
plt.ylabel('Angle Error (rad)')
plt.grid(True)
plt.legend()

plt.show()

# ==============================
# 3. 결론 가이드
# ==============================

print("\n=== CONTROL ANALYSIS ===")
print("Kp 0.5 → 느리지만 안정적")
print("Kp 1.0 → 균형")
print("Kp 2.0 → 빠르지만 진동 가능")