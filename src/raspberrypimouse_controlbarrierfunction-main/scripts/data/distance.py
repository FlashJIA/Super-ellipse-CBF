import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 加载机器人轨迹数据
# data = pd.read_csv("/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/data/datarobot_trajectories.csv")
data = pd.read_csv("/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/data/datarobot.csv")
# 提取机器人轨迹
robot_positions = [  # 将每个机器人的 (x, y) 位置存储到列表中
    data[[f'Robot_{i}_x', f'Robot_{i}_y']].values for i in range(len(data.columns) // 2)
]

# 计算每个时间步长的最小距离
time_steps = np.arange(len(data))  # 时间步长
min_distances = []

for t in range(len(data)):
    distances = []
    for i in range(len(robot_positions)):
        for j in range(i + 1, len(robot_positions)):
            # 计算欧几里得距离
            distance = np.linalg.norm(robot_positions[i][t] - robot_positions[j][t])
            distances.append(distance)
    min_distances.append(min(distances))

max_time_steps = 400
if len(time_steps) > max_time_steps:
    time_steps = time_steps[:max_time_steps]
    min_distances = min_distances[:max_time_steps]
# 安全距离
safety_distance = 0.1

# 绘图
plt.figure(figsize=(10, 6))
plt.plot(time_steps, min_distances, label="Minimum distance", color="blue")  # 最小距离曲线
plt.axhline(y=safety_distance, color="red", linestyle="--", label="Safety distance")  # 安全距离线

# 调整 y 轴范围
plt.ylim(0, 1)  # 根据安全距离和最小距离调整 y 轴范围，例如到 0.02m
y_ticks=np.linspace(0,1,11)
plt.yticks(y_ticks)

# 图例和标签
plt.xlabel("Time Steps")
plt.ylabel("Distance (cm)")
plt.title("Minimum Distance vs Safety Distance")
plt.legend()
plt.grid(True)

# 显示图形
plt.show()