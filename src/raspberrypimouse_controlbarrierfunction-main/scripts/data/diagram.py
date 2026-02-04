import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 超椭圆参数
a = 0.9  # x轴半径
b = 1.2  # y轴半径
n = 2  # 控制曲线的形状，n越大越接近矩形
angle = 180  # 超级椭圆的旋转角度

# 超级椭圆公式
def super_ellipse(theta, a, b, n):
    x = np.sign(np.cos(theta)) * (np.abs(np.cos(theta)) ** (2 / n)) * a
    y = np.sign(np.sin(theta)) * (np.abs(np.sin(theta)) ** (2 / n)) * b
    return x, y

# 加载 CSV 文件
# data = pd.read_csv("/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/data/datarobot_trajectories.csv")
data = pd.read_csv("/home/robolab/catkin_ws_jia/src/raspberrypimouse_controlbarrierfunction-main/scripts/data/datarobot2.csv")

# 绘制每个机器人的轨迹
fig, ax = plt.subplots()
ax.set_xlim((-4, 4))
ax.set_ylim((-4, 4))
ax.grid(True)

N = len(data.columns) // 2  # 机器人数量
for i in range(N):
    ax.plot(data[f'Robot_{i}_x'], data[f'Robot_{i}_y'], label=f'Robot {i}')
    ax.scatter(data[f'Robot_{i}_x'][0], data[f'Robot_{i}_y'][0], color='red', s=50, marker='o')  # 起点
    ax.scatter(data[f'Robot_{i}_x'].iloc[-1], data[f'Robot_{i}_y'].iloc[-1], color='blue', s=50, marker='x')  # 终点

# 生成超椭圆点
theta = np.linspace(0, 2 * np.pi, 100)
x_super, y_super = super_ellipse(theta, a, b, n)

# 旋转超椭圆
rotation_matrix = np.array([[np.cos(np.radians(angle)), -np.sin(np.radians(angle))],
                            [np.sin(np.radians(angle)), np.cos(np.radians(angle))]])
rotated_super = rotation_matrix @ np.array([x_super, y_super])
x_super_rot, y_super_rot = rotated_super


# 填充超椭圆
ax.fill(x_super_rot, y_super_rot, color="red", alpha=0.3, label="Super-Ellipse Area")  # 填充颜色
# 绘制超椭圆
ax.plot(x_super_rot, y_super_rot, color="red", label="Super-Ellipse")

# 添加图例
ax.legend()
plt.show()