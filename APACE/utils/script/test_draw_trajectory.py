import numpy as np
import matplotlib.pyplot as plt

def generate_trajectory(x0_truth, y0_truth, x1_truth, y1_truth, v, dt, num_steps, theta_truth):
    # 存储轨迹
    trajectory_x = [x1_truth]
    trajectory_y = [y1_truth]
    
    x0, y0, x1, y1, theta=x0_truth, y0_truth, x1_truth, y1_truth, theta_truth
    # 生成轨迹
    for _ in range(num_steps):
        # 计算连线向量
        r = np.array([x0 - x1, y0 - y1])
        r_mag = np.linalg.norm(r)  # 计算向量的模长
        if r_mag < 0.01:
            break
        r_unit = r / r_mag  # 单位向量

        # 计算垂直向量
        r_perp = np.array([-r_unit[1], r_unit[0]])  # 垂直方向单位向量

        # 计算速度向量，速度方向与r_unit保持给定角度夹角
        v_vector = v * (np.cos(theta) * r_unit + np.sin(theta) * r_perp)

        # 更新位置
        x1 += v_vector[0] * dt
        y1 += v_vector[1] * dt

        # 记录轨迹
        trajectory_x.append(x1)
        trajectory_y.append(y1)
    trajectory_x = trajectory_x[::-1]
    trajectory_y = trajectory_y[::-1]
    theta = -theta
    x0, y0, x1, y1=x0_truth, y0_truth, x1_truth, y1_truth
    for _ in range(num_steps):
        # 计算连线向量
        r = np.array([x0 - x1, y0 - y1])
        r_mag = np.linalg.norm(r)  # 计算向量的模长
        if r_mag < 0.01:
            break
        r_unit = r / r_mag  # 单位向量

        # 计算垂直向量
        r_perp = np.array([-r_unit[1], r_unit[0]])  # 垂直方向单位向量

        # 计算速度向量，速度方向与r_unit保持给定角度夹角
        v_vector = v * (np.cos(theta) * r_unit + np.sin(theta) * r_perp)

        # 更新位置
        x1 += v_vector[0] * dt
        y1 += v_vector[1] * dt

        # 记录轨迹
        trajectory_x.append(x1)
        trajectory_y.append(y1)
    
    return trajectory_x, trajectory_y

# 示例使用
x0, y0 = 0, 0  # 中心点
x1, y1 = 0, -1  # 轨迹点初始位置
x00, y00 = x0, y0 # truth
x11, y11 = x1, y1 
v = 0.1  # 速度大小
dt = 0.01  # 时间步长
num_steps = 10000  # 迭代次数

# 生成0度轨迹
trajectory_x0, trajectory_y0 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, 0) 
# 生成15度轨迹
trajectory_x15, trajectory_y15 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, np.pi / 12)
# 生成30度轨迹
trajectory_x30, trajectory_y30 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, np.pi / 6) 
# 生成45度轨迹
trajectory_x45, trajectory_y45 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, np.pi / 4) 
# 生成60度轨迹
trajectory_x60, trajectory_y60 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, np.pi / 3) 
# 生成60度轨迹
trajectory_x75, trajectory_y75 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, 5*np.pi / 12) 
# 生成90度轨迹
trajectory_x90, trajectory_y90 = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, np.pi / 2) 
# 生成反向轨迹
# trajectory_x_reverse, trajectory_y_reverse = generate_trajectory(x0, y0, x1, y1, v, dt, num_steps, -theta)

# 绘制轨迹
plt.plot(trajectory_x0, trajectory_y0, label="0")
plt.plot(trajectory_x15, trajectory_y15, label="15")
plt.plot(trajectory_x30, trajectory_y30, label="30")
plt.plot(trajectory_x45, trajectory_y45, label="45")
plt.plot(trajectory_x60, trajectory_y60, label="60")
plt.plot(trajectory_x75, trajectory_y75, label="75")
plt.plot(trajectory_x90, trajectory_y90, label="90")
# plt.plot(trajectory_x_reverse, trajectory_y_reverse, label="反向轨迹")
plt.plot(x00, y00, 'ro', label="feature")
plt.plot(x11, y11, 'bo', label="start")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("trajectory")
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
