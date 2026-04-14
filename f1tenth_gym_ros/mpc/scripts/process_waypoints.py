import numpy as np

data = np.loadtxt('/sim_ws/src/mpc/scripts/levine_raw.csv', delimiter=',')
x = data[:, 0]
y = data[:, 1]

# 计算 yaw
yaw = np.zeros(len(x))
for i in range(len(x) - 1):
    dx = x[i+1] - x[i]
    dy = y[i+1] - y[i]
    yaw[i] = np.arctan2(dy, dx)
yaw[-1] = yaw[-2]

# 平滑 yaw（避免突变）
from scipy.ndimage import uniform_filter1d
yaw = np.unwrap(yaw)
yaw = uniform_filter1d(yaw, size=5)

# 计算曲率
kappa = np.zeros(len(x))
for i in range(1, len(x) - 1):
    dx1 = x[i] - x[i-1]; dy1 = y[i] - y[i-1]
    dx2 = x[i+1] - x[i]; dy2 = y[i+1] - y[i]
    cross = dx1 * dy2 - dy1 * dx2
    norm1 = np.hypot(dx1, dy1)
    norm2 = np.hypot(dx2, dy2)
    if norm1 * norm2 < 1e-6:
        kappa[i] = 0.0
    else:
        kappa[i] = 2 * abs(cross) / (norm1 * norm2 * (norm1 + norm2))
kappa[0] = kappa[1]
kappa[-1] = kappa[-2]

# 平滑曲率
kappa = uniform_filter1d(kappa, size=10)

# 速度插值
v_max = 1.5
v_min = 0.5
kappa_max = np.max(kappa) if np.max(kappa) > 0 else 1.0
v = v_max - (kappa / kappa_max) * (v_max - v_min)
v = np.clip(v, v_min, v_max)

out = np.column_stack([x, y, v, yaw])
np.savetxt('/sim_ws/src/mpc/scripts/levine_waypoints_mpc.csv', out, delimiter=',', fmt='%.6f')
print(f"Done! {len(x)} waypoints")
print("yaw range:", yaw.min(), yaw.max())
print("v range:", v.min(), v.max())
