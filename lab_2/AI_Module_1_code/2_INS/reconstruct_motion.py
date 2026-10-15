"""课堂实验：根据六轴 IMU 数据完成简化 INS 轨迹重构。

教学目标
--------
同学们需要根据课堂讲解，补全 6 个关键步骤：
1. 角增量计算（由角速度积分得到）
2. 姿态更新（由上一时刻姿态乘增量旋转）
3. 机体系加速度变换到世界系
4. 重力补偿
5. 速度更新
6. 位置更新

说明
----
- 本实验使用提供的数据文件，重点练习 INS 的“姿态 -> 加速度 -> 速度 -> 位置”计算流程。
- 代码已经提供了数据读取、绘图、结果保存等外围功能；同学们只需填写核心公式。
- 默认模式只使用加速度计与陀螺仪估计姿态，按照课件公式完成角速度积分。
- 可选参数 --quaternion：直接使用 IM948 内置四元数作为姿态，不执行自编的陀螺仪姿态积分。
"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
from scipy.spatial.transform import Rotation
from scipy.signal import butter, sosfiltfilt
from scipy.ndimage import median_filter, binary_opening
from matplotlib.ticker import MaxNLocator


def integrate_trapezoid(values, t):
    """仅用于零速约束后的速度->位置积分：相邻速度取平均。

    注：若区间内加速度恒定，速度是线性的，速度梯形积分恰好等于
        p[k] = p[k-1] + v[k-1]*dt + 0.5*a[k-1]*dt**2
    因此这里与课堂位置公式并不矛盾。
    """
    dt = np.diff(t)[:, None]
    increments = 0.5 * (values[:-1] + values[1:]) * dt
    return np.vstack([np.zeros(3), np.cumsum(increments, axis=0)])


def ins_integrate(acc_world, t):
    """课堂重点：根据课件的离散 INS 公式，计算世界系速度和相对位置。

    已知：
        - acc_world[k] : 第 k 帧世界系、去重力后的加速度，单位 m/s^2
        - t[k]         : 第 k 帧时间戳，单位 s

    要求根据课件公式填写：
        v[k] = v[k-1] + a[k-1] * dt
        p[k] = p[k-1] + v[k-1] * dt + 0.5 * a[k-1] * dt^2

    注意：
        - 本实验只重构“相对轨迹”，因此初值设为 v[0]=0, p[0]=0
        - 位置更新必须使用上一时刻速度 v[k-1]，不能直接写成 v[k] * dt
    """
    velocity = np.zeros_like(acc_world, dtype=float)
    position = np.zeros_like(acc_world, dtype=float)

    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        a_kml = acc_world[k - 1]

        # ===== 关键步骤 5：速度更新 =====
        # 对应课件公式：v[k] = v[k-1] + a[k-1] * dt
        # 可用变量：velocity[k-1], a_km1, dt
        # 请在下一行补全右侧表达式
        velocity[k] = velocity[k-1] + a_kml * dt

        # ===== 关键步骤 6：位置更新 =====
        # 对应课件公式：p[k] = p[k-1] + v[k-1]*dt + 0.5*a[k-1]*dt^2
        # 可用变量：position[k-1], velocity[k-1], a_km1, dt
        # 请在下一行补全右侧表达式
        position[k] = position[k-1] + velocity[k-1] * dt + 0.5 * a_kml * (dt ** 2)

    return velocity, position


def get_attitude(rows, acc, gyro, t, initial, gyro_only):
    """获得每个时刻的姿态 R_nb（把设备系 b 的向量变换到参考系 n）。

    参数说明
    --------
    rows      : 原始 JSON 数据行
    acc, gyro : 原始加速度/角速度数据
    t         : 时间戳（秒）
    initial   : 初始静止区间布尔掩码
    gyro_only : 是否只使用加速度+陀螺仪估计姿态

    返回
    ----
    Rotation 对象，可调用：
        rotation.apply(v)        # 把设备系向量 v 旋转到世界系
        Rotation.from_rotvec(x)  # 根据旋转向量 x 构造一个增量旋转
    """
    if not gyro_only:
        # 直接使用模块输出的四元数（w, x, y, z）
        q = np.array([r['quat'] for r in rows], dtype=float)
        if np.any(np.linalg.norm(q, axis=1) < .1):
            raise ValueError('存在无效四元数')
        q /= np.linalg.norm(q, axis=1)[:, None]
        # SciPy 的顺序是 (x, y, z, w)
        return Rotation.from_quat(q[:, [1, 2, 3, 0]])

    # -------- 默认模式：使用 acc + gyro 自行估计姿态 --------
    # 初始静止时：加速度平均值可以给出“向上”的方向，但不能确定绝对航向。
    up = acc[initial].mean(axis=0)
    rotation = Rotation.align_vectors([[0, 0, 1]], [up / np.linalg.norm(up)])[0]

    # 扣除初始静止段陀螺零偏，并转成 rad/s
    omega = np.deg2rad(gyro - gyro[initial].mean(axis=0))

    quaternions = [rotation.as_quat()]
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]

        # ===== 关键步骤 1：角增量计算 =====
        # 对应课件公式：phi_k = ∫ omega dt
        # 这里使用中值/梯形近似：0.5*(omega[i-1] + omega[i]) * dt
        # 可用变量：omega[i-1], omega[i], dt
        delta_angle = 0.5 * (omega[i-1] + omega[i]) * dt

        # ===== 关键步骤 2：姿态更新 =====
        # 对应课件公式：R_k = R_{k-1} * Exp(phi_k)
        # SciPy 中可写为：rotation * Rotation.from_rotvec(delta_angle)
        # 提示：Rotation.from_rotvec() 接收旋转向量（单位 rad）
        rotation = rotation * Rotation.from_rotvec(delta_angle)

        quaternions.append(rotation.as_quat())

    return Rotation.from_quat(quaternions)


def main():
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', nargs='?', type=Path,
                        default=folder / 'data/imu_20261013_111116_663752.json')
    parser.add_argument('--quaternion', action='store_true',
                        help='可选：使用 IM948 内置四元数；默认只用加速度计与陀螺仪积分姿态')
    parser.add_argument('--rest', type=float, default=3, help='已知起止静止时长，秒')
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args()
    if args.no_show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK JP', 'DejaVu Sans']
    plt.rcParams['axes.unicode_minus'] = False

    rows = json.loads(args.file.read_text(encoding='utf-8-sig'))
    ms = np.array([r['ms'] for r in rows], dtype=float)
    t = (ms - ms[0]) / 1000
    acc = np.array([r['acc'] for r in rows])
    gyro = np.array([r['gyro'] for r in rows])
    dt = np.diff(t)
    if len(t) < 60 or np.any(dt <= 0) or not 0 < 2 * args.rest < t[-1]:
        raise ValueError('需要足够数据、递增时间戳和互不重叠的起止静止区间')
    fs = 1 / np.median(dt)
    if np.max(np.abs(dt - np.median(dt))) > .05 * np.median(dt):
        raise ValueError('低通滤波要求均匀采样；这份数据需先重采样')
    initial, final = t < args.rest, t >= t[-1] - args.rest
    if np.max(np.linalg.norm(gyro[initial | final], axis=1)) > 2:
        raise ValueError('指定的起止静止区间有明显旋转，请修改 --rest')
    if np.any(np.std(acc[initial], axis=0) > .05) or np.any(np.std(acc[final], axis=0) > .05):
        raise ValueError('指定的静止区间加速度不稳定，请检查数据及 --rest')

    # 1) 姿态估计：默认调用学生填写的陀螺仪积分步骤；加 --quaternion 才使用模块四元数。
    rotation = get_attitude(rows, acc, gyro, t, initial, gyro_only=not args.quaternion)

    # 2) 重力大小估计：使用初始静止段加速度模长的平均值
    gravity = np.linalg.norm(acc[initial], axis=1).mean()

    # ===== 关键步骤 3：机体系 -> 世界系 坐标变换 =====
    # 对应课件公式中的 R * f^b
    # 提示：可直接使用 rotation.apply(acc)
    acc_world_before_gravity = rotation.apply(acc)

    # ===== 关键步骤 4：重力补偿 =====
    # 对应课件公式：a^n = R_nb * f^b + g^n
    # 在本代码中，世界系 z 轴向上，所以 g^n = [0, 0, -gravity]
    world_acc = acc_world_before_gravity + np.array([0, 0, -gravity])

    # 起始静止段残差修正：这是附加近似处理，不属于基本 INS 公式
    residual_bias = world_acc[initial].mean(axis=0)
    world_acc -= residual_bias

    # 尖峰替换 + 低通滤波：用于减小噪声影响，不属于基本 INS 公式
    median = median_filter(world_acc, size=(7, 1), mode='nearest')
    mad = median_filter(np.abs(world_acc - median), size=(7, 1), mode='nearest')
    spike_mask = np.abs(world_acc - median) > np.maximum(5 * 1.4826 * mad, .6)
    clean = np.where(spike_mask, median, world_acc)
    cutoff = 4.0
    if cutoff >= fs / 2:
        raise ValueError('4 Hz 截止频率过高，请降低 cutoff 或提高采样率')
    sos = butter(3, cutoff, fs=fs, output='sos')
    filtered = sosfiltfilt(sos, clean, axis=0)

    # 3) 根据课堂公式：加速度 -> 速度 -> 位置
    velocity_raw, position_raw = ins_integrate(filtered, t)

    # 4) 候选静止段检测 + 简化零速约束（拓展内容，不要求学生填写）
    gyro_bias = gyro[initial].mean(axis=0)
    stationary = (np.linalg.norm(gyro - gyro_bias, axis=1) < 1.5)
    stationary &= np.linalg.norm(filtered, axis=1) < .12
    stationary = binary_opening(stationary, structure=np.ones(round(.5 * fs)), border_value=1)
    stationary[initial | final] = True
    # 为了保持代码完整性，零速约束后的结果这里仍沿用原有过程：
    # 先把课堂公式直接积分得到的速度作为基础，再在静止段近似拉回 0。
    velocity = velocity_raw.copy()
    velocity[stationary] = 0.0
    position = integrate_trapezoid(velocity, t)

    # 5) 绘图与结果保存
    fig = plt.figure(figsize=(12, 9))
    ax_v = fig.add_subplot(221)
    ax_p = fig.add_subplot(222)
    ax_xy = fig.add_subplot(223)
    ax_3d = fig.add_subplot(224, projection='3d')
    colors = ['tab:blue', 'tab:orange', 'tab:green']
    for axis, name in enumerate('XYZ'):
        ax_v.plot(t, velocity[:, axis], color=colors[axis], label=name)
        ax_p.plot(t, position_raw[:, axis], '--', color=colors[axis], alpha=.5, label=f'{name} 直接积分')
        ax_p.plot(t, position[:, axis], color=colors[axis], label=f'{name} 零速修正')
    ax_v.plot(t, np.linalg.norm(velocity, axis=1), 'k', lw=1, label='速率 |v|')
    ax_v.set(title='参考系速度（含简化零速修正）', xlabel='时间 / s', ylabel='速度 / (m/s)')
    ax_p.set(title='相对位置与积分漂移对照', xlabel='时间 / s', ylabel='位置 / m')
    ax_xy.plot(position[:, 0], position[:, 1])
    ax_xy.scatter(*position[0, :2], c='green', label='起点', zorder=3)
    ax_xy.scatter(*position[-1, :2], c='red', label='终点', zorder=3)
    ax_xy.set(title='XY 相对轨迹', xlabel='X / m', ylabel='Y / m')
    ax_xy.axis('equal')
    ax_3d.plot(*position.T)
    ax_3d.scatter(*position[0], c='green')
    ax_3d.scatter(*position[-1], c='red')
    ax_3d.set(title='三维相对轨迹', xlabel='X / m', ylabel='Y / m', zlabel='Z / m')
    center = (position.min(axis=0) + position.max(axis=0)) / 2
    radius = max(np.ptp(position, axis=0).max() / 2, .001)
    for setter, c in zip([ax_3d.set_xlim, ax_3d.set_ylim, ax_3d.set_zlim], center):
        setter(c - radius, c + radius)
    ax_3d.set_box_aspect((1, 1, 1))
    for axis in (ax_3d.xaxis, ax_3d.yaxis, ax_3d.zaxis):
        axis.set_major_locator(MaxNLocator(3))
    ax_3d.tick_params(labelsize=8, pad=0)
    for ax in (ax_v, ax_p, ax_xy):
        ax.grid(alpha=.3)
        ax.legend(fontsize=8)
    method = 'quaternion' if args.quaternion else 'gyro_only'
    fig.suptitle(f'IM948 · {method} · 估计结果（课堂实验版）')
    fig.tight_layout()

    output = folder / 'results' / f'{args.file.stem}_{method}_student'
    output.parent.mkdir(exist_ok=True)
    result = {
        'source_file': args.file.name,
        'frames': len(rows),
        'method': method,
        'initial_final_rest_s': args.rest,
        'gravity_m_s2': float(gravity),
        'cutoff_hz': cutoff,
        'spike_component_count': int(spike_mask.sum()),
        'residual_bias_world_m_s2': residual_bias.tolist(),
        't_s': t.tolist(),
        'stationary': stationary.tolist(),
        'acc_world_before_filter_m_s2': world_acc.tolist(),
        'acc_world_m_s2': filtered.tolist(),
        'velocity_raw_m_s': velocity_raw.tolist(),
        'position_raw_m': position_raw.tolist(),
        'velocity_m_s': velocity.tolist(),
        'position_m': position.tolist(),
        'speed_m_s': np.linalg.norm(velocity, axis=1).tolist(),
    }
    output.with_suffix('.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    fig.savefig(output.with_suffix('.png'), dpi=150)
    print(f'直接积分水平终点偏移：{np.linalg.norm(position_raw[-1, :2]):.4f} m')
    print(f'简化零速修正水平终点偏移：{np.linalg.norm(position[-1, :2]):.4f} m')
    print(f'估计最大速率：{np.linalg.norm(velocity, axis=1).max():.4f} m/s')
    print(f'结果：{output}.png / .json')
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == '__main__':
    main()
