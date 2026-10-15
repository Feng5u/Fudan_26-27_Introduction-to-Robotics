"""课堂练习一：读取 JSON，用 Matplotlib 绘制加速度、角速度和四元数。
运行：python plot_imu.py；指定文件：python plot_imu.py data/你的文件.json
"""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP']
plt.rcParams['axes.unicode_minus'] = False

def main():
    # 默认数据和输出都相对于脚本位置，避免工作目录不同导致找不到文件。
    folder = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', nargs='?', type=Path,
                        default=folder/'data/imu_20261013_111116_663752.json')
    parser.add_argument('--no-show', action='store_true', help='只保存图片，不弹出窗口')
    args = parser.parse_args()
    if args.no_show:
        matplotlib.use('Agg')

    # 1. JSON 数组中的每个对象对应一帧，提取为 N×3 或 N×4 数组。
    rows = json.loads(args.file.read_text(encoding='utf-8-sig'))
    ms = np.array([r['ms'] for r in rows], dtype=float)
    t = (ms-ms[0])/1000                  # 设备相对时间，秒
    acc = np.array([r['acc'] for r in rows])    # 含重力，m/s²
    gyro = np.array([r['gyro'] for r in rows])  # 角速度，°/s
    quat = np.array([r['quat'] for r in rows])  # 四元数，w/x/y/z

    # 2. 三张子图共用时间轴；每一列是一条分量曲线。
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    groups = [(acc, '含重力加速度', 'm/s²', ['X', 'Y', 'Z']),
              (gyro, '角速度', '°/s', ['X', 'Y', 'Z']),
              (quat, '姿态四元数', '无量纲', ['w', 'x', 'y', 'z'])]
    for ax, (values, title, unit, labels) in zip(axes, groups):
        for column, label in enumerate(labels):
            ax.plot(t, values[:, column], label=label, linewidth=1)
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.grid(alpha=.3)
        ax.legend(loc='upper right', ncol=len(labels))
    axes[-1].set_xlabel('设备相对时间 / s')
    fig.suptitle(f'IM948：{len(rows)} 帧，{t[-1]:.2f} 秒')
    fig.tight_layout()

    # 3. 同时保存图片和显示窗口，适合提交课堂练习结果。
    output = folder/'results'/f'{args.file.stem}_curves.png'
    output.parent.mkdir(exist_ok=True)
    fig.savefig(output, dpi=150)
    print(f'数据：{args.file.name}，{len(rows)} 帧，{t[-1]:.3f} 秒')
    print(f'图片：{output}')
    if not args.no_show:
        plt.show()
    plt.close(fig)


if __name__ == '__main__':
    main()
