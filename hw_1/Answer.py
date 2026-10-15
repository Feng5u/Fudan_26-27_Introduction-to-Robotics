# -*- coding: utf-8 -*-
"""五类运动坐标形式机器人运动空间的动画，几何参数可实时调整

运动空间就是"末端能到达的所有位置"的集合，做法很直接：
  1. 把三个关节的取值范围各自均匀分成 N 段；
  2. 对每一组关节值做正运动学，算出末端位置，共 N³ ≈ 2.2 万个点；
  3. 把这些点画成三维散点云 —— 这一团点就是运动空间。

窗口里：左上角单选按钮切换构型，下面五个滑块实时调整连杆尺寸和关节范围，
点云立刻跟着重算；红点是末端当前位置，按正弦在关节范围内摆动；镜头缓慢旋转。

运行：
    python3 workspace_anim.py          打开交互窗口
    python3 workspace_anim.py --save   保存动画 GIF 和五张图到 img/
"""
import argparse
import os
import sys

# 保存时不弹窗口，先切成不依赖显示器的后端（否则 SSH 里跑会报错）
if "--save" in sys.argv:
    import matplotlib
    matplotlib.use("Agg")

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.widgets import RadioButtons, Slider

plt.rcParams["font.sans-serif"] = ["WenQuanYi Micro Hei", "Droid Sans Fallback", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

N = 28                  # 每个关节的采样段数，总点数 = N³ ≈ 2.2 万
HOLD = 32               # 存 GIF 时每种构型停留的帧数
GIF_DPI = 55            # 存 GIF 用的分辨率（点云很密，再高文件就太大了）
IMG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "img")
D2R = np.pi / 180.0
SWEEP = (0.55, 0.83, 1.21)      # 三个关节的摆动频率，取不成整数比


# --------------------------------------------------------------------------
# 正运动学：p 是参数字典，后三个是关节值（回转关节用弧度，移动关节用米）
# --------------------------------------------------------------------------

def fk_cartesian(p, d1, d2, d3):
    """PPP：三个互相正交的移动副，末端位置就是三个行程。"""
    return d1, d2, d3


def fk_cylindrical(p, th, r, z):
    """RPP：绕 z 轴回转 + 沿径向伸缩 + 竖直升降。"""
    return r * np.cos(th), r * np.sin(th), z


def fk_spherical(p, th, beta, r):
    """RRP：绕 z 轴回转 + 俯仰 + 沿手臂伸缩。beta 是相对水平面的仰角。"""
    return (r * np.cos(beta) * np.cos(th),
            r * np.cos(beta) * np.sin(th),
            p["h0"] + r * np.sin(beta))


def fk_scara(p, t1, t2, d3):
    """RRP：两个轴线平行的回转副 + 一个竖直移动副。"""
    a1, a2 = p["a1"], p["a2"]
    return (a1 * np.cos(t1) + a2 * np.cos(t1 + t2),
            a1 * np.sin(t1) + a2 * np.sin(t1 + t2),
            d3)


def fk_puma(p, t1, t2, t3):
    """RRR：腰回转 + 肩俯仰 + 肘俯仰（大臂 a2、小臂 a3）。"""
    a2, a3 = p["a2"], p["a3"]
    r = a2 * np.cos(t2) + a3 * np.cos(t2 + t3)
    z = p["h0"] + a2 * np.sin(t2) + a3 * np.sin(t2 + t3)
    return r * np.cos(t1), r * np.sin(t1), z


# --------------------------------------------------------------------------
# 五种构型
#   sliders：(参数名, 显示标签, 下限, 上限, 初值)，滑块直接给出数值
#   ranges ：由参数字典算出三个关节的取值范围（弧度 / 米）
# --------------------------------------------------------------------------

ROBOTS = [
    dict(name="直角坐标", dof="PPP，3 个移动副", fk=fk_cartesian,
         sliders=[("Lx", "x 行程 Lx (m)", 0.10, 0.80, 0.50),
                  ("Ly", "y 行程 Ly (m)", 0.10, 0.80, 0.40),
                  ("Lz", "z 行程 Lz (m)", 0.10, 0.80, 0.40)],
         ranges=lambda p: ((0, p["Lx"]), (0, p["Ly"]), (0, p["Lz"]))),

    dict(name="圆柱坐标", dof="RPP，1 回转 + 2 移动", fk=fk_cylindrical,
         sliders=[("r0", "最小半径 r0 (m)", 0.00, 0.35, 0.10),
                  ("L", "径向行程 L (m)", 0.10, 0.70, 0.45),
                  ("H", "竖直行程 H (m)", 0.10, 0.70, 0.45),
                  ("Th", "回转半角 ±θ (°)", 30, 180, 180)],
         ranges=lambda p: ((-p["Th"] * D2R, p["Th"] * D2R),
                           (p["r0"], p["r0"] + p["L"]),
                           (0, p["H"]))),

    dict(name="球坐标", dof="RRP，2 回转 + 1 移动", fk=fk_spherical,
         sliders=[("r0", "最小半径 r0 (m)", 0.00, 0.35, 0.10),
                  ("L", "径向行程 L (m)", 0.10, 0.70, 0.45),
                  ("bmin", "仰角下限 βmin (°)", -85, 30, -45),
                  ("bmax", "仰角上限 βmax (°)", -30, 85, 60),
                  ("h0", "肩高 h0 (m)", 0.00, 0.40, 0.30)],
         ranges=lambda p: ((-np.pi, np.pi),
                           (p["bmin"] * D2R, p["bmax"] * D2R),
                           (p["r0"], p["r0"] + p["L"]))),

    dict(name="SCARA", dof="RRP，2 回转 + 1 移动", fk=fk_scara,
         sliders=[("a1", "大臂 a1 (m)", 0.10, 0.60, 0.35),
                  ("a2", "小臂 a2 (m)", 0.10, 0.60, 0.30),
                  ("T2", "肘关节半角 ±θ2 (°)", 15, 180, 150),
                  ("H", "竖直行程 H (m)", 0.05, 0.30, 0.20),
                  ("T1", "腰关节半角 ±θ1 (°)", 30, 180, 180)],
         ranges=lambda p: ((-p["T1"] * D2R, p["T1"] * D2R),
                           (-p["T2"] * D2R, p["T2"] * D2R),
                           (0, p["H"]))),

    dict(name="PUMA 拟人臂", dof="RRR，3 个回转副", fk=fk_puma,
         sliders=[("a2", "大臂 a2 (m)", 0.10, 0.60, 0.35),
                  ("a3", "小臂 a3 (m)", 0.10, 0.60, 0.30),
                  ("h0", "肩高 h0 (m)", 0.05, 0.60, 0.25),
                  ("T2", "肩关节半角 ±θ2 (°)", 15, 180, 140),
                  ("T3", "肘关节半角 ±θ3 (°)", 15, 180, 140)],
         ranges=lambda p: ((-np.pi, np.pi),
                           (-p["T2"] * D2R, p["T2"] * D2R),
                           (-p["T3"] * D2R, p["T3"] * D2R))),
]

DEFAULT = lambda robot: {key: init for key, _, _, _, init in robot["sliders"]}


def workspace_points(robot, p, n=N):
    """关节范围均匀采样，逐点正解，返回可达点云，形状 (M, 3)。"""
    q1, q2, q3 = (np.linspace(lo, hi, n) for lo, hi in robot["ranges"](p))
    Q1, Q2, Q3 = np.meshgrid(q1, q2, q3, indexing="ij")
    X, Y, Z = robot["fk"](p, Q1, Q2, Q3)
    return np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])


def tcp(robot, p, t):
    """让三个关节各按不同频率摆动，末端就会慢慢扫过整个运动空间。"""
    q = [lo + (hi - lo) * 0.5 * (1 + np.sin(f * t + ph))
         for (lo, hi), f, ph in zip(robot["ranges"](p), SWEEP, (0.0, 2.0, 4.0))]
    return robot["fk"](p, *q)


def draw(ax, robot, p, scat=None):
    """画出点云，按点云范围定坐标轴，更新标题。返回新的散点图元。"""
    pts = workspace_points(robot, p)

    if scat is not None:
        scat.remove()
    scat = ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                      s=4, alpha=0.14, color="#3B7DD8", depthshade=False)

    # 三个轴取同样的长度，否则球会被压扁成长方体
    c = (pts.max(0) + pts.min(0)) / 2
    r = (pts.max(0) - pts.min(0)).max() * 0.55
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1), zoom=1.05)

    ax.set_title("%s   %s" % (robot["name"], robot["dof"]), fontsize=13)
    return scat


class App:
    """交互窗口。with_controls=False 时只有动画，用来导 GIF。"""

    def __init__(self, fig, with_controls=True):
        self.fig = fig
        self.k = 0
        self.with_controls = with_controls
        self.scat = None
        self.sliders = []
        self.params = {}

        if with_controls:               # 左边留出放控件的位置
            self.ax = fig.add_axes([0.31, 0.04, 0.67, 0.92], projection="3d")
        else:
            self.ax = fig.add_subplot(projection="3d")
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")
        self.ax.set_zlabel("z (m)")

        self.dot, = self.ax.plot([], [], [], "o", color="#C0392B", ms=7)

        if with_controls:
            self._make_controls()

        self.select(0)

        total = len(ROBOTS) * HOLD
        self.anim = FuncAnimation(fig, self.frame, frames=total,
                                  interval=50, blit=False, repeat=True)

    def _make_controls(self):
        ax_radio = self.fig.add_axes([0.02, 0.72, 0.20, 0.24], frameon=False)
        self.radio = RadioButtons(ax_radio, [r["name"] for r in ROBOTS], active=0)
        for lbl in self.radio.labels:
            lbl.set_fontsize(9)
        self.radio.on_clicked(self.select)

        # 五个固定的滑块槽位，切换构型时重建
        self.slots = [self.fig.add_axes([0.045, 0.60 - i * 0.075, 0.21, 0.022])
                      for i in range(5)]

    def select(self, which):
        """换构型：滑块换成该构型的参数，参数重置成默认值。"""
        self.k = which if isinstance(which, int) else \
            [r["name"] for r in ROBOTS].index(which)
        robot = ROBOTS[self.k]
        self.params = DEFAULT(robot)

        for s in self.sliders:
            s.disconnect_events()
        self.sliders = []

        if self.with_controls:
            for ax_s in self.slots:
                ax_s.clear()
                ax_s.set_visible(False)
            for ax_s, (key, label, lo, hi, init) in zip(self.slots, robot["sliders"]):
                ax_s.set_visible(True)
                s = Slider(ax_s, label, lo, hi, valinit=init)
                s.label.set_fontsize(8)
                s.valtext.set_fontsize(8)
                # matplotlib 默认把标签放在滑轨左侧并右对齐，会顶出画面外，挪到上方
                s.label.set_position((0.0, 2.2))
                s.label.set_ha("left")
                s.label.set_va("bottom")
                s.on_changed(self.on_slider)
                self.sliders.append(s)

        self.redraw()

    def on_slider(self, _val):
        for s, spec in zip(self.sliders, ROBOTS[self.k]["sliders"]):
            self.params[spec[0]] = s.val
        self.redraw()

    def redraw(self):
        self.scat = draw(self.ax, ROBOTS[self.k], self.params, self.scat)
        self.fig.canvas.draw_idle()

    def frame(self, i):
        if not self.with_controls:      # 导出时自动轮流放五种构型
            k = (i // HOLD) % len(ROBOTS)
            if k != self.k:
                self.select(k)

        x, y, z = tcp(ROBOTS[self.k], self.params, i * 0.06)
        self.dot.set_data([x], [y])
        self.dot.set_3d_properties([z])

        # 镜头在整个动画里正好转一圈
        self.ax.view_init(elev=22, azim=i / (len(ROBOTS) * HOLD) * 360.0)
        return ()


def save():
    os.makedirs(IMG_DIR, exist_ok=True)

    # 每种构型的静态图，给报告当插图
    for robot in ROBOTS:
        fig = plt.figure(figsize=(8, 7.5), facecolor="white")
        ax = fig.add_subplot(projection="3d")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_zlabel("z (m)")
        draw(ax, robot, DEFAULT(robot))
        ax.view_init(elev=22, azim=-60)
        path = os.path.join(IMG_DIR, "工作空间_%s.png" % robot["name"].replace(" ", ""))
        fig.savefig(path, dpi=150, facecolor="white")
        plt.close(fig)
        print("已保存：%s" % path)

    fig = plt.figure(figsize=(8, 7.5), facecolor="white")
    app = App(fig, with_controls=False)
    path = os.path.join(IMG_DIR, "运动空间动画.gif")
    app.anim.save(path, writer=PillowWriter(fps=20), dpi=GIF_DPI)
    plt.close(fig)
    print("已保存：%s" % path)


def main():
    ap = argparse.ArgumentParser(description="五类运动坐标机器人运动空间的动画")
    ap.add_argument("--save", action="store_true", help="保存 GIF 和五张图到 img/")
    args = ap.parse_args()

    if args.save:
        save()
        return 0

    fig = plt.figure(figsize=(11.5, 7.5), facecolor="white")
    app = App(fig)                  # 必须留住引用，否则动画会被回收
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
