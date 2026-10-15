from dataclasses import dataclass
import numpy as np

np.set_printoptions(precision=4, suppress=True)

@dataclass
class DHLink:
    """改进 DH"""
    a: float
    alpha: float
    d: float
    offset: float

def dh_matrix(link: DHLink, theta: float) -> np.ndarray:
    """改进 DH 的 ^{i-1}_i T"""
    theta += link.offset
    ca, sa = np.cos(link.alpha), np.sin(link.alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,     -st,     0,    link.a],
        [st*ca,  ct*ca,  -sa,  -link.d*sa],
        [st*sa,  ct*sa,   ca,   link.d*ca],
        [0,       0,      0,    1],
    ])

def forward_kinematics(dh_table, q) -> np.ndarray:
    """输入 6 个关节角，返回末端位姿 ^{0}_6 T"""
    T = np.eye(4)
    for link, theta in zip(dh_table, q):
        T = T @ dh_matrix(link, theta)
    return T

def rotm_to_p(T):
    """
    返回 (x, y, z)
    """

    return T[:3, 3]

def rotm_to_rpy(R):
    """
    返回 (rx, ry, rz)
    """
    r11, r12, r13 = R[0, :3]
    r21, r22, r23 = R[1, :3]
    r31, r32, r33 = R[2, :3]

    sy = np.sqrt(r11*r11 + r21*r21)
    singular = sy < 1e-6

    if not singular:
        rx = np.arctan2(r32, r33)
        ry = np.arctan2(-r31, sy)
        rz = np.arctan2(r21, r11)
    else:
        rx = np.arctan2(-r23, r22)
        ry = np.arctan2(-r31, sy)
        rz = 0.0

    return np.rad2deg([rx, ry, rz])

dh_table = [
    DHLink(a=0, alpha=0, d=173.90, offset=0),
    DHLink(a=0, alpha=np.pi/2, d=0, offset=-np.pi/2),
    DHLink(a=-135.00, alpha=0, d=0, offset=0),
    DHLink(a=-120.00, alpha=0, d=88.78, offset=-np.pi/2),
    DHLink(a=0, alpha=np.pi/2, d=95.00, offset=0),
    DHLink(a=0, alpha=-np.pi/2, d=65.50, offset=0),
]

q1_th = [np.deg2rad(x) for x in [-10, 37, -30, 66, -50, 60]]
q1_re = [np.deg2rad(x) for x in [-7.11, 27.86, -22.14, 49.3, -37.0, 44.64]]
q2_th = [0, -1, 1.5, -1, 0.5, np.pi - 3.2]
q2_re = [np.deg2rad(x) for x in [-6.24, 0.08, 16.17, 17.31, -18.8, 35.15]]

q = q2_th

T_end = forward_kinematics(dh_table, q)

Position = rotm_to_p(T_end)
RPY = rotm_to_rpy(T_end)

ans = np.concatenate([Position, RPY])
print(ans)