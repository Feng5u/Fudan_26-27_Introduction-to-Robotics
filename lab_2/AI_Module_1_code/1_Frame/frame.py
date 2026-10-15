"""INS 课堂练习：Body <-> Inertial 坐标变换与重力补偿。

任务：填好 TODO 1~6 后运行 python frame.py。
约定：右手系，列向量；R_IB 将 Body 向量转换到惯性系；姿态角采用 ZYX 姿态角约定 (phi: X, theta: Y, psi: Z)；角度单位为度。
"""
import numpy as np

def euler_to_rotation(phi_deg, theta_deg, psi_deg):
    phi, theta, psi = np.deg2rad([phi_deg, theta_deg, psi_deg])
    # TODO 1: 根据 phi 构造绕 X 轴的旋转矩阵 Rx
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    # TODO 2: 根据 theta 构造绕 Y 轴的旋转矩阵 Ry
    Ry = np.array([
        [np.cos(theta), 0, np.sin(theta)],
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    # TODO 3: 根据 psi 构造绕 Z 轴的旋转矩阵 Rz
    Rz = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    # TODO 4: 按 ZYX 约定组合：R_IB = Rz @ Ry @ Rx
    R_IB = Rz @ Ry @ Rx
    return R_IB

def convert_measurement(R_IB, f_B, g_I):
    # TODO 5: 将 Body 系比力 f_B 转到惯性系
    f_I = R_IB @ f_B
    # TODO 6: 按 a_I = f_I + g_I 进行重力补偿
    a_I = f_I + g_I
    return f_I, a_I

if __name__ == '__main__':
    g_I = np.array([0., 0., -9.81])
    R = euler_to_rotation(phi_deg=60, theta_deg=30, psi_deg=0)
    # 静止 IMU 的理想测量，随姿态自动计算，不能改变角度却固定测量值
    f_B = R.T @ (-g_I)
    f_I, a_I = convert_measurement(R, f_B, g_I)
    print('Body 系比力:', np.round(f_B, 4))
    print('惯性系比力:', np.round(f_I, 4))
    print('重力补偿后的运动加速度:', np.round(a_I, 6))
    assert np.allclose(R.T@R, np.eye(3), atol=1e-8),'旋转矩阵应当正交'
    assert np.isclose(np.linalg.det(R), 1, atol=1e-8),'旋转矩阵行列式应等于1'
    assert np.allclose(a_I, np.zeros(3), atol=1e-8),'静止 IMU 补偿后应为零'
    print('全部检查通过')
