import numpy as np

def Inv_Dyn2(theta, theta_d, theta_dd, f_external=None):
    """5自由度机械臂 Newton–Euler 逆动力学"""

    # ---------------- 参数定义 ----------------
    th = np.zeros(6)
    d = np.zeros(6)
    a = np.zeros(6)
    alp = np.zeros(6)
    th[0] = theta[0]
    th[1] = theta[1]+176.87/180*np.pi
    th[2] = theta[2]-160.51/180*np.pi
    th[3] = theta[3]-16.36/180*np.pi
    th[4] = theta[4]-np.pi/2
    th[5] = theta[5]

    # base_link 初值
    w0   = np.zeros(3)
    wd0  = np.zeros(3)
    vd0  = np.array([0, 0, 9.8])   # 重力加速度
    z = np.array([0, 0, 1])

    # 质量、惯性张量、质心（你的数据表直接塞到数组里）
    m = np.array([
    1.187,   # Link1
    1.3827,   # Link2
    0.72,   # Link3
    0.392,  # Link4
    0.333,   # Link5
    0.362     # Link6
])

    I = [np.eye(3) * 1e-6 for _ in range(6)]
    
    pc = [
    np.array([0, 0.005, -0.004]),
    np.array([0.216, -0.0029239, 0.00064686]),
    np.array([0.1574, 0.01, 0.0003]),
    np.array([0.0629, 0.054, -0.0051302]),
    np.array([-1.7702E-14, 0, 0]),
    np.array([0, -0.003, 0.05])
    ]


    # ---------------- 计算T、R、p ----------------
    R = [None]*6

    # Joint 1, alpha=0
    ct, st = np.cos(th[0]), np.sin(th[0])
    ct, st = np.cos(th[0]), np.sin(th[0])
    R[0] = np.array([
        [ ct, -st, 0],
        [ st,  ct, 0],
        [  0,   0, 1]
    ])

    # Joint 2, alpha=pi/2, cosα=0, sinα=1
    ct, st = np.cos(th[1]), np.sin(th[1])
    R[1] = np.array([
        [ ct, -st, 0],
        [ 0,   0, -1],
        [ st,  ct, 0]
    ])

    # Joint 3, alpha=0, cosα=1, sinα=0
    ct, st = np.cos(th[2]), np.sin(th[2])
    R[2] = np.array([
        [ ct, -st, 0],
        [ st,  ct, 0],
        [ 0,   0, 1]
    ])

    # Joint 4, alpha=0, cosα=1, sinα=0
    ct, st = np.cos(th[3]), np.sin(th[3])
    R[3] = np.array([
        [ ct, -st, 0],
        [ st,  ct, 0],
        [ 0,   0, 1]
    ])

    # Joint 5, alpha=-pi/2, cosα=0, sinα=-1
    ct, st = np.cos(th[4]), np.sin(th[4])
    R[4] = np.array([
        [ ct, -st, 0],
        [ 0,  0, 1],
        [-st, -ct, 0]
    ])

    # Joint 6, alpha=-pi/2, cosα=0, sinα=-1
    ct, st = np.cos(th[5]), np.sin(th[5])
    R[5] = np.array([
        [ ct, -st, 0],
        [ 0,  0, 1],
        [-st, -ct, 0]
    ])

    Rt = [Ri.T for Ri in R]
    p = [
    np.array([0.0, 0.0, 0.1226]),
    np.array([0.0, 0.0, 0.0]),
    np.array([0.320, 0.0, 0.0]),
    np.array([0.2587, 0.0, 0.0]),
    np.array([0.07143, 0.0, 0.0]),
    np.array([0.0, 0.026, 0.0]),
    np.array([0.0, 0.0, 0.0])]

    # ---------------- Forward recursion ----------------
    w, wd, vd, F, N = [w0], [wd0], [vd0], [], []
    for i in range(6):
        wi  = Rt[i] @ w[i] + theta_d[i]*z
        wdi = Rt[i] @ wd[i] + np.cross(Rt[i] @ w[i], z*theta_d[i]) + theta_dd[i]*z
        vdi = Rt[i] @ (np.cross(wd[i], p[i]) + np.cross(w[i], np.cross(w[i], p[i])) + vd[i])
        vcdi = np.cross(wdi, pc[i]) + np.cross(wi, np.cross(wi, pc[i])) + vdi
        Fi = m[i]*vcdi
        Ni = I[i] @ wdi + np.cross(wi, I[i] @ wi)

        w.append(wi); wd.append(wdi); vd.append(vdi)
        F.append(Fi); N.append(Ni)

    # ---------------- Backward recursion ----------------
    if f_external is None:
        f_next, n_next = np.zeros(3), np.zeros(3)
    else:
        f_next = -np.array(f_external[:3])
        n_next = -np.array(f_external[3:])

    tau = np.zeros(6)
    for i in reversed(range(6)):
        fi = (R[i+1] @ f_next if i < 5 else f_next) + F[i]
        ni = N[i] + (R[i+1] @ n_next if i < 5 else n_next) \
             + np.cross(pc[i], F[i]) + np.cross(p[i+1], (R[i+1] @ f_next if i < 4 else f_next))
        tau[i] = ni @ z
        f_next, n_next = fi, ni

    return tau

def friction_torque(theta_d, friction_params=None):
    """
    计算关节摩擦力矩
    参数：
        theta_d: 关节角速度 [rad/s] - 5x1矩阵
        friction_params: 摩擦参数字典 (可选)
            格式: {'viscous': [fv1, fv2, fv3, fv4, fv5], 
                   'coulomb': [fc1, fc2, fc3, fc4, fc5]}
    返回值：
        tau_friction: 摩擦力矩 [N·m] - 5x1矩阵
    """
    # 默认摩擦参数 (如果没有提供)
    if friction_params is None:
        # 粘性摩擦系数 (N·m·s/rad)
        fv = [0.003232, 0.003391, 0.007291, 0.005000, 0.003000]  # 后两个为默认值
        # 库仑摩擦系数 (N·m)
        fc = [0.046962, 0.041413, 0.047655, 0.060000, 0.040000]  # 后两个为默认值
    else:
        fv = friction_params['viscous']
        fc = friction_params['coulomb']
    
    tau_friction = np.zeros(5)
    
    for i in range(5):
        # 摩擦模型：粘性摩擦 + 库仑摩擦 (总是与运动方向相反)
        # 注意：摩擦力矩应该总是阻碍运动，因此取负号
        viscous = -fv[i] * theta_d[i]                    # 粘性摩擦：与角速度成正比，方向相反
        coulomb = -fc[i] * np.sign(theta_d[i])          # 库仑摩擦：恒定大小，方向与运动相反
        tau_friction[i] = viscous + coulomb
    
    return tau_friction

def test_random(n_tests=10, seed=42):
    np.random.seed(seed)  # 保证可复现
    for i in range(n_tests):
        # 随机生成关节角度 (-pi ~ pi)，角速度 (-2 ~ 2)，角加速度 (-5 ~ 5)
        theta = np.zeros(6)   # 6个关节角
        qd    = np.zeros(6)   # 6个关节速度
        qdd   = np.zeros(6)   # 6个关节加速度
        tau2 = Inv_Dyn2(theta, qd, qdd)

        print(f"Test #{i+1}")
        print("theta =", np.round(theta, 3))
        print("qd    =", np.round(qd, 3))
        print("qdd   =", np.round(qdd, 3))
        print("tau2  =", np.round(tau2, 6))
        print("-" * 60)


if __name__ == "__main__":

    print(Inv_Dyn2(np.zeros(6), np.zeros(6), np.zeros(6)))
