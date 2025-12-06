import numpy as np
import math
PI = math.pi
from math import atan2, sqrt, acos, pi, sin, cos, atan

from utils.utils_math import rotation_matrix_to_euler, rotation_matrix_to_quaternion, quaternion_to_rotation_matrix

np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)

class PiperArm:
    def __init__(self):
        # DH参数定义（单位：米/弧度）
        # self.alpha = [0, -pi / 2, 0, pi / 2, -pi / 2, pi / 2]  # 扭转角
        # self.a = [0, 0, 0.28503, -0.02198, 0, 0]  # 连杆长度
        # self.d = [0.123, 0, 0, 0.25075, 0, 0.091]  # 连杆偏移
        # self.theta_offset = [0, -172.2135102 * pi / 180, -102.7827493 * pi / 180, 0, 0, 0]  # 初始角度偏移
        # self.theta_offset = [0, -172.241 * pi / 180, -100.78 * pi / 180, 0, 0, 0]  # 初始角度偏移
        # self.l = 0.06 + 0.091 + 0.07 # 夹爪中点 到 joint4
        # self.l = 0

        self.alpha = [0, -pi / 2, 0, pi / 2, -pi / 2, pi / 2]  # 扭转角
        self.a = [0, 0, 0.28503, -0.02198, 0, 0]  # 连杆长度
        self.d = [0.123, 0, 0, 0.25075, 0, 0.091]  # 连杆偏移
        self.theta_offset = [0, -172.22 * pi / 180, -102.78 * pi / 180, 0, 0, 0]  # 初始角度偏移
        self.l = 0
        
        # ========================================
        # 手眼标定参数 (link6 → camera_color_optical_frame)
        # 由 easy_handeye2 标定生成，请勿手动修改
        # ========================================

        # R = np.array([[-0.09,  0.94,  0.32],
        #              [-0.99 ,  -0.11,  0.06],
        #              [0.09, -0.31,  0.95]])

        # self.link6_q_camera = rotation_matrix_to_quaternion(R)   # w x y z

        # # easy_handeye 标定结果 (2025-10-30 第二次标定)
        # # 平移向量 (单位: 米)
        # self.link6_t_camera = [-0.060055, 0.000844, 0.008386]  # X=-6.0cm, Y=+0.08cm, Z=+0.84cm
        
        # # 四元数 (x, y, z, w) - 注意顺序！
        # self.link6_q_camera = np.array([-0.067453, 0.038034, -0.703879, 0.706086])  # x, y, z, w
        
        # 第一次easy_handeye标定 (已废弃 - 坐标系异常)
        # self.link6_q_camera = np.array([-0.078839, 0.060313, -0.692102, 0.714941])
        # self.link6_q_camera = np.array([ 0.714941,-0.078839, 0.060313, -0.692102])
        # self.link6_t_camera = [-0.065534, 0.003982, 0.008868]

        #11.26 17:12标定
        # self.link6_q_camera = np.array([0.7783364732064637,-0.058408666301587905, 0.03886678206510452,-0.6239151668530724])
        # self.link6_t_camera = [-0.07296806342284513, 0.015543204991977594, -0.007126796291521382]
        # self.link6_t_camera = [-0.07, 0.04, 0.04]

        #12.1 23.30标定
        # 当换了没有13度的连接件后的qt
        # translation:
        #   x: -0.06477069558187146
        #   y: -0.026310042772697995
        #   z: 0.0360855967441351
        # rotation:
        #   x: 0.05688362578128418
        #   y: -0.00025517067294333895
        #   z: -0.6821343496991684
        #   w: 0.7290109169048119
        # self.link6_q_camera = np.array([0.7290109169048119, 0.05688362578128418, -0.00025517067294333895, -0.6821343496991684])
        # self.link6_t_camera = [-0.06477069558187146, -0.026310042772697995, 0.0360855967441351]     

        #12.2 18.15标定        
     #         translation: 
     #   x: -0.05542734326232865
     #   y: 0.056368608427934015
     #   z: 0.038633623349149146
     # rotation: 
     #   x: 0.03464173160721756
     #   y: 0.04226705985348467
     #   z: -0.6609403996728951
     #   w: 0.7484458792476772
        # self.link6_q_camera = np.array([0.7484458792476772, 0.03464173160721756, 0.04226705985348467, -0.6609403996728951])
        # self.link6_t_camera = [-0.05542734326232865, 0.056368608427934015, 0.038633623349149146]
# translation: 
#   x: -0.04349580974909609
#   y: -0.030304206057014574
#   z: 0.03978019500779535
# rotation: 
#   x: 0.07329519537884518
#   y: 0.006448010305290408
#   z: -0.7085092267079163
#   w: 0.7018553363530338
        # ✅ 当前使用的标定结果 (link6 → camera_color_optical_frame)
        self.link6_q_camera = np.array([0.7018553363530338, 0.07329519537884518, 0.006448010305290408, -0.7085092267079163])  # [w, x, y, z]
        self.link6_t_camera = [-0.04349580974909609, -0.030304206057014574, 0.03978019500779535]  # [x, y, z] 单位：米
        
        # ========================================
        # 注意：标定后请使用 read_calibration_result.py 读取
        # 新的标定结果并替换上面两行参数
        # ========================================

        # link limitation
        self.link_limits = [[-154, 154], [0, 195], [-175, 0], [-106, 106], [-75, 75], [-100, 100]]

    def limitation_check(self, joints):
        for i in range(6):
            angle = joints[i] / PI * 180
            if (angle < self.link_limits[i][0] or angle > self.link_limits[i][1]):
                return False

        return True

    def dh_transform(self, alpha, a, d, theta):
        """
        计算Denavit-Hartenberg标准参数的4x4齐次变换矩阵

        参数:
        alpha (float): 连杆扭转角（绕x_(i-1)轴的旋转角，弧度）
        a (float): 连杆长度（沿x_(i-1)轴的平移量，米）
        d (float): 连杆偏移量（沿z_i轴的平移量，米）
        theta (float): 关节角（绕z_i轴的旋转角，弧度）

        返回:
        np.ndarray: 4x4齐次变换矩阵
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # 构建标准DH变换矩阵
        transform = np.array([
            [ct, -st, 0, a],
            [ca * st, ca * ct, -sa, -sa * d],
            [sa * st, sa * ct, ca, ca * d],
            [0, 0, 0, 1]
        ])
        return transform

    def forward_kinematics(self, joints):
        T_total = np.eye(4)
        for i in range(6):
            T = self.dh_transform(self.alpha[i], self.a[i], self.d[i], self.theta_offset[i] + joints[i])
            T_total = T_total @ T

        return T_total

    def _rpy_from_R(self, R):
        """旋转矩阵转ZYX欧拉角"""
        sy = -R[2, 0]
        cy = np.sqrt(max(1.0 - sy * sy, 1e-12))
        yaw = np.arctan2(R[1, 0], R[0, 0])
        pitch = np.arctan2(sy, cy)
        roll = np.arctan2(R[2, 1], R[2, 2])
        return np.array([roll, pitch, yaw])

    def forward_kinematics_sub(self, joints, end):
        # 0_T_end
        T_total = np.eye(4)
        for i in range(end):
            print("i alpha a d theta", self.alpha[i], self.a[i], self.d[i], self.theta_offset[i])
            T = self.dh_transform(self.alpha[i], self.a[i], self.d[i], self.theta_offset[i] + joints[i])
            T_total = T_total @ T

        return T_total

    def inverse_kinematics(self, T_target):
        """Pieper解法逆运动学求解"""
        # 计算 joint4 位置   point under base frame [ 0.5  -0.02  0.2   1.  ]
        joint4_p = T_target @ np.array([0, 0, -self.l, 1], dtype=float)
        px, py, pz = joint4_p[0], joint4_p[1], joint4_p[2]

        # px = T_target[0, 3]
        # py = T_target[1, 3]
        # pz = T_target[2, 3]

        # 计算 link1 2 3 角度
        theta1 = atan2(py , px)
        T01 = self.dh_transform(self.alpha[0], self.a[0], self.d[0], theta1)

        # Convert P05 to frame 1
        P15 = np.linalg.inv(T01) @ np.array([px, py, pz, 1])
        x1, z1 = P15[0], P15[2]
        a1, a2 = self.a[2], self.a[3]
        d1, d2 = self.d[2], self.d[3]
        l1 = sqrt(a1 ** 2 + d1 ** 2)
        l2 = sqrt(a2 ** 2 + d2 ** 2)
        l3 = sqrt(x1 ** 2 + z1 ** 2)

        cos_phi3 = (l1**2 + l2**2 - l3**2) / (2.0 * l1 * l2)
        if abs(cos_phi3) > 1:
            print("no ik solution, fail theta 3")
            return False
        phi3 = acos(cos_phi3)
        print("phi3 is ", phi3 / PI * 180)

        phi3 = atan2(sqrt(1 - cos_phi3 ** 2), cos_phi3)
        # print("phi3 is ", phi3 / PI * 180)
        gamma = atan2(abs(self.d[3]), abs(self.a[3]))
        # print("gamma is ", gamma / PI * 180)
        theta3 = -(gamma + phi3) - self.theta_offset[2]


        cos_phi2 = (l1**2 + l3**2 - l2**2) / (2 * l1 * l3)
        if abs(cos_phi2) > 1:
            print("no ik solution, fail theta 2")
            return False
        phi2 = acos(cos_phi2)

        beta = atan(x1 / z1)
        if z1 > 0:
            theta2 = - (PI / 2 + phi2 - beta) - self.theta_offset[1]
        else:
            beta = atan(x1 / abs(z1))
            print("phi2", phi2 / 3.14 * 180)
            print("beta", beta / 3.14 * 180)
            theta2 = - (phi2 - (PI / 2 - beta)) - self.theta_offset[1]

        q_sol = [theta1, theta2, theta3, 0, 0, 0]

        if not self.limitation_check(q_sol):
            print("no feasible solution")
            return False

        # 计算link4 5 6 角度
        T03 = self.forward_kinematics_sub(q_sol , 3)
        R03 = T03[0:3, 0:3]
        R34d = np.array([[1, 0, 0],[0, 0, -1],[0, 1, 0]])
        T34d = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        R03d = R03 @ R34d
        R06 = T_target[0:3, 0:3]
        R36 = R03d.T @ R06

        print("needed R36", R36)
        rxyzs = rotation_matrix_to_euler(R36)

        for rxyz in rxyzs:
            q_sol = [theta1, theta2, theta3, rxyz[0], rxyz[1], rxyz[2]]
            if self.limitation_check(q_sol):
                T_fk = self.forward_kinematics(q_sol)
                print("expect T", T_target)
                print("T_fk", T_fk)
                print("ik result", q_sol)
                return q_sol

        print("no feasible solution")
        return False

    
    def _numeric_jacobian(self, q, eps=1e-6):
        """
        数值雅可比矩阵计算（6x6）
        前3行：位置对关节角的偏导
        后3行：姿态(RPY)对关节角的偏导
        """
        J = np.zeros((6, 6), dtype=float)
        T0 = self.forward_kinematics(q)
        p0 = T0[:3, 3]
        R0 = T0[:3, :3]

        rpy0 = self._rpy_from_R(R0)

        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            T1 = self.forward_kinematics(q + dq)
            p1 = T1[:3, 3]
            R1 = T1[:3, :3]
            rpy1 = self._rpy_from_R(R1)
            J[:3, i] = (p1 - p0) / eps
            J[3:, i] = (rpy1 - rpy0) / eps
        return J

    def clip_to_limits(self, joints):
        """裁剪关节角到合法范围（弧度）"""
        j = np.array(joints, dtype=float).copy()
        for i in range(6):
            lo = np.deg2rad(self.link_limits[i][0])
            hi = np.deg2rad(self.link_limits[i][1])
            j[i] = np.clip(j[i], lo, hi)
        return j

    def inverse_kinematics_damped(self, T_target, initial_guess=None, max_iterations=200, tol_pos=1e-4, tol_ori=1e-3):
        """
        阻尼最小二乘IK（Levenberg-Marquardt风格）
        - 使用数值雅可比
        - 阻尼因子防止奇异性
        - 自适应阻尼+多重启策略
        
        参数:
            T_target: 4x4目标位姿矩阵
            initial_guess: 初始关节角度（弧度），None则使用零位
            max_iterations: 最大迭代次数
            tol_pos: 位置收敛容差（米）
            tol_ori: 姿态收敛容差（弧度，轴向量范数）
        
        返回:
            关节角度列表，或None（失败时）
        """
        Tt = np.array(T_target, dtype=float)
        p_des = Tt[:3, 3].copy()
        R_des = Tt[:3, :3].copy()

        # 初始猜测
        if initial_guess is None:
            q = np.zeros(6, dtype=float)
        else:
            q = np.array(initial_guess, dtype=float).copy()
        q = self.clip_to_limits(q)

        # 权重因子（位置优先）
        w_pos = 1.0
        w_ori = 0.15  # 🔧 进一步降低姿态权重，优先满足位置

        # 自适应阻尼因子
        lam = 5e-3  # 🔧 初始阻尼略高，提高稳定性
        lam_min, lam_max = 1e-4, 1e-1
        step_max = np.deg2rad(3.0)  # 🔧 减小步长避免震荡

        best_q = q.copy()
        best_err = float('inf')

        for iteration in range(max_iterations):
            T = self.forward_kinematics(q)
            p = T[:3, 3]
            R = T[:3, :3]

            # 位置误差
            e_pos = p_des - p
            pos_err_norm = np.linalg.norm(e_pos)

            # 姿态误差（小角度轴向量近似）
            R_err = R.T @ R_des
            e_omega = 0.5 * np.array([
                R_err[2,1] - R_err[1,2],
                R_err[0,2] - R_err[2,0],
                R_err[1,0] - R_err[0,1],
            ])

            # 记录最佳解
            if pos_err_norm < best_err:
                best_err = pos_err_norm
                best_q = q.copy()

            # 检查收敛
            if pos_err_norm < tol_pos and np.linalg.norm(e_omega) < tol_ori:
                print(f"  ✓ 阻尼IK收敛: 迭代{iteration+1}次, 位置误差={pos_err_norm*1000:.3f}mm")
                return q.tolist()

            # 加权误差
            e = np.hstack([w_pos*e_pos, w_ori*e_omega])

            # 计算雅可比
            J = self._numeric_jacobian(q)
            W = np.diag([w_pos, w_pos, w_pos, w_ori, w_ori, w_ori])
            JW = W @ J

            # 阻尼最小二乘求解
            H = JW @ JW.T + (lam**2) * np.eye(6)
            dq = J.T @ W @ np.linalg.solve(H, e)

            # 限制单步变化
            nrm = np.linalg.norm(dq)
            if nrm > step_max and nrm > 1e-12:
                dq *= (step_max / nrm)

            # 更新
            q_new = self.clip_to_limits(q + dq)
            T_new = self.forward_kinematics(q_new)
            new_err = np.linalg.norm(T_new[:3, 3] - p_des)

            # 🔧 自适应阻尼：误差减小则降低阻尼，否则增加
            if new_err < pos_err_norm:
                lam = max(lam * 0.7, lam_min)
                q = q_new
            else:
                lam = min(lam * 1.5, lam_max)

        # 返回最佳解（如果误差可接受）
        if best_err < 0.005:  # 5mm
            print(f"  ⚠️ 阻尼IK达最大迭代，返回最佳解: {best_err*1000:.2f}mm")
            return best_q.tolist()
        else:
            print(f"  ❌ 阻尼IK失败: 最佳误差={best_err*1000:.2f}mm")
            return None

    def inverse_kinematics_refined(self, T_target, initial_guess=None, max_iterations=100, tolerance=1e-6, enable_diversified_seeds=True):
        """
        高精度逆运动学求解：优先使用阻尼最小二乘法（鲁棒），失败时尝试scipy优化
        
        参数:
            T_target: 4x4目标位姿矩阵
            initial_guess: 初始关节角度猜测（如果None，则使用零位）
            max_iterations: 最大迭代次数
            tolerance: 位置容差（米）
            enable_diversified_seeds: 是否启用多样化种子点策略（提高鲁棒性）
        
        返回:
            优化后的关节角度列表，或None（失败时）
        """
        # 🔧 策略1: 优先使用阻尼最小二乘（简洁、鲁棒、自适应）
        result_damped = self.inverse_kinematics_damped(
            T_target, 
            initial_guess=initial_guess,
            max_iterations=max_iterations,
            tol_pos=tolerance,
            tol_ori=1e-3
        )
        
        if result_damped is not None:
            T_check = self.forward_kinematics(result_damped)
            error = np.linalg.norm(T_check[:3, 3] - T_target[:3, 3])
            if error < 0.002:  # 2mm精度
                return result_damped
        
        # 🔧 策略2: 回退到scipy优化（如果需要）
        print(f"  ⚠️ 阻尼IK未达到高精度，尝试scipy优化...")
        from scipy.optimize import least_squares
        
        # 定义关节限位（提前定义，用于种子点裁剪）
        lower_bounds = np.array([-2.967, -2.618, -2.618, -3.054, -1.588, -3.054])
        upper_bounds = np.array([2.967, 2.618, 2.618, 3.054, 1.588, 3.054])
        
        # 准备初始猜测（用于scipy备用方案）
        if initial_guess is None:
            # 尝试解析解（Pieper）
            analytical_sol = self.inverse_kinematics(T_target)
            if analytical_sol is not False and analytical_sol is not None:
                initial_guess = analytical_sol
                # 🔧 关键修复：强制裁剪解析解到限位范围内
                initial_guess = np.clip(np.array(initial_guess), lower_bounds, upper_bounds).tolist()
                print(f"  [调试] 解析IK解已裁剪到限位: {[f'{np.rad2deg(j):.1f}' for j in initial_guess]}°")
            else:
                # 使用安全的默认种子点（远离奇异点）
                initial_guess = [0.0, 1.0, -1.0, 0.0, 0.5, 0.0]  # 典型配置
        
        # 如果阻尼IK得到了接近解，用它作为种子点
        if result_damped is not None:
            initial_guess = result_damped
        
        # 🔧 关键修复：确保种子点在限位内（防止x0 infeasible）
        initial_guess = np.clip(np.array(initial_guess), lower_bounds, upper_bounds).tolist()
        
        # 2. 定义误差函数（位置+姿态）
        def error_function(q):
            T_current = self.forward_kinematics(q.tolist())
            
            # 位置误差（3维）
            pos_error = T_current[:3, 3] - T_target[:3, 3]
            
            # 姿态误差（使用旋转矩阵的差异，转换为3维向量）
            R_current = T_current[:3, :3]
            R_target = T_target[:3, :3]
            R_error = R_current.T @ R_target  # 相对旋转
            
            # 将旋转误差转换为轴角表示（更适合优化）
            trace = np.trace(R_error)
            if trace >= 3.0 - 1e-6:  # 几乎无旋转误差
                rot_error = np.zeros(3)
            else:
                angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
                if angle < 1e-6:
                    rot_error = np.zeros(3)
                else:
                    axis = np.array([
                        R_error[2, 1] - R_error[1, 2],
                        R_error[0, 2] - R_error[2, 0],
                        R_error[1, 0] - R_error[0, 1]
                    ]) / (2 * np.sin(angle))
                    rot_error = axis * angle
            
            # 加权误差（位置误差权重更高）
            return np.concatenate([pos_error * 10.0, rot_error])
        
        # 2. 尝试优化（单次尝试）
        def try_optimize(seed, attempt_label=""):
            """尝试从给定种子点优化"""
            # 🔧 关键修复：确保种子点在限位内
            seed_clipped = np.clip(np.array(seed), lower_bounds, upper_bounds)
            
            # 验证裁剪是否改变了种子点
            if not np.allclose(seed, seed_clipped, atol=1e-6):
                print(f"    ⚠️  种子点{attempt_label}已裁剪到限位")
            
            try:
                result = least_squares(
                    error_function,
                    x0=seed_clipped,
                    bounds=(lower_bounds, upper_bounds),
                    method='trf',
                    ftol=tolerance,
                    xtol=1e-8,
                    max_nfev=max_iterations,
                    verbose=0
                )
            except ValueError as e:
                print(f"    ❌ least_squares失败{attempt_label}: {e}")
                return None, float('inf')
            
            if result.success:
                optimized_q = result.x.tolist()
                T_final = self.forward_kinematics(optimized_q)
                final_pos_error = np.linalg.norm(T_final[:3, 3] - T_target[:3, 3])
                
                if final_pos_error < 0.002:  # 2mm精度
                    print(f"  ✓ 高精度IK成功{attempt_label}: 位置误差={final_pos_error*1000:.3f}mm")
                    print(f"    优化迭代: {result.nfev}次, 最终残差: {result.cost:.2e}")
                    print(f"    关节角度: {np.rad2deg(optimized_q)}")
                    return optimized_q, final_pos_error
                else:
                    return optimized_q, final_pos_error
            
            return None, float('inf')
        
        # 5. 第一次尝试：使用初始猜测
        best_q, best_error = try_optimize(initial_guess)
        if best_q is not None and best_error < 0.002:
            return best_q
        
        # 6. 如果失败且启用多样化种子点，尝试额外的种子点
        if enable_diversified_seeds and (best_q is None or best_error >= 0.002):
            print(f"  ⚠️  初始种子点IK失败/精度不足 (误差={best_error*1000:.1f}mm)，尝试多样化种子点...")
            
            # 生成多样化种子点策略
            seed_strategies = []
            
            # 策略1: 在初始猜测基础上添加小扰动（5次）
            for i in range(5):
                perturbed = np.array(initial_guess) + np.random.uniform(-0.2, 0.2, 6)  # ±11.5度
                perturbed = np.clip(perturbed, lower_bounds, upper_bounds)
                seed_strategies.append((perturbed, f" (扰动#{i+1})"))
            
            # 策略2: 使用零位附近的种子点（远离奇异点的安全配置）
            seed_strategies.append((np.array([0.0, 1.0, -1.0, 0.0, 0.5, 0.0]), " (零位变体#1)"))
            seed_strategies.append((np.array([0.0, 0.8, -0.8, 0.0, 0.3, 0.0]), " (零位变体#2)"))
            seed_strategies.append((np.array([0.0, 1.5, -1.5, 0.0, 0.0, 0.0]), " (零位变体#3)"))
            
            # 策略3: 基于目标位置的启发式种子点
            target_x = T_target[0, 3]
            target_y = T_target[1, 3]
            target_z = T_target[2, 3]
            
            # 根据目标位置估算J1（偏航角）
            j1_est = np.arctan2(target_y, target_x)
            # 根据目标高度估算J2/J3
            if target_z > 0.2:
                seed_strategies.append((np.array([j1_est, 1.2, -0.8, 0.0, 0.5, 0.0]), " (启发式#1)"))
            else:
                seed_strategies.append((np.array([j1_est, 1.5, -1.2, 0.0, 0.5, 0.0]), " (启发式#2)"))
            
            # 尝试所有种子点
            for seed, label in seed_strategies:
                candidate_q, candidate_error = try_optimize(seed, label)
                
                # 更新最佳解
                if candidate_q is not None and candidate_error < best_error:
                    best_q = candidate_q
                    best_error = candidate_error
                    
                    # 如果达到高精度，立即返回
                    if best_error < 0.002:
                        print(f"  ✓ 多样化种子点成功找到高精度解！")
                        return best_q
        
        # 7. 返回最佳结果（可能不满足精度要求）
        if best_q is not None:
            if best_error < 0.002:
                return best_q
            else:
                print(f"  ⚠️ 所有种子点尝试后，最佳精度: {best_error*1000:.2f}mm")
                return best_q
        else:
            print(f"  ❌ 所有IK优化尝试均失败")
            return None


    def get_joint_tf(self, joint_idx, angle):
        """获取指定关节的变换矩阵"""
        transform = self.dh_transform(self.alpha[joint_idx], self.a[joint_idx], self.d[joint_idx], self.theta_offset[joint_idx] + angle)
        return transform

