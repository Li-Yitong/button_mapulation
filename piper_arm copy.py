import numpy as np
import math
PI = math.pi
from math import atan2, sqrt, acos, pi, sin, cos, atan
import numpy as np
from utils.utils_math import rotation_matrix_to_euler, rotation_matrix_to_quaternion, quaternion_to_rotation_matrix

np.set_printoptions(precision=2)
np.set_printoptions(suppress=True)

class PiperArm:
    def __init__(self):
        # DH参数定义（单位：米/弧度）
        self.alpha = [0, -pi / 2, 0, pi / 2, -pi / 2, pi / 2]  # 扭转角
        self.a = [0, 0, 0.28503, -0.02198, 0, 0]  # 连杆长度
        self.d = [0.123, 0, 0, 0.25075, 0, 0.091]  # 连杆偏移
        self.theta_offset = [0, -172.2135102 * pi / 180, -102.7827493 * pi / 180, 0, 0, 0]  # 初始角度偏移
        # self.theta_offset = [0, -172.241 * pi / 180, -100.78 * pi / 180, 0, 0, 0]  # 初始角度偏移
        # self.l = 0.06 + 0.091 + 0.07 # 夹爪中点 到 joint4
        self.l = 0

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
        self.link6_q_camera = np.array([0.7018553363530338, 0.07329519537884518, 0.006448010305290408, -0.7085092267079163])
        self.link6_t_camera = [-0.04349580974909609, -0.030304206057014574, 0.03978019500779535]


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

    
    def inverse_kinematics_refined(self, T_target, initial_guess=None, max_iterations=100, tolerance=1e-6):
        """
        高精度逆运动学求解：在解析解基础上进行数值优化
        
        参数:
            T_target: 4x4目标位姿矩阵
            initial_guess: 初始关节角度猜测（如果None，则使用解析解）
            max_iterations: 最大迭代次数
            tolerance: 位置容差（米）
        
        返回:
            优化后的关节角度列表，或None（失败时）
        """
        from scipy.optimize import least_squares
        
        # 1. 获取初始解（解析解或用户提供）
        if initial_guess is None:
            initial_guess = self.inverse_kinematics(T_target)
            if initial_guess is False or initial_guess is None:
                # 解析解失败，无法优化
                return None
        
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
            # 使用Rodriguez公式的逆
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
        
        # 3. 定义关节限位约束
        def constraint_bounds():
            # Piper机械臂的关节限位（弧度）
            lower_bounds = np.array([-2.967, -2.618, -2.618, -3.054, -1.588, -3.054])
            upper_bounds = np.array([2.967, 2.618, 2.618, 3.054, 1.588, 3.054])
            return lower_bounds, upper_bounds
        
        # 4. 使用Levenberg-Marquardt算法优化
        lower, upper = constraint_bounds()
        result = least_squares(
            error_function,
            x0=np.array(initial_guess),
            bounds=(lower, upper),
            method='trf',  # Trust Region Reflective (处理边界更好)
            ftol=tolerance,
            xtol=1e-8,
            max_nfev=max_iterations,
            verbose=0
        )
        
        # 5. 验证结果
        if result.success:
            optimized_q = result.x.tolist()
            
            # 检查最终误差
            T_final = self.forward_kinematics(optimized_q)
            final_pos_error = np.linalg.norm(T_final[:3, 3] - T_target[:3, 3])
            
            if final_pos_error < 0.001:  # 1mm精度
                print(f"  ✓ 高精度IK成功: 位置误差={final_pos_error*1000:.3f}mm")
                print(f"    优化迭代: {result.nfev}次, 最终残差: {result.cost:.2e}")
                print(f"    关节角度: {np.rad2deg(optimized_q)}")
                return optimized_q
            else:
                print(f"  ⚠️ 优化收敛但精度不足: {final_pos_error*1000:.2f}mm")
                return optimized_q  # 仍然返回，但警告精度
        else:
            print(f"  ❌ 数值优化失败: {result.message}")
            return initial_guess  # 返回初始解析解


    def get_joint_tf(self, joint_idx, angle):
        """获取指定关节的变换矩阵"""
        transform = self.dh_transform(self.alpha[joint_idx], self.a[joint_idx], self.d[joint_idx], self.theta_offset[joint_idx] + angle)
        return transform

