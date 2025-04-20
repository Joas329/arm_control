import numpy as np
from sympy import symbols, sin, cos, Matrix, atan2, sqrt, acos, lambdify
from numpy.linalg import norm

class IKSPSolver():
    def __init__(self):
        self.desiredPose = np.array([
                                [1, 0, 0, 0.5],
                                [0, 1, 0, 0.2],
                                [0, 0, 1, 0.3],
                                [0, 0, 0, 1]
                            ])
        self.joint_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        theta1, theta2, theta3, theta4, theta5, theta6 = symbols(
            'theta1 theta2 theta3 theta4 theta5 theta6',
            real=True
        )

        self.dh_params = np.array([
            (0.0,     1.5707,    0.0,       theta1),    # Joint 1
            (0.0,     0.0,       -0.063263, theta2),    # Joint 2
            (0.4566,  0.0,       0.0,       theta3),    # Joint 3
            (0.11713, 0.0,       0.063262,  theta4),    # Joint 4
            (0.43497, 0.0,       0.0605,    theta5),    # Joint 5
            (0.082,   0.0,       -0.0605,   theta6)     # Joint 6
        ])

        # *************** Forward Kinematics *************** #
        self.Transformations_list = []
        T_curr = Matrix.eye(4)
        self.Transformations_list.append(T_curr)

        for (a_i, alpha_i, d_i, th_i) in self.dh_params:
            T_i = self.dh_transform(a_i, alpha_i, d_i, th_i)
            T_curr = T_curr * T_i
            self.Transformations_list.append(T_curr)

        # *************** Inverse Kinematics Equations *************** #
        self.ik_equations = self.get_ik_equations()
        self.print_equation('theta6')

    def dh_transform(sefl, a, alpha, d, theta):
        return Matrix([
            [ cos(theta),           -sin(theta),           0,             a ],
            [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d ],
            [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d ],
            [ 0,                     0,                      0,            1 ]
        ])

    def print_equation(self, joint_name):
        equations = self.get_ik_equations()
        if joint_name in equations:
            print(f"{joint_name} = {equations[joint_name]}")
        else:
            print(f"Invalid joint name: '{joint_name}'. Valid options: {list(equations.keys())}")

    def get_ik_equations(self):
        # Step 1: Get symbolic wrist center
        T06 = self.Transformations_list[6]
        R06 = T06[:3, :3]
        P06 = T06[:3, 3]
        z = Matrix([0, 0, 1])
        d6 = -0.0605
        Pw = P06 - d6 * R06 @ z

        x, y, z_wc = Pw[0], Pw[1], Pw[2]

        # θ1
        theta1_expr = atan2(y, x)

        # Planar projection from joint 2's perspective
        d2 = -0.063263
        a2 = 0.4566
        a3 = 0.11713
        r = sqrt(x**2 + y**2)
        z2 = z_wc - d2
        R_len = sqrt(r**2 + z2**2)

        # θ3 via cosine law
        cos_theta3 = (R_len**2 - a2**2 - a3**2) / (2 * a2 * a3)
        theta3_expr = acos(cos_theta3)

        # θ2 from triangle
        alpha = atan2(z2, r)
        beta = atan2(a3 * sin(theta3_expr), a2 + a3 * cos(theta3_expr))
        theta2_expr = alpha - beta

        # T03
        T01 = self.dh_transform(0.0, 1.5707, 0.0, theta1_expr)
        T12 = self.dh_transform(0.0, 0.0, d2, theta2_expr)
        T23 = self.dh_transform(a2, 0.0, 0.0, theta3_expr)
        T03 = T01 * T12 * T23
        R03 = T03[:3, :3]
        R36 = R03.T * R06

        # θ5 from R36
        theta5_expr = acos(R36[2, 2])
        theta4_expr = atan2(R36[1, 2], R36[0, 2])
        theta6_expr = atan2(R36[2, 1], -R36[2, 0])

        return {
            'theta1': theta1_expr,
            'theta2': theta2_expr,
            'theta3': theta3_expr,
            'theta4': theta4_expr,
            'theta5': theta5_expr,
            'theta6': theta6_expr,
            'wrist_center': Pw
        }

    def solve_for_joint_x(self, desired_3d_pose, joint_name):
        ik_eqs = self.get_ik_equations()

        if joint_name not in ik_eqs:
            raise ValueError(f"Invalid joint name '{joint_name}'. Use one of: {list(ik_eqs.keys())}")

        R06 = desired_3d_pose[:3, :3]
        P06 = desired_3d_pose[:3, 3]
        d6 = -0.0605
        z = np.array([0, 0, 1])
        wrist_center = P06 - d6 * R06 @ z
        x_val, y_val, z_val = wrist_center

        x, y, z = symbols("x y z")

        joint_expr = ik_eqs[joint_name]
        joint_func = lambdify((x, y, z), joint_expr, modules="numpy")

        result = joint_func(x_val, y_val, z_val)
        return float(result)