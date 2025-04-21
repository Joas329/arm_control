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

        x, y, z = symbols('x y z', real=True)
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = symbols(
            'r11 r12 r13 r21 r22 r23 r31 r32 r33', real=True
        )
        self.position_syms = (x, y, z)
        self.R_syms = (r11, r12, r13, r21, r22, r23, r31, r32, r33)

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
        # Position symbols
        x, y, z = symbols('x y z', real=True)
        # Rotation matrix symbols for R^0_6
        r11, r12, r13, r21, r22, r23, r31, r32, r33 = symbols(
            'r11 r12 r13 r21 r22 r23 r31 r32 r33', real=True
        )

        # Extract DH link parameters for IK (following the Pieper method indexing)
        d2 = self.dh_params[1][2]   # d for joint 2
        a2 = self.dh_params[2][0]   # a for joint 3
        a3 = self.dh_params[3][0]   # a for joint 4
        d6 = self.dh_params[5][2]   # d for joint 6

        # Compute wrist-center coordinates
        px = x - d6 * r13
        py = y - d6 * r23
        pz = z - d6 * r33

        # Solve for theta1
        theta1 = atan2(py, px)

        # Planar distance to wrist center in joint-2/3 plane
        r = sqrt(px**2 + py**2)
        s = pz - d2
        Rlen = sqrt(r**2 + s**2)

        # Solve for theta3 via law of cosines
        cos_theta3 = (Rlen**2 - a2**2 - a3**2) / (2 * a2 * a3)
        theta3 = acos(cos_theta3)

        # Solve for theta2
        alpha = atan2(s, r)
        beta = atan2(a3 * sin(theta3), a2 + a3 * cos(theta3))
        theta2 = alpha - beta

        # Reconstruct R^0_3 to isolate wrist orientation
        T01 = self.dh_transform(*self.dh_params[0][:3], theta1)
        T12 = self.dh_transform(*self.dh_params[1][:3], theta2)
        T23 = self.dh_transform(*self.dh_params[2][:3], theta3)
        R03 = (T01 * T12 * T23)[:3, :3]

        # Compose R^0_6 from symbols
        R06 = Matrix([[r11, r12, r13],
                      [r21, r22, r23],
                      [r31, r32, r33]])
        # Wrist orientation
        R36 = R03.T * R06

        # Solve for theta4, theta5, theta6
        theta5 = acos(R36[2, 2])
        theta4 = atan2(R36[1, 2], R36[0, 2])
        theta6 = atan2(R36[2, 1], -R36[2, 0])

        return {
            'theta1': theta1,
            'theta2': theta2,
            'theta3': theta3,
            'theta4': theta4,
            'theta5': theta5,
            'theta6': theta6,
            'wrist_center': Matrix([px, py, pz]),
            'R_symbols': (r11, r12, r13, r21, r22, r23, r31, r32, r33)
        }

    def solve_for_joint_x(self, desired_3d_pose, joint_name):
        if joint_name not in self.ik_equations:
            raise ValueError(f"Unknown joint '{joint_name}'")

        # Extract end-effector pose
        px = desired_3d_pose[0, 3]
        py = desired_3d_pose[1, 3]
        pz = desired_3d_pose[2, 3]
        R06_vals = desired_3d_pose[:3, :3].flatten()

        # Evaluate expression
        expr = self.ik_equations[joint_name]
        f = lambdify(tuple(self.position_syms) + tuple(self.R_syms), expr, 'numpy')
        val = float(f(px, py, pz, *R06_vals))

        # Compute the two possible solutions per equation
        if joint_name == 'theta1':
            # atan2(y,x) and atan2(y,x)+pi
            return val, val + np.pi

        elif joint_name == 'theta2':
            # theta2 = alpha - beta  =>  theta2_alt = alpha + beta
            d2 = float(self.dh_params[1][2])
            a2 = float(self.dh_params[2][0])
            a3 = float(self.dh_params[3][0])
            r_planar = np.hypot(px, py)
            s = pz - d2
            alpha = np.arctan2(s, r_planar)
            # principal val = alpha - beta, so beta = alpha - val
            beta = alpha - val
            return val, alpha + beta

        elif joint_name == 'theta3':
            # acos -> two solutions: +acos, -acos
            return val, -val

        elif joint_name == 'theta5':
            # acos -> two solutions: +acos, -acos
            return val, -val

        elif joint_name == 'theta4':
            # atan2 -> two solutions: val and val + pi
            return val, val + np.pi

        elif joint_name == 'theta6':
            # atan2 -> two solutions: val and val + pi
            return val, val + np.pi

        return val, None