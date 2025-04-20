import numpy as np
from sympy import symbols, sin, cos, Matrix

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
        Transformations_list = []
        T_curr = Matrix.eye(4)
        Transformations_list.append(T_curr) 

        for (a_i, alpha_i, d_i, th_i) in self.dh_params:
            T_i = self.dh_transform(a_i, alpha_i, d_i, th_i)
            T_curr = T_curr * T_i
            Transformations_list.append(T_curr)

    def dh_transform(sefl, a, alpha, d, theta):
        return Matrix([
            [ cos(theta),           -sin(theta),           0,             a ],
            [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d ],
            [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d ],
            [ 0,                     0,                      0,            1 ]
        ])