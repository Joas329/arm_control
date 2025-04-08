import sympy
from sympy import symbols, sin, cos, Matrix

###############################################################################
# Step 1) Define symbolic variables for the 6 revolute joint angles
###############################################################################
theta1, theta2, theta3, theta4, theta5, theta6 = symbols(
    'theta1 theta2 theta3 theta4 theta5 theta6',
    real=True
)

###############################################################################
# Step 2) Define the known DH parameters (alpha_i, a_i, d_i) for each joint.
#         We'll keep them numeric, but keep each theta_i symbolic.
#         The order we use is (a_i, alpha_i, d_i, theta_i).
#         Make sure this matches your DH convention.
###############################################################################
dh_params = [
    (0.0,     1.5707,    0.0,       theta1),    # Joint 1
    (0.0,     0.0,       -0.063263, theta2),    # Joint 2
    (0.4566,  0.0,       0.0,       theta3),    # Joint 3
    (0.11713, 0.0,       0.063262,  theta4),    # Joint 4
    (0.43497, 0.0,       0.0605,    theta5),    # Joint 5
    (0.082,   0.0,       -0.0605,   theta6)     # Joint 6
]

###############################################################################
# Helper: Construct a single 4x4 DH transform (revolute) with (a, alpha, d, theta).
###############################################################################
def dh_transform(a, alpha, d, theta):
    """
    Standard Denavit-Hartenberg 4x4 transform for a revolute joint.
      a: Link length
      alpha: Link twist
      d: Link offset
      theta: Joint angle (symbolic)
    """
    return Matrix([
        [ cos(theta),           -sin(theta),           0,             a ],
        [ sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d ],
        [ sin(theta)*sin(alpha), cos(theta)*sin(alpha),  cos(alpha),  cos(alpha)*d ],
        [ 0,                     0,                      0,            1 ]
    ])

###############################################################################
# Step 3) Compute forward kinematics T_0^i for i=1..6
###############################################################################
T_list = []
T_curr = Matrix.eye(4)  # T_0^0 is identity
T_list.append(T_curr)   # store it in a list for easy indexing

for (a_i, alpha_i, d_i, th_i) in dh_params:
    T_i = dh_transform(a_i, alpha_i, d_i, th_i)
    T_curr = T_curr * T_i  # multiply to get T_0^i
    T_list.append(T_curr)

# Now T_list[i] is T_0^i for i from 0..6

###############################################################################
# Extract rotation and position for each frame i in base coords
###############################################################################
def rot_and_pos(T):
    R = T[0:3, 0:3]
    p = T[0:3, 3]
    return R, p

R_list = []
p_list = []
for i in range(len(T_list)):
    R_i, p_i = rot_and_pos(T_list[i])
    R_list.append(R_i)
    p_list.append(p_i)

###############################################################################
# Step 4) Build the 6x6 Jacobian (because we have 6 revolute joints).
#         For each joint i (1-based), the column i of J is:
#
#   [ z_0^(i-1) x (p_0^(6) - p_0^(i-1)) ]
#   [              z_0^(i-1)           ]
#
###############################################################################
n = 6
J = sympy.zeros(6, n)

p_end = p_list[6]  # origin of frame 6 in base coords

for i in range(1, n+1):
    # i-th joint is attached to frame i-1
    R_0_i_minus_1 = R_list[i-1]
    p_0_i_minus_1 = p_list[i-1]

    # The z-axis of frame i-1 in base coords is the 3rd column of R_0^(i-1)
    z_i_minus_1 = R_0_i_minus_1[:, 2]

    # top 3 rows = cross(z, p_end - p_0^(i-1))
    linear_part = z_i_minus_1.cross(p_end - p_0_i_minus_1)
    # bottom 3 rows = z
    angular_part = z_i_minus_1

    for row in range(3):
        J[row, i-1]   = linear_part[row]
        J[row+3, i-1] = angular_part[row]

###############################################################################
# Step 5) Print or simplify the resulting symbolic 6x6 Jacobian
###############################################################################
print("Symbolic 6x6 Jacobian for the 6-DOF arm:\n")
sympy.pprint(J)

# print("\nPositions (d_0^i) of each frame i in the base frame:\n")

# for i in range(len(p_list)):
#     d_0_i = p_list[i]
#     print(f"d_0^{i} = ")
#     sympy.pprint(d_0_i)  # Symbolic 3x1 vector
#     print()

# J_simpl = sympy.simplify(J)
# print("\nSimplified version:\n")
# sympy.pprint(J_simpl)

###############################################################################
# Example usage:
# You can evaluate J at specific joint angles (in radians) by substituting:
#
# numeric_J = J_simpl.subs({
#     theta1: 0.0,
#     theta2: 1.57,
#     theta3: -0.5,
#     theta4: 0.0,
#     theta5: 0.0,
#     theta6: 0.0
# })
# print("\nNumeric Jacobian:\n", numeric_J.evalf())
###############################################################################


###############################################################################
# Step 6) Rotational Matrices
###############################################################################

# R_j1 = Matrix([
#     [cos(-theta1), -sin(-theta1), 0],
#     [sin(-theta1),  cos(-theta1),  0],
#     [0,             0,             1]
# ])

# # Joint 2: Axis of rotation [0, 0, 1], i.e. R_z(theta2)
# R_j2 = Matrix([
#     [cos(theta2), -sin(theta2), 0],
#     [sin(theta2),  cos(theta2), 0],
#     [0,            0,           1]
# ])

# # Joint 3: Same as Joint 2, R_z(theta3)
# R_j3 = Matrix([
#     [cos(theta3), -sin(theta3), 0],
#     [sin(theta3),  cos(theta3), 0],
#     [0,            0,           1]
# ])

# # Joint 4: Axis of rotation [1, 0, 0], i.e. R_x(theta4)
# R_j4 = Matrix([
#     [1,         0,          0],
#     [0,  cos(theta4), -sin(theta4)],
#     [0,  sin(theta4),  cos(theta4)]
# ])

# # Joint 5: Axis of rotation [0, 0, 1], i.e. R_z(theta5)
# R_j5 = Matrix([
#     [cos(theta5), -sin(theta5), 0],
#     [sin(theta5),  cos(theta5), 0],
#     [0,            0,           1]
# ])

# # Joint 6: Axis of rotation [-1, 0, 0], i.e. R_x(-theta6)
# R_j6 = Matrix([
#     [1,         0,               0],
#     [0,  cos(-theta6),  -sin(-theta6)],
#     [0,  sin(-theta6),   cos(-theta6)]
# ])


# ###############################################################################
# # Step 7) Compute the jacobian
# ###############################################################################
# def build_jacobian_6dof(R_list, p_list):
#     """
#     Build the 6x6 Jacobian for a 6-DOF revolute arm using the standard formula:
#        Column i = [ z_{0}^{(i-1)} x (p_{0}^{(6)} - p_{0}^{(i-1)}) ]
#                    [                 z_{0}^{(i-1)}                ]
#     where z_{0}^{(i-1)} is the 3rd column of R_list[i-1]
#     and p_{0}^{(i-1)} is p_list[i-1].
#     """
#     # We have frames 0..6 -> n=6
#     n = 6
#     # A 6x6 zero matrix
#     J_local = sympy.zeros(6, n)

#     # End-effector position d_0^6
#     p_end = p_list[6]

#     for i in range(1, n+1):
#         # i-th column => joint i
#         R_0_im1 = R_list[i-1]
#         p_0_im1 = p_list[i-1]

#         # z-axis = 3rd col of R_0^(i-1)
#         z_im1 = R_0_im1[:, 2]

#         # top 3 rows = cross(z, p_end - p_0^(i-1))
#         linear_part = z_im1.cross(p_end - p_0_im1)

#         # bottom 3 rows = z
#         angular_part = z_im1

#         for row in range(3):
#             J_local[row, i-1]   = linear_part[row]
#             J_local[row+3, i-1] = angular_part[row]

#     return J_local

# ###############################################################################
# # Step 8) Actually build and print the Jacobian
# ###############################################################################
# J_6x6 = build_jacobian_6dof(R_list, p_list)

# # print("\nPositions (d_0^i) of each frame i in the base frame:\n")
# # for i, p in enumerate(p_list):
# #     print(f"d_0^{i} = ")
# #     sympy.pprint(p)
# #     print()

# print("\nSymbolic 6x6 Jacobian for the 6-DOF arm:\n")
# sympy.pprint(J_6x6)

# # # Optionally simplify
# # J_simpl = sympy.simplify(J_6x6)
# # print("\nSimplified 6x6 Jacobian:\n")
# # sympy.pprint(J_simpl)

# ###############################################################################
# # Example usage:
# # Evaluate at a particular set of angles (in radians), e.g.:
# #
# # angle_dict = {
# #   theta1: 0.0,
# #   theta2: 1.57,
# #   theta3: -0.5,
# #   theta4: 0.2,
# #   theta5: 0.1,
# #   theta6: -1.0
# # }
# #
# # numeric_J = J_simpl.subs(angle_dict)
# # print("\nNumeric J at these joint angles:\n", numeric_J.evalf())
# ###############################################################################
