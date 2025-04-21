import numpy as np
from scipy.spatial.transform import Rotation as R
from IKPSolver import IKSPSolver  # Make sure this file is in the same directory

# Step 1: Build the desired pose from position and rotation
position = np.array([0.2883, 0.3379, -0.0381])
euler_angles = [-1.8133, 0.0, 0.0435]  # in radians

rotation_matrix = R.from_euler('xyz', euler_angles).as_matrix()

T = np.eye(4)
T[:3, :3] = rotation_matrix
T[:3, 3] = position

print("Transformation Matrix:")
print(np.round(T, 4))

# Step 2: Initialize the IK solver
solver = IKSPSolver()

# Step 3: Solve and print all joint angles
print("\nJoint Angles (degrees):")
for joint in ['theta1','theta2','theta3','theta4','theta5','theta6']:
    sol1, sol2 = solver.solve_for_joint_x(T, joint)
    deg1, deg2 = np.degrees(sol1), np.degrees(sol2)
    print(f"{joint}: {deg1:.2f}°,  alternative: {deg2:.2f}°")