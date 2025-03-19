from urdf_parser_py.urdf import URDF
import numpy as np

class InverseKinematics:
    def __init__(self, urdf_file_path):
        with open(urdf_file_path, 'r') as file:
            urdf_xml_string = file.read()

        # This line is hardcoded for our capstone arm
        urdf_xml_string = urdf_xml_string.replace('<?xml version="1.0" ?>', '')

        self.robot = URDF.from_xml_string(urdf_xml_string)

        self.joint_axis = []
        self.joint_translations = []
        self.joint_names = []
        self.joint_limits = []

        for joint in self.robot.joints:
            if joint.type in ["revolute", "continuous"]:
                xyz = joint.origin.xyz if (joint.origin and joint.origin.xyz) else [0, 0, 0]
                rpy = joint.origin.rpy if (joint.origin and joint.origin.rpy) else [0, 0, 0]
                axis = joint.axis if joint.axis is not None else [0, 0, 1]

                self.joint_rpy.append(rpy)
                self.joint_axis.append(axis)
                self.joint_translations.append(xyz)
                self.joint_names.append(joint.name)

        self.joint_axis = np.array(self.joint_axis)
        self.joint_translations = np.array(self.joint_translations)
        self.joint_rpy = np.array(self.joint_rpy)
        self.num_joints = len(self.joint_names)

    def axis_angle_rotation_matrix(self, axis, angle):
        """
        Creates a 3x3 rotation matrix from an axis and an angle.

        Parameters:
        - axis: A 3-element unit vector representing rotation axis [kx, ky, kz]
        - angle: Rotation angle in radians

        Returns:
        - 3x3 rotation matrix
        """
        c = np.cos(angle)
        s = np.sin(angle)
        v = 1 - c
        kx, ky, kz = axis

        # Build rotation matrix
        r00 = kx * kx * v + c
        r01 = kx * ky * v - kz * s
        r02 = kx * kz * v + ky * s

        r10 = kx * ky * v + kz * s
        r11 = ky * ky * v + c
        r12 = ky * kz * v - kx * s

        r20 = kx * kz * v - ky * s
        r21 = ky * kz * v + kx * s
        r22 = kz * kz * v + c

        return np.array([[r00, r01, r02],
                         [r10, r11, r12],
                         [r20, r21, r22]])

    def homogeneous_transform(self, axis, translation, angle):
        """
        Create a homogeneous transformation matrix from axis-angle rotation and translation.

        Parameters:
        - axis: A 3-element unit vector representing rotation axis
        - translation: A 3-element vector representing translation
        - angle: Rotation angle in radians

        Returns:
        - 4x4 homogeneous transformation matrix
        """
        # Get rotation matrix
        rot_matrix = self.axis_angle_rotation_matrix(axis, angle)

        # Create translation column vector
        trans_vec = np.array([[translation[0]],
                              [translation[1]],
                              [translation[2]]])

        # Combine into homogeneous transform
        h_matrix = np.concatenate((rot_matrix, trans_vec), axis=1)
        h_matrix = np.concatenate((h_matrix, np.array([[0, 0, 0, 1]])), axis=0)

        return h_matrix

    def forward_kinematics(self, joint_angles, end_effector_offset=[0, 0, 0], joint_index=-1):
        """
        Compute the position of a point on the robot in the base frame.

        Parameters:
        - joint_angles: Array of joint angles in radians
        - end_effector_offset: Position vector in the frame of the specified joint
        - joint_index: Index of the joint (-1 for end effector/last joint)

        Returns:
        - 3-element vector with position in base frame
        """
        # Convert end effector position to homogeneous coordinates
        position = np.array([[end_effector_offset[0]],
                            [end_effector_offset[1]],
                            [end_effector_offset[2]],
                            [1]])

        # If joint_index is -1, use the last joint
        if joint_index == -1:
            joint_index = self.num_joints - 1

        # Store original joint index
        orig_joint_index = joint_index

        # Apply transformations from the specified joint back to the base
        transform_result = None

        while joint_index >= 0:
            # Get current joint's transformation matrix
            current_transform = self.homogeneous_transform(
                self.joint_axis[joint_index],
                self.joint_translations[joint_index],
                joint_angles[joint_index]
            )

            # Apply transform
            if joint_index == orig_joint_index:
                transform_result = current_transform @ position
            else:
                transform_result = current_transform @ transform_result

            joint_index -= 1

        # Extract position from homogeneous result
        result_position = np.array([
            transform_result[0][0],
            transform_result[1][0],
            transform_result[2][0]
        ])

        return result_position

    def jacobian(self, joint_angles, end_effector_offset=[0, 0, 0]):
        """
        Compute the Jacobian matrix for the current robot configuration.

        Parameters:
        - joint_angles: Array of joint angles in radians
        - end_effector_offset: Position vector of end effector in last joint frame

        Returns:
        - 3xN Jacobian matrix where N is the number of joints
        """
        # Get end effector position in base frame
        end_effector_position = self.forward_kinematics(joint_angles, end_effector_offset)

        # Initialize Jacobian matrix
        jacobian_matrix = np.zeros((3, self.num_joints))

        # Compute Jacobian column for each joint
        for i in range(self.num_joints):
            # Get current joint position in base frame
            joint_position = self.forward_kinematics(joint_angles, [0, 0, 0], i)

            # Vector from joint to end effector
            joint_to_end = end_effector_position - joint_position
            # Get rotation axis for this joint in base frame
            # Note: This is simplified - in a real system, the joint axis would need
            # to be transformed to the base frame based on the current configuration
            joint_axis = self.joint_axis[i]

            # Compute Jacobian column using cross product (rotational joint)
            jacobian_column = np.cross(joint_axis, joint_to_end)

            # Add to Jacobian matrix
            jacobian_matrix[:, i] = jacobian_column

        return jacobian_matrix

    def inverse_kinematics(self, start_angles, target_position, end_effector_offset=[0, 0, 0], max_iterations=500, tolerance=0.01):
        """
        Solve inverse kinematics using pseudoinverse of the Jacobian.

        Parameters:
        - start_angles: Initial guess for joint angles
        - target_position: Desired end effector position in base frame
        - end_effector_offset: Position vector of end effector in last joint frame
        - max_iterations: Maximum number of iterations
        - tolerance: Error tolerance for convergence

        Returns:
        - Joint angles that achieve the target position
        """
        # Step size parameters
        position_step_size = 0.05  # Scale factor for position increments
        max_angle_step = 0.2      # Maximum joint angle change per iteration

        # Initialize current joint angles
        current_angles = np.array(start_angles)

        # Convert target position to numpy array
        target = np.array(target_position)

        # Main IK loop
        for iteration in range(max_iterations):
            # Get current end effector position
            current_position = self.forward_kinematics(current_angles, end_effector_offset)

            # Compute position error
            position_error = target - current_position

            # Check for convergence
            error_magnitude = np.linalg.norm(position_error)
            if error_magnitude < tolerance:
                print(f"Converged after {iteration} iterations with error {error_magnitude:.4f}")
                break

            # Print progress every 50 iterations
            if iteration % 50 == 0:
                print(f"Iteration {iteration}: Error = {error_magnitude:.4f}")
                print(f"Current position: {current_position}")
                print(f"Current angles: {np.degrees(current_angles)}\n")

            # Scale step size based on error magnitude
            step = position_error * position_step_size / error_magnitude

            # Get Jacobian at current configuration
            J = self.jacobian(current_angles, end_effector_offset)

            # Compute pseudoinverse of Jacobian
            J_pinv = np.linalg.pinv(J)

            # Calculate joint angle changes
            angle_changes = J_pinv @ step

            # Limit step size for stability
            angle_changes = np.clip(angle_changes, -max_angle_step, max_angle_step)

            # Update joint angles
            current_angles = current_angles + angle_changes

        # If max iterations reached without convergence
        if iteration == max_iterations - 1:
            print(f"Warning: Did not converge after {max_iterations} iterations. Final error: {error_magnitude:.4f}")

        return current_angles

    def solve_ik(self, target_position, end_effector_offset=[0, 0, 0], initial_guess=None):
        """
        Wrapper method to solve inverse kinematics with multiple initial guesses if needed.

        Parameters:
        - target_position: Desired end effector position [x, y, z]
        - end_effector_offset: Offset from last joint to end effector
        - initial_guess: Initial joint angles guess (if None, uses zeros)

        Returns:
        - Joint angles solution or None if no solution found
        """
        # Use zero angles as default initial guess if none provided
        if initial_guess is None:
            initial_guess = np.zeros(self.num_joints)

        # Try to solve IK with the initial guess
        solution = self.inverse_kinematics(
            initial_guess,
            target_position,
            end_effector_offset,
            max_iterations=500,
            tolerance=0.01
        )

        # Verify solution accuracy
        final_position = self.forward_kinematics(solution, end_effector_offset)
        error = np.linalg.norm(np.array(target_position) - final_position)

        if error < 0.05:  # 5cm tolerance
            print(f"Solution found with error: {error:.4f}")
            print(f"Joint angles (degrees): {np.degrees(solution)}")
            return solution
        else:
            print(f"Solution not accurate enough. Error: {error:.4f}")
            # Could implement more advanced retry strategies here
            return None

# Example usage
if __name__ == "__main__":
    # Path to your URDF file
    urdf_file_path = './description/arm_hardware.urdf'

    # Create IK solver
    ik_solver = InverseKinematics(urdf_file_path)

    # Define target position for end effector
    target = [0.4, 0.3, 0.5]  # x, y, z in meters

    # Offset from last joint to end effector (if any)
    end_effector_offset = [0, 0, 0.1]  # Example: 10cm along z-axis

    # Solve IK
    joint_angles = ik_solver.solve_ik(target, end_effector_offset)

    if joint_angles is not None:
        print("\nFinal Joint Angles (degrees):")
        for i, name in enumerate(ik_solver.joint_names):
            print(f"{name}: {np.degrees(joint_angles[i]):.2f}")

        # Verify solution
        final_pos = ik_solver.forward_kinematics(joint_angles, end_effector_offset)
        print(f"\nTarget position: {target}")
        print(f"Achieved position: {final_pos}")
        print(f"Error: {np.linalg.norm(np.array(target) - final_pos):.4f} meters")
