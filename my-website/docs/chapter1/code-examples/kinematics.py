import math

def forward_kinematics_2d_arm(l1, l2, theta1_deg, theta2_deg):
    """
    Calculates the end-effector position (x, y) for a 2D two-link robot arm
    using forward kinematics.

    Args:
        l1 (float): Length of the first link.
        l2 (float): Length of the second link.
        theta1_deg (float): Angle of the first joint in degrees relative to the x-axis.
        theta2_deg (float): Angle of the second joint in degrees relative to the first link.

    Returns:
        tuple: (x, y) coordinates of the end-effector.
    """
    theta1_rad = math.radians(theta1_deg)
    theta2_rad = math.radians(theta2_deg)

    # X-coordinate of the end-effector
    x = l1 * math.cos(theta1_rad) + l2 * math.cos(theta1_rad + theta2_rad)
    # Y-coordinate of the end-effector
    y = l1 * math.sin(theta1_rad) + l2 * math.sin(theta1_rad + theta2_rad)

    return x, y

if __name__ == "__main__":
    # Example usage:
    link1_length = 1.0  # meters
    link2_length = 0.8  # meters
    joint1_angle_deg = 30  # degrees
    joint2_angle_deg = 60  # degrees

    end_effector_pos = forward_kinematics_2d_arm(link1_length, link2_length, joint1_angle_deg, joint2_angle_deg)
    print(f"For a 2D arm with L1={link1_length}m, L2={link2_length}m:")
    print(f"Joint angles: Theta1={joint1_angle_deg}°, Theta2={joint2_angle_deg}°")
    print(f"End-effector position (x, y): {end_effector_pos[0]:.2f}m, {end_effector_pos[1]:.2f}m")

    joint1_angle_deg = 90
    joint2_angle_deg = 0
    end_effector_pos = forward_kinematics_2d_arm(link1_length, link2_length, joint1_angle_deg, joint2_angle_deg)
    print(f"\nJoint angles: Theta1={joint1_angle_deg}°, Theta2={joint2_angle_deg}°")
    print(f"End-effector position (x, y): {end_effector_pos[0]:.2f}m, {end_effector_pos[1]:.2f}m")