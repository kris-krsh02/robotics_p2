import math


class PathPlanner:
    def __init__(self):
        self.angular_speed = None

    def set_angular_speed(self, angular_speed):
        self.angular_speed = angular_speed

    def get_vel_commands(self, curr_pos, target_pos):
        """
        Calculate velocity commands to move from curr_pos to target_pos.
        Linear velocity is constant. We need to calculate angular velocity to orient robot towards target.

        Args:
        curr_pos: tuple (x, y, angle) current position and orientation
        target_pos: tuple (x, y) target position

        Returns:
            tuple (linear_velocity, time_to_turn)
        """

        delta_x = target_pos[0] - curr_pos[0]
        delta_y = target_pos[1] - curr_pos[1]
        target_angle = math.atan2(delta_y, delta_x)
        angle_diff = target_angle - curr_pos[2]

        # Normalize angle_diff to be within [-pi, pi]
        angle_diff = angle_diff % (2 * math.pi)
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi

        # Tell robot to turn until it's facing the target
        # Angular velocty != angle_diff to avoid extreme turns
        time_to_turn = abs(angle_diff) / self.angular_speed
        return time_to_turn
