import math, rospy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation


class PositionTracker:
    def __init__(self):
        self.current_position = None  # (x, y, theta)
        rospy.init_node("position_tracker_node", anonymous=True)
        self.sub = rospy.Subscriber("/odom", Odometry, self.callback)
        self.pub = rospy.Publisher("/command_type", str, queue_size=1)
        rospy.spin()

    def callback(self, data):
        position_x = data.pose.pose.position.x
        position_y = data.pose.pose.position.y

        self.is_target_reached((position_x, position_y), (5, 5))
        orientation_q = data.pose.pose.orientation  # quaternion form
        rotation = Rotation(orientation_q)
        rotation_euler = rotation.as_euler("xyz")
        theta = rotation_euler[1]  # TODO: not sure which angle to use
        self.current_position = (position_x, position_y, theta)

    def is_target_reached(self, curr_pos, target_pos, threshold=0.3):
        distance = math.sqrt(
            math.pow((curr_pos[0] - target_pos[0]), 2)
            + math.pow((curr_pos[1] - target_pos[1]), 2)
        )

        if distance <= threshold:
            self.pub.publish("target reached")


if __name__ == "__main__":
    tracker = PositionTracker()
