import rospy, rosnode
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from avoid_obstacles import ObstacleAvoider
from wall_follower import WallFollower


class Navigator:
    def __init__(self):
        self.linear_velocity = 1.0
        self.angular_velocity = 0.5
        self.target_reached = False
        self.obstacle_avoider = None
        self.position_tracker = None
        self.target = None
        self.robot_mode = "standard"  # standard or wall_following ? idea is to supress other behaviors when wall following

        rospy.init_node("navigator_node", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.spin()

    def run(self, obstacle_avoider, position_tracker, time_to_turn, target):
        """
        Operator function to run the navigator between points.
        Responsible for listening to all topics and publishing velocity commands.
        """
        self.obstacle_avoider = obstacle_avoider
        self.position_tracker = position_tracker
        self.target = target
        twist = Twist()
        twist.angular.z = self.angular_velocity
        twist.linear.x = 0
        self.pub.publish(twist)

        while self.target_reached is False:
            self.sub = rospy.Subscriber("/command_type", String, self.callback)

    def callback(self, data):
        if self.robot_mode == "wall_following":
            return  # supress everything for wall following

        data = data.data
        if data == "target reached":
            # stop robot
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            self.pub.publish(twist_msg)
            self.target_reached = True

        elif data.startswith("obstacle detected"):
            twist_msg = Twist()
            twist_msg.linear.x = 0
            twist_msg.angular.z = 0
            self.pub.publish(twist_msg)
            obstacle_type = data.split(" - ")[1]  # do we need this?

            self.robot_mode = "wall_following"
            wall_follower = WallFollower(self.position_tracker)
            wall_follower.follow_walls()
            self.robot_mode = "standard"
            # how do we make sure we move along the wall?
            # wall following behavior to be implemented

        elif data == "drive forward":
            twist_msg = Twist()
            twist_msg.linear.x = self.linear_velocity
            twist_msg.angular.z = 0
            self.pub.publish(twist_msg)
