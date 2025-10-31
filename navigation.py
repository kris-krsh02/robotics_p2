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
        self.path_planner = None
        self.target = None
        self.robot_mode = "standard"  # standard or wall_following ? idea is to supress other behaviors when wall following

        rospy.init_node("navigator_node", anonymous=True)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.spin()

    def run(
        self, path_planner, obstacle_avoider, position_tracker, time_to_turn, target
    ):
        """
        Operator function to run the navigator between points.
        Responsible for listening to all topics and publishing velocity commands.
        """
        self.obstacle_avoider = obstacle_avoider
        self.position_tracker = position_tracker
        self.path_planner = path_planner
        self.target = target
        self.position_tracker.target = target
        self.target_reached = False
        twist = Twist()
        twist.angular.z = self.angular_velocity
        twist.linear.x = 0
        self.pub.publish(twist)
        rospy.sleep(time_to_turn)  # give time to turn towards target

        while self.target_reached is False:
            self.sub = rospy.Subscriber("/command_type", String, self.callback)

    def callback(self, data):
        if self.robot_mode == "wall_following":
            return  # supress everything for wall following

        data = data.data
        if data == "target reached":
            # stop robot
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.pub.publish(twist)
            self.target_reached = True

        elif data.startswith("obstacle detected"):
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.pub.publish(twist)
            obstacle_type = data.split(" - ")[1]  # do we need this?

            self.robot_mode = "wall_following"
            wall_follower = WallFollower(self.position_tracker)
            wall_follower.follow_walls()
            self.robot_mode = "standard"
            time_to_turn = self.path_planner.get_vel_commands(
                self.position_tracker.current_position, self.target
            )  # need to orient again after wall following
            twist.linear.x = 0
            twist.angular.z = self.angular_velocity
            self.pub.publish(twist)
            rospy.sleep(time_to_turn)

        elif data == "drive forward":
            twist = Twist()
            twist.linear.x = self.linear_velocity
            twist.angular.z = 0
            self.pub.publish(twist)
