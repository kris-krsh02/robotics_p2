import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


class WallFollower:
    def __init__(self, position_tracker):
        self.position_tracker = position_tracker
        self.wall_hit_point = position_tracker.current_position
        self.target_point = position_tracker.target_point
        self.laser_readings = None
        self.linear_velocity = 1
        self.angular_velocity = 0.5

        # Line parameters for Bug 2
        self.m = (self.wall_hit_point[1] - self.target_point[1]) / (
            (self.wall_hit_point[0] - self.target_point[0])
        )  # slope
        self.b = self.wall_hit_point[1] - self.m * self.wall_hit_point[0]  # y-intercept

        self.hit_point_target_distance = math.sqrt(
            math.pow((self.wall_hit_point[0] - self.target_point[0]), 2)
            + math.pow((self.wall_hit_point[1] - self.target_point[1]), 2)
        ) # to check if new point on line is closer than hit point

        # self.pub = rospy.Publisher("/command_type", String, queue_size=1)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        laser_offset = 0.257
        self.obstacle_thres = 0.3 + laser_offset

    # Bug 2 algorithm - follow walls until we reach a point on the line to the target
    def follow_walls(self):
        # Turn away from wall
        # Follow wall until new obstacle or closer point on line to target reached

        while not self.reached_closer_point():
            action = self.obstacle_detected()
            twist = Twist()

            if action == "turn left":
                twist.linear.x = 0
                twist.angular.z = self.angular_velocity

            elif action == "follow wall":
                twist.linear.x = self.linear_velocity
                twist.angular.z = 0

            elif action == "turn right":
                twist.linear.x = 0
                twist.angular.z = -self.angular_velocity

            else:  # nothing to follow - shouldn't really happen but just in case
                twist.linear.x = 0
                twist.angular.z = 0
                break

            self.pub.publish(twist)
            rospy.sleep(0.1) # quick enough to detect changes on time?

    def obstacle_detected(self):
        ranges = self.laser_readings

        # This needs to be different from the standard obstacle avoider
        # Need front region, left and right regions, left front and right front regions (like diagonals?)
        l_range, lf_range, front_range, rf_range, r_range = [], [], [], [], []
        region_size = len(ranges) // 5
        l_range = ranges[0:region_size]
        lf_range = ranges[region_size : 2 * region_size]
        front_range = ranges[2 * region_size : 3 * region_size]
        rf_range = ranges[3 * region_size : 4 * region_size]
        r_range = ranges[4 * region_size :]

        # Get minium range reading on each side
        try:
            l_range = [x for x in l_range if not math.isnan(x)]
            l_min = min(l_range)
        except:
            l_min = self.obstacle_thres + 10

        try:
            r_range = [x for x in r_range if not math.isnan(x)]
            r_min = min(r_range)
        except:
            r_min = self.obstacle_thres + 10

        try:
            front_range = [x for x in front_range if not math.isnan(x)]
            front_min = min(front_range)
        except:
            front_min = self.obstacle_thres + 10

        try:
            lf_range = [x for x in lf_range if not math.isnan(x)]
            lf_min = min(lf_range)
        except:
            lf_min = self.obstacle_thres + 10

        try:
            rf_range = [x for x in rf_range if not math.isnan(x)]
            rf_min = min(rf_range)
        except:
            rf_min = self.obstacle_thres + 10

        if front_min < self.obstacle_thres:
            return "turn left"

        elif rf_min < self.obstacle_thres:
            return "follow wall"

        elif front_min > self.obstacle_thres and rf_min > self.obstacle_thres:
            return "turn right"

        return "no obstacle"

    def reached_closer_point(self):
        """
        Check if we have reached a point on the line to the target that is closer than the wall hit point.
        """
        curr_pos = self.position_tracker.current_position
        if curr_pos[1] == self.m * curr_pos[0] + self.b:
            new_point_distance = math.sqrt(
                math.pow((curr_pos[0] - self.target_point[0]), 2)
                + math.pow((curr_pos[1] - self.target_point[1]), 2)
            )
            
            if new_point_distance < self.hit_point_target_distance:
                return True

        return False

    def scan_callback(self, data):
        self.laser_readings = data.ranges
