import rospy, math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class ObstacleAvoider:
    def __init__(self):
        rospy.init_node("obstacle_avoider_node", anonymous=True)
        self.pub = rospy.Publisher("/command_type", String, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.avoid_dispatcher)
        laser_offset = 0.257
        self.mid = None
        self.obstacle_thres = 0.3 + laser_offset
        self.sym_thres = 0.01

    def avoid_dispatcher(self, data):
        if self.mid is None:
            self.mid = len(data.ranges) // 2

        ranges = data.ranges

        # Get minium range reading on each side
        try:
            l_range = [x for x in ranges[self.mid :] if not math.isnan(x)]
            l_min = min(l_range)
        except:
            l_min = self.obstacle_thres + 10

        try:
            r_range = [x for x in ranges[: self.mid] if not math.isnan(x)]
            r_min = min(r_range)
        except:
            r_min = self.obstacle_thr + 10

        # Dispatch if at least one sensor is within obstacle threshold
        if l_min < self.obstacle_thr or r_min < self.obstacle_thr:
            if abs(l_min - r_min) < self.sym_thr:  # check for symmetric obstacles
                self.pub.publish("obstacle detected - symmetric")
            else:
                self.pub.publish("obstacle detected - asymmetric")
        
