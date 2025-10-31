import rospy
from std.msgs.msg import String


class Driver:
    def __init__(self):
        rospy.init_node("driver_node", anonymous=True)
        self.pub = rospy.Publisher("/command_type", String, queue_size=1)
        self.rate = rospy.Rate(10)  # 10hz

    def drive(self):
        while not rospy.is_shutdown():
            drive_msg = String()
            drive_msg.data = "drive forward"
            self.pub.publish(drive_msg)
            self.rate.sleep()


if __name__ == "__main__":
    driver = Driver()
