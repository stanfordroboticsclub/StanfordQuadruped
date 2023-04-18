import rospy
from std_msgs.msg import String


class StatusPublisher:

    def __init__(self):
        self.status_publisher = rospy.Publisher("/robot_status_messages", String, queue_size = 10)

    def publish_message(self, message):
        self.status_publisher.publish(message)
