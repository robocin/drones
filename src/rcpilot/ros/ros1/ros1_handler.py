from rcpilot.ros.ros_handler import ROSHandlerInterface
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ROS1Handler(ROSHandlerInterface):
    def __init__(self):
        self.subscriber = None
        self.publisher = None
        self.cv_bridge = CvBridge()

        rospy.init_node("ros_handler")

    def create_subscriber(self, topic=None, message_type=None):
        pass
    
    def create_publisher(self, topic, message_type): 
        self.publisher = rospy.Publisher(topic, message_type, queue_size=8)

    def publish_message(self, message, topic):
        if self.publisher is None:
            self.create_publisher(topic, type(message))
        self.publisher.publish(message)
    
    def publish_image(self, image, topic):
        if len(image.shape) == 2:
            image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="mono8")
        else:
            image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.publish_message(image_msg, topic)
    
    def get_topic_message(self, topic, message_type):
        msg = rospy.wait_for_message(topic, message_type)
        return msg

    def get_topic_image(self, topic):
        image_msg = self.get_topic_message(topic, Image)
        image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        return image