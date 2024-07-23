from rcpilot.ros.ros_handler import ROSHandlerInterface
from rcpilot.ros.ros2.nodes import SubscriberNode, PublisherNode
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ROS2Handler(ROSHandlerInterface):
    def __init__(self) -> None:
        self.subscriber = None
        self.publisher = None
        if rclpy.ok():
            pass
        else:
            rclpy.init()
        self.cv_bridge = CvBridge()

    def create_subscriber(self, topic=None, message_type=None):
        self.subscriber = SubscriberNode()

    def create_publisher(self, topic=None, message_type=None):
        self.publisher = PublisherNode()

    def publish_message(self, message, topic):
        if self.publisher is None:
            self.create_publisher()
        self.publisher.publish_message_to_topic(message, topic)
    
    def publish_image(self, image, topic):
        if len(image.shape) == 2:
            image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="mono8")
        else:
            image_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding="rgb8")
        self.publish_message(image_msg, topic)

    def get_topic_message(self, topic, message_type):
        if self.subscriber is None:
            self.create_subscriber()
        msg = self.subscriber.read_from_topic(topic, message_type)
        return msg
    
    def get_topic_image(self, topic):
        image_msg = self.get_topic_message(topic, Image)
        if image_msg is None:
            return None
        image = self.cv_bridge.imgmsg_to_cv2(image_msg)

        return image