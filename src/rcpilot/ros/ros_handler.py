class ROSHandlerInterface:
    def create_subscriber(self, topic, message_type):
        """create ros subscriber node"""
        pass
    def create_publisher(self, topic, message_type):
        """create ros publisher node"""
        pass
    def publish_message(self, message, topic):
        """publish ros message to a specific topic"""
        pass
    def publish_image(self, image, topic):
        """publish image to a specific topic"""
        pass
    def get_topic_message(self, topic, message_type):
        """get message from a topic"""
        pass
    def get_topic_image(self, topic):
        """get image from a topic"""
        pass

def create_ros_handler(ros_version) -> ROSHandlerInterface:
    if ros_version == 1:
        from rcpilot.ros.ros1.ros1_handler import ROS1Handler
        return ROS1Handler()
    elif ros_version == 2:
        from rcpilot.ros.ros2.ros2_handler import ROS2Handler
        return ROS2Handler()
    
    # TODO: add exception and handle it
    return None