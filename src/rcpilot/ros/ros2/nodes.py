import warnings
from visualization_msgs.msg import Marker

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

class SubscriberNode(Node):
    # TODO: consider not unsubscribing to a topic all the time. Maybe, parametrize this.
    '''
    Implements a ROS2 subscriber node
    
    It contains a list of ros2 subscriptions, one for each topic we're interested in reading. 
    '''

    # static class variable "node_id" for differenciating between multiple SubscriberNodes
    # TODO; this will break if an instance of this class is created withing another process. Possible solutions: use uuid4 for id or have a global inter-process variable for id or use ephoch?
    node_id = 0

    def __init__(self):
        SubscriberNode.node_id += 1
        super().__init__(f'SubscriberNode_{SubscriberNode.node_id}')
        # TODO: there is already a Node.publishers and Node.subscriptions in rclpy, is there a need for a list to hold them in this subclass? Maybw just use the Node class attribute
        self.__subscribers: Subscription = []

    def create_subscriber(self, msg_type, topic):
        """
        creates a subscriber if one for that topic does not exist.
        """

        subscriber = None
        if(topic not in [subscriber['topic'] for subscriber in self.__subscribers]):
            subscriber = self.create_subscription(
                msg_type,
                topic,
                self.callback_wrapper(topic),
                qos_profile=rclpy.qos.qos_profile_sensor_data
            )
            self.__subscribers.append({'subscriber': subscriber, 'topic': topic, 'message': None})
        else:
            warnings.warn(f"Subscriber for topic {topic} already exists. No subscriber was created")
        return subscriber

    def callback_wrapper(self, topic_name):
        """
        callback_function wrapper for passing extra parameter "topic_name" as a parameter to the callback function
        # TODO is this wrapper really necessary? Can we pass the topic_name with just the callback_function?
        """
        def callback(msg):
            self.callback_function(topic_name, msg)
        return callback

    def callback_function(self, topic: str, msg):
        """
        Called everytime a new message is received from a topic

        Stores the received message in the subscriber for that topic
        """
        # get the subscriber for that message
        subscriber = next((subscriber for subscriber in self.__subscribers if subscriber['topic'] == topic), None)
        # put the received message in the subscriber's message buffer 
        subscriber['message'] = msg

    def read_from_topic(self, topic: str, message_type=None):
        """
        TODO
        """

        # add '/' if necessary
        if topic[0] != '/':
            topic = f"/{topic}"
        
        # get the subscriber (dict) subscribed to the given topic
        subscriber = next((subscriber for subscriber in self.__subscribers if subscriber['topic'] == topic),None)

        # if a subscriber for the given topic does not exist, create one
        if subscriber == None:
            if(message_type == None):
                raise BaseException(f"No subscriber for the topic {topic} was found. One coudn't be created because message_type param wasn't provided to read_from_topic.")
            else:
                subscriber = self.create_subscriber(message_type, topic)
                warnings.warn(f"No subscriber for the topic {topic} was found. One was created.")
        
        # allow the execution of the callback function if a message is received in that topic
        rclpy.spin_once(self)

        # get the received message and remove it from the subscriber (dict) buffer
        received_message_index = next((index for index, item in enumerate(self.__subscribers) if item['topic'] == topic), None)
        if received_message_index is not None:
            received_message_holder = self.__subscribers[received_message_index]
            received_message = received_message_holder['message']
            received_message_holder['message'] = None
        else:
            received_message = None

        return received_message

class PublisherNode(Node):
    '''
    Implements a ROS2 publisher node 
    '''

    node_id = 0

    def __init__(self):
        PublisherNode.node_id += 1
        super().__init__(f'publisher_node_{PublisherNode.node_id}')
        # TODO: there is already a Node.publishers and Node.subscriptions in rclpy, is there a need for a list to hold them in this subclass? Maybw just use the Node class attribute
        self._publishers: Publisher = []

    def create_publisher(self, *args, **kwargs) -> Publisher:
        # TODO: ver se tou botando barra se não estiver
        """
        Creates a ros2 publisher

        (This function lightly overrides (wraps) the original rclpy.node.Node.create_publisher)

        Params:
            either one:
            - 2 arguments (msg_type, topic: str), or
            - 3 argumetns (self, msg_type, topic: str), or
            - 3+ arguments, as per original rclpy.node.Node.create_publisher

        """
        topic = args[1]
        if(topic not in [publisher.topic for publisher in self._publishers]):
            if(len(args) == 2 or len(args)==3 and args[0] == self):
                publisher = super().create_publisher(msg_type=args[0], topic=args[1], qos_profile=10)
                self._publishers.append(publisher)
                return publisher
            elif(len(args) > 3 or len(args)==3):
                publisher = super().create_publisher(*args)
                return publisher
            else:
                raise TypeError("""
                create_publisher() takes either
                    - 2 arguments (msg_type, topic: str), or
                    - 3 argumetns (self, msg_type, topic: str), or
                    - 3+ arguments, as per original rclpy.node.Node.create_publisher
                """)
        else:
            warnings.warn("A publisher for this topic already exists in this node. Doing nothing.")

    def publish_message_to_topic(self, message, topic: str):
        """
        Publish the given message to the specified topic.

        Params:
            message: The message to be published.
            topic:   The topic to which the message will be published.

        Returns:
            None

        Warnings:
            If no publisher for the given topic exists, create one and publish the message with it.

        """
    
        # get the publisher for the given topic 
        publisher = next((publisher for publisher in self._publishers if publisher.topic == topic),None)

        # if one for that topic doesn't exist, create one 
        if publisher == None:
            warnings.warn(f"No publisher for the topic {topic} was found. One was created.")
            publisher = self.create_publisher(type(message), topic)
        
        # define lifetime of the Marker (amount of time it will live on after the message is sent) (if zero, the marker disappear immediately after the Node closing)
        if(type(message) == Marker):
            message.lifetime.sec = 1
        # print(type(message)) 
        publisher.publish(message)

