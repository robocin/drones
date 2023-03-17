from definitions import connect_to_vehicle, get_position
import time
import rclpy
import rclpy.node
import std_msgs.msg
import geometry_msgs.msg


class DroneTelemetry(rclpy.node.Node):
    def __init__(self):
        super().__init__('drone_telemetry')

        self.publisher_ = self.create_publisher(
            geometry_msgs.msg.PoseStamped, 'drone_pos', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.vehicle = connect_to_vehicle(connection_string="127.0.0.1:14552")

    def timer_callback(self):
        position_array = get_position(self.vehicle, digitsPrecision=2)

        msg = geometry_msgs.msg.PoseStamped()
        msg.pose.position.x = position_array[0]
        msg.pose.position.y = position_array[1]
        msg.pose.position.z = position_array[2]

        # msg.header.frame_id =

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    drone_telemetry = DroneTelemetry()

    rclpy.spin(DroneTelemetry())


if __name__ == '__main__':
    main()
