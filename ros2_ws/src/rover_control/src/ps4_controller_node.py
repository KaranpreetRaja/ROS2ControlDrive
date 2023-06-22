import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pyPS4Controller.controller import Controller


def map_to_torque(current_num):
    current_min = -32767
    current_max = 32767

    desired_min = -100
    desired_max = 100

    scaling_factor = (desired_max - desired_min) / (current_max - current_min)

    desired_number = (current_num - current_min) * scaling_factor + desired_min

    return desired_number

# ros2 topic pub /drive_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]""


class MyROS2Node(Node):
    def __init__(self):
        super().__init__('my_ros2_node')
        self.publisher = self.create_publisher(
            Float64MultiArray, '/drive_controller/commands', 10)

    def publish_data(self, data):
        msg = Float64MultiArray()
        msg.data = [1.0, 1.0, 1.0]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published data: {msg.data}")


class MyController(Controller):
    def __init__(self, ros2_node, **kwargs):
        super().__init__(**kwargs)
        self.ros2_node = ros2_node

    def on_L3_up(self, value):
        a = map_to_torque(value)
        self.ros2_node.publish_data(a)

    def on_L3_down(self, value):
        a = map_to_torque(value)
        self.ros2_node.publish_data('L3 down:{}'.format(a))

    def on_L3_x_at_rest(self):
        self.ros2_node.publish_data('L3 rest')

    def on_L3_y_at_rest(self):
        self.ros2_node.publish_data('L3 rest')

    def on_R3_up(self, value):
        a = map_to_torque(value)
        self.ros2_node.publish_data('R3 up:{}'.format(a))

    def on_R3_down(self, value):
        a = map_to_torque(value)
        self.ros2_node.publish_data('R3 down:{}'.format(a))

    def on_R3_x_at_rest(self):
        self.ros2_node.publish_data('R3 rest')

    def on_R3_y_at_rest(self):
        self.ros2_node.publish_data('R3 rest')


def main(args=None):
    rclpy.init(args=args)
    node = MyROS2Node()
    controller = MyController(
        ros2_node=node, interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
