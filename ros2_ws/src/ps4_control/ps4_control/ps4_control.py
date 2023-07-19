import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pyPS4Controller.controller import Controller


def map_to_torque(current_num):
    current_min = -32767
    current_max = 32767

    desired_min = -10
    desired_max = 10

    scaling_factor = float(desired_max - desired_min) / \
        float(current_max - current_min)

    desired_number = float(current_num - current_min) * \
        scaling_factor + float(desired_min)

    return desired_number

# ros2 topic pub /drive_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"


class MyROS2Node(Node):
    def __init__(self):
        super().__init__('ps4_contol')
        print("ROS2 Node Initialized")
        self.publisher = self.create_publisher(
            Float64MultiArray, '/drive_controller/commands', 10)
        self.right_data = [0.0, 0.0, 0.0]
        self.left_data = [0.0, 0.0, 0.0]

    def publish_data(self):
        print("Publishing Data")
        msg = Float64MultiArray()
        msg.data = self.right_data + self.left_data
        self.publisher.publish(msg)
        self.get_logger().info(f"Published data: {msg.data}")
        print(f"Published data: {msg.data}")

    def update_left_data(self, data):
        print("Left Data Updated")
        self.left_data = data
        self.publish_data()
        print("Left Data Published")

    def update_right_data(self, data):
        self.right_data = data
        self.publish_data()


class MyController(Controller):
    def __init__(self, ros2_node, **kwargs):
        super().__init__(**kwargs)
        self.ros2_node = ros2_node

    def on_L3_up(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.update_left_data([-torque, -torque, -torque])

    def on_L3_down(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.update_left_data([-torque, -torque, -torque])

    def on_L3_y_at_rest(self):
        self.ros2_node.update_left_data([0.0, 0.0, 0.0])

    def on_R3_up(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.update_right_data([torque, torque, torque])

    def on_R3_down(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.update_right_data([torque, torque, torque])

    def on_R3_y_at_rest(self):
        self.ros2_node.update_right_data([0.0, 0.0, 0.0])


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
