import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def joy_callback(self, msg):
        twist = Twist()
        twist.linear.x = msg.axes[1]  # 左スティック上下
        twist.angular.z = msg.axes[2]  # 右スティック左右
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

