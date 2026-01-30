
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.cb, 10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

    def cb(self, pose):
        odom = Odometry()
        odom.pose.pose.position.x = pose.x
        odom.pose.pose.position.y = pose.y
        self.pub.publish(odom)

def main():
    rclpy.init()
    node = OdomPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
