
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class GoalSeekController(Node):
    def __init__(self):
        super().__init__('goal_seek_controller')

        self.declare_parameter('goal_x', 5.5)
        self.declare_parameter('goal_y', 5.5)
        self.declare_parameter('kp_linear', 1.5)
        self.declare_parameter('kp_angular', 6.0)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value

        self.pose = None

        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Goal Seek Controller initialized")

    def pose_cb(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = angle_to_goal - self.pose.theta

        cmd = Twist()
        cmd.linear.x = self.kp_linear * distance
        cmd.angular.z = self.kp_angular * angle_error

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = GoalSeekController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
