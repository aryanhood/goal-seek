
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('waypoints', [
            [2.0, 2.0],
            [8.0, 2.0],
            [8.0, 8.0],
            [2.0, 8.0]
        ])

        self.waypoints = self.get_parameter('waypoints').value
        self.index = 0
        self.pose = None

        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)

    def pose_cb(self, msg):
        self.pose = msg

    def loop(self):
        if self.pose is None or self.index >= len(self.waypoints):
            return

        goal = self.waypoints[self.index]
        dx = goal[0] - self.pose.x
        dy = goal[1] - self.pose.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.3:
            self.index += 1
            self.get_logger().info(f"Reached waypoint {self.index}")
            return

        angle = math.atan2(dy, dx)
        err = angle - self.pose.theta

        cmd = Twist()
        cmd.linear.x = min(2.0, dist)
        cmd.angular.z = 4.0 * err
        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
