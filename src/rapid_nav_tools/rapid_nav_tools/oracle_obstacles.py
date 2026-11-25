"""Publish the scenario obstacles' true positions and velocities as tracked obstacles."""

import sys

from nav_msgs.msg import Odometry
from rapid_nav_msgs.msg import TrackedObstacle, TrackedObstacleArray
from rapid_nav_tools.obstacle_driver import load_scenarios
from rapid_nav_tools.scenarios import covering_radius, obstacle_topics, world_velocity
import rclpy
from rclpy.node import Node

MAX_AGE_S = 0.5


class OracleObstacles(Node):
    """Read the obstacles' Gazebo odometry and publish them on one TrackedObstacleArray topic."""

    def __init__(self, obstacles):
        """Subscribe to the odometry of every obstacle and start the publishing timer."""
        super().__init__('oracle_obstacles')
        self.declare_parameter('scenario', '')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('velocity_mode', 'true')
        self.declare_parameter('topic', '/obstacles/tracked')
        self.declare_parameter('frame_id', 'map')
        mode = str(self.get_parameter('velocity_mode').value).lower()
        if mode not in ('true', 'zero'):
            raise ValueError("velocity_mode must be 'true' or 'zero', got {!r}".format(mode))
        self.zero = mode == 'zero'
        self.frame_id = self.get_parameter('frame_id').value
        self.obstacles = obstacles
        self.latest = {}
        for obstacle in obstacles:
            odom_topic = obstacle_topics(obstacle['name'])[1]
            self.create_subscription(
                Odometry, odom_topic, lambda msg, n=obstacle['name']: self.on_odom(n, msg), 10)
        self.pub = self.create_publisher(
            TrackedObstacleArray, self.get_parameter('topic').value, 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.publish)

    def on_odom(self, name, msg):
        """Remember the newest odometry of an obstacle."""
        self.latest[name] = msg

    def publish(self):
        """Publish the obstacles whose odometry is recent."""
        now = self.get_clock().now().nanoseconds
        out = TrackedObstacleArray()
        out.header.frame_id = self.frame_id
        out.source = 'oracle_static' if self.zero else 'oracle'
        newest = 0
        for obstacle in self.obstacles:
            msg = self.latest.get(obstacle['name'])
            if msg is None:
                continue
            stamp = msg.header.stamp.sec * 10 ** 9 + msg.header.stamp.nanosec
            if now - stamp > MAX_AGE_S * 1e9:
                continue
            newest = max(newest, stamp)
            p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
            entry = TrackedObstacle()
            entry.id = obstacle['name']
            entry.position.x, entry.position.y = p.x, p.y
            if not self.zero:
                entry.velocity.x, entry.velocity.y = world_velocity(
                    (q.x, q.y, q.z, q.w), (t.x, t.y, t.z))
            entry.radius = covering_radius(obstacle['shape'])
            entry.confidence = 1.0
            # Ground truth: no uncertainty (0 keeps the hazard layer's fixed cost band).
            entry.position_sigma = 0.0
            entry.velocity_sigma = 0.0
            out.obstacles.append(entry)
        out.header.stamp.sec = newest // 10 ** 9 if newest else now // 10 ** 9
        out.header.stamp.nanosec = newest % 10 ** 9 if newest else now % 10 ** 9
        self.pub.publish(out)


def main(args=None):
    """Load the scenario named by the parameter and publish its obstacles until shut down."""
    rclpy.init(args=args)
    probe = rclpy.create_node('oracle_obstacles_probe')
    probe.declare_parameter('scenario', '')
    name = probe.get_parameter('scenario').value
    probe.destroy_node()
    scenarios = load_scenarios()
    if name not in scenarios:
        print('unknown scenario {!r}'.format(name), file=sys.stderr)
        rclpy.shutdown()
        return 1
    node = OracleObstacles(scenarios[name].get('obstacles') or [])
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
