"""Spawn the moving obstacles of a scenario and drive them along their scripted paths."""

import math
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rapid_nav_tools.scenarios import (
    obstacle_sdf,
    obstacle_topics,
    PathFollower,
    shape_height,
    SHAPES,
    spawn_height,
    StartTrigger,
    validate_scenario,
)
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32
from visualization_msgs.msg import Marker, MarkerArray
import yaml

SPAWN_TIMEOUT_S = 60
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def load_scenarios():
    """Return the dictionary of nav_scenarios.yaml."""
    path = os.path.join(get_package_share_directory('rapid_nav_navigation'), 'config',
                        'nav_scenarios.yaml')
    with open(path, encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def yaw_of(orientation):
    """Return the yaw of a quaternion message."""
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                      1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2))


class ObstacleDriver(Node):
    """Move the obstacles of one scenario; publish /scenario/ready and RViz markers."""

    def __init__(self):
        """Declare the scenario parameter; the rest is set up once the obstacles exist."""
        super().__init__('obstacle_driver')
        self.declare_parameter('scenario', '')

    def setup(self, scenario_name, obstacles):
        """Create the publishers, subscribers and timers for the given obstacles."""
        self.obstacles = obstacles
        self.followers = {o['name']: PathFollower(o['path'], o['speed'], o.get('mode', 'once'))
                          for o in obstacles}
        self.triggers = {o['name']: StartTrigger(o.get('start')) for o in obstacles}
        self.pose = {}
        self.robot_xy = None
        self.goal_index = 0
        self.ready = False
        self.cmd_pub = {}
        for obstacle in obstacles:
            name = obstacle['name']
            cmd_topic, odom_topic = obstacle_topics(name)
            self.cmd_pub[name] = self.create_publisher(Twist, cmd_topic, 10)
            self.create_subscription(
                Odometry, odom_topic, lambda msg, n=name: self.on_obstacle(n, msg), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', self.on_robot, 10)
        self.create_subscription(Int32, '/scenario/goal', self.on_goal, LATCHED)
        self.ready_pub = self.create_publisher(Bool, '/scenario/ready', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/scenario/obstacles', 10)
        self.create_timer(0.05, self.drive)
        self.create_timer(0.1, self.publish_markers)
        self.get_logger().info('scenario {}: {} obstacles'.format(scenario_name, len(obstacles)))

    def now_s(self):
        """Return the simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_obstacle(self, name, msg):
        """Remember an obstacle's true pose; announce readiness once all have reported."""
        p = msg.pose.pose
        self.pose[name] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if not self.ready and len(self.pose) == len(self.obstacles):
            self.ready = True
            self.ready_pub.publish(Bool(data=True))
            self.get_logger().info('all {} obstacles spawned: /scenario/ready'.format(
                len(self.obstacles)))

    def on_robot(self, msg):
        """Remember the robot's true position."""
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_goal(self, msg):
        """Remember the goal nav_run is about to send (1-based)."""
        self.goal_index = msg.data

    def drive(self):
        """Command every obstacle: zero until its trigger fires, then along its path."""
        for name, follower in self.followers.items():
            cmd = Twist()
            pose = self.pose.get(name)
            if pose is not None and self.ready:
                was = self.triggers[name].fired
                if self.triggers[name].update(self.goal_index, self.robot_xy) and not was:
                    self.get_logger().info('{} starts at sim t = {:.2f} s'.format(
                        name, self.now_s()))
                if self.triggers[name].fired:
                    vx, vy, wz = follower.step(*pose)
                    cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
            self.cmd_pub[name].publish(cmd)

    def publish_markers(self):
        """Publish one marker per obstacle at its true pose, for RViz."""
        array = MarkerArray()
        for index, obstacle in enumerate(self.obstacles):
            pose = self.pose.get(obstacle['name'])
            if pose is None:
                continue
            spec = SHAPES[obstacle['shape']]
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'scenario'
            marker.id = index
            marker.action = Marker.ADD
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = shape_height(obstacle['shape']) / 2.0
            marker.pose.orientation.z = math.sin(pose[2] / 2.0)
            marker.pose.orientation.w = math.cos(pose[2] / 2.0)
            if spec['kind'] == 'cylinder':
                marker.type = Marker.CYLINDER
                marker.scale.x = marker.scale.y = 2.0 * spec['radius']
                marker.scale.z = spec['height']
            else:
                marker.type = Marker.CUBE
                marker.scale.x, marker.scale.y, marker.scale.z = spec['size']
            marker.color.r, marker.color.g, marker.color.b = spec['colour']
            marker.color.a = 0.8
            array.markers.append(marker)
        self.marker_pub.publish(array)


def spawn(obstacle):
    """Spawn one obstacle at the start of its path; return (exit code, output)."""
    start = obstacle['path'][0]
    command = ['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'arena',
               '-name', obstacle['name'], '-x', str(start['x']), '-y', str(start['y']),
               '-z', str(spawn_height(obstacle['shape'])),
               '-string', obstacle_sdf(obstacle['name'], obstacle['shape'])]
    result = subprocess.run(command, capture_output=True, text=True, timeout=SPAWN_TIMEOUT_S,
                            check=False)
    return result.returncode, result.stdout + result.stderr


def main(args=None):
    """Load the scenario, spawn its obstacles and drive them until shut down."""
    rclpy.init(args=args)
    node = ObstacleDriver()
    name = node.get_parameter('scenario').value
    scenarios = load_scenarios()
    if name not in scenarios:
        print('unknown scenario {!r}'.format(name), file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1
    scenario = scenarios[name]
    validate_scenario(name, scenario)
    obstacles = scenario.get('obstacles') or []
    if not obstacles:
        print('scenario {} has no moving obstacles'.format(name))
        node.destroy_node()
        rclpy.shutdown()
        return 0
    for obstacle in obstacles:
        code, output = spawn(obstacle)
        if code != 0:
            print('spawning {} failed (exit {}):\n{}'.format(obstacle['name'], code, output),
                  file=sys.stderr)
            node.destroy_node()
            rclpy.shutdown()
            return 1
    node.setup(name, obstacles)
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
