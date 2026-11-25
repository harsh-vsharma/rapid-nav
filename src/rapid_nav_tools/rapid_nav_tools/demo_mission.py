"""
demo_mission: the demo's task, a patrol of checkpoints driven through Nav2, drawn in RViz.

The robot drives to each checkpoint in turn (NavigateToPose in the map frame; the SLAM map frame
starts at the robot's spawn, which is the arena origin) and starts over after the last one. A
checkpoint that fails is retried once, then skipped (a goal Nav2 rejects while it starts up is
simply sent again); one that takes longer than goal_timeout_s is
cancelled and counts as a failure. The first goal also starts the scenario's moving obstacles
(/scenario/goal, as nav_run does). With map_file set, the SLAM pose graph is saved there after
every checkpoint reached (slam_toolbox's serialize_map), and the next demo continues from it.

/viz/mission (MarkerArray, map frame): the route as a faint loop, each checkpoint as a numbered
ring: amber the current target, teal reached on this lap, grey still to come.
"""

import math
import sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import ColorRGBA, Int32
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
# x, y, yaw: east end, north-east corner, north, west past the cart's loop, south, home. All are
# goals of the tour and crowd scenarios, clear of the crates, the pillars and the cart's loop.
CHECKPOINTS = [3.5, 0.0, 0.0,
               3.5, 2.8, 1.5708,
               -1.0, 3.2, 3.1416,
               -4.0, 1.0, -1.5708,
               -1.0, -3.0, 0.0,
               0.0, 0.0, 0.0]
PENDING, ACTIVE, REACHED, SKIPPED = 'pending', 'active', 'reached', 'skipped'
COLOURS = {PENDING: (0.75, 0.78, 0.82, 0.55), ACTIVE: (1.0, 0.72, 0.20, 1.0),
           REACHED: (0.25, 0.80, 0.70, 0.9), SKIPPED: (0.85, 0.35, 0.35, 0.7)}


def parse_checkpoints(flat):
    """Return [(x, y, yaw), ...] from a flat list of numbers (x, y, yaw per checkpoint)."""
    values = [float(v) for v in flat]
    if not values or len(values) % 3:
        raise ValueError('checkpoints needs x, y, yaw per checkpoint, got {} numbers'.format(
            len(values)))
    return [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]


def yaw_quaternion(yaw):
    """Return (z, w) of the quaternion of a rotation by yaw about z."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Mission:
    """The patrol's bookkeeping, without ROS: which checkpoint is next and what each one is."""

    def __init__(self, count, retries=1):
        """Start at checkpoint 0 of lap 1, every checkpoint pending."""
        self.count, self.retries = count, retries
        self.index, self.lap, self.attempts = 0, 1, 0
        self.state = [PENDING] * count
        self.state[0] = ACTIVE

    def finished(self, success):
        """Record the result of the active checkpoint and move on (or retry it)."""
        if not success and self.attempts < self.retries:
            self.attempts += 1
            return
        self.state[self.index] = REACHED if success else SKIPPED
        self.attempts = 0
        self.index += 1
        if self.index == self.count:
            self.index, self.lap = 0, self.lap + 1
            self.state = [PENDING] * self.count
        self.state[self.index] = ACTIVE


class DemoMission(Node):
    """Drive the checkpoints in a loop and draw them."""

    def __init__(self):
        """Read the parameters, create the action client, the publishers and the timer."""
        super().__init__('demo_mission')
        self.declare_parameter('checkpoints', CHECKPOINTS)
        self.declare_parameter('goal_timeout_s', 90.0)
        self.declare_parameter('pause_s', 1.0)
        self.declare_parameter('map_file', '')
        self.map_file = str(self.get_parameter('map_file').value)
        self.saver = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        self.saving = False
        self.checkpoints = parse_checkpoints(self.get_parameter('checkpoints').value)
        self.timeout = float(self.get_parameter('goal_timeout_s').value)
        self.pause = float(self.get_parameter('pause_s').value)
        self.mission = Mission(len(self.checkpoints))
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.goal_pub = self.create_publisher(Int32, '/scenario/goal', LATCHED)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/mission', LATCHED)
        self.goal_handle = None        # the accepted goal, while one runs
        self.sending = False      # a goal is on its way to the server
        self.started = None       # sim time the running goal was sent
        self.resume_at = 0.0      # sim time the next goal may be sent
        self.goals_sent = 0
        self.create_timer(0.5, self.tick)

    def now(self):
        """Return the node's (simulated) time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        """Send the next goal when idle, cancel one that takes too long, redraw the markers."""
        now = self.now()
        if self.goal_handle is None and not self.sending and now >= self.resume_at:
            if self.client.server_is_ready():
                self.send(now)
        elif self.goal_handle is not None and now - self.started > self.timeout:
            self.get_logger().warning('checkpoint {}: no arrival in {:.0f} s, cancelling'.format(
                self.mission.index + 1, self.timeout))
            self.goal_handle.cancel_goal_async()
            self.started = now  # do not cancel again while the result is on its way
        self.draw(now)

    def send(self, now):
        """Send the active checkpoint to Nav2."""
        x, y, yaw = self.checkpoints[self.mission.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = yaw_quaternion(yaw)
        self.goals_sent += 1
        # The scenario's obstacles start on goal 1 (and some wait for later goals).
        self.goal_pub.publish(Int32(data=self.goals_sent))
        self.sending, self.started = True, now
        self.get_logger().info('lap {}: checkpoint {} at ({:.1f}, {:.1f})'.format(
            self.mission.lap, self.mission.index + 1, x, y))
        self.client.send_goal_async(goal).add_done_callback(self.on_accepted)

    def on_accepted(self, future):
        """Keep the goal handle, or count a rejected goal as a failure."""
        self.sending = False
        handle = future.result()
        if handle is None or not handle.accepted:
            # Nav2 rejects goals until its lifecycle nodes are active: try again shortly, without
            # counting it against the checkpoint.
            self.goals_sent -= 1
            self.resume_at = self.now() + 2.0
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        """Move on when Nav2 reports the goal's outcome."""
        status = future.result().status
        self.goal_handle = None
        self.done(status == GoalStatus.STATUS_SUCCEEDED,
                  {GoalStatus.STATUS_ABORTED: 'aborted',
                   GoalStatus.STATUS_CANCELED: 'cancelled'}.get(status, str(status)))

    def done(self, success, why=''):
        """Record the outcome and wait pause_s before the next goal."""
        index = self.mission.index + 1
        if success:
            self.get_logger().info('checkpoint {} reached'.format(index))
            self.save_map()
        else:
            self.get_logger().warning('checkpoint {} failed ({})'.format(index, why))
        self.mission.finished(success)
        self.resume_at = self.now() + self.pause

    def save_map(self):
        """Ask slam_toolbox to save the pose graph (skipped while a save runs or SLAM is off)."""
        if not self.map_file or self.saving or not self.saver.service_is_ready():
            return
        self.saving = True
        future = self.saver.call_async(SerializePoseGraph.Request(filename=self.map_file))
        future.add_done_callback(self.on_saved)

    def on_saved(self, future):
        """Log the outcome of a map save."""
        self.saving = False
        result = future.result()
        if result is not None and result.result == SerializePoseGraph.Response.RESULT_SUCCESS:
            self.get_logger().info('map saved to {}.posegraph'.format(self.map_file))
        else:
            self.get_logger().warning('map save to {} failed'.format(self.map_file))

    def draw(self, now):
        """Publish the route and the checkpoint rings."""
        markers = []
        route = self.marker('route', 0, Marker.LINE_STRIP)
        route.scale.x = 0.015
        route.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.18)
        route.points = [Point(x=x, y=y, z=0.01) for x, y, _ in
                        self.checkpoints + self.checkpoints[:1]]
        markers.append(route)
        for i, ((x, y, _), state) in enumerate(zip(self.checkpoints, self.mission.state)):
            colour = ColorRGBA(**dict(zip('rgba', COLOURS[state])))
            radius = 0.30 + (0.05 * math.sin(3.0 * now) if state == ACTIVE else 0.0)
            ring = self.marker('checkpoint', i, Marker.LINE_STRIP)
            ring.scale.x = 0.04 if state == ACTIVE else 0.025
            ring.color = colour
            ring.points = [Point(x=x + radius * math.cos(a), y=y + radius * math.sin(a), z=0.02)
                           for a in (2.0 * math.pi * k / 40 for k in range(41))]
            disc = self.marker('checkpoint_fill', i, Marker.CYLINDER)
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.005
            disc.scale.x = disc.scale.y = 2.0 * radius
            disc.scale.z = 0.01
            disc.color = ColorRGBA(r=colour.r, g=colour.g, b=colour.b, a=0.15)
            label = self.marker('label', i, Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y, 0.45
            label.scale.z = 0.28
            label.color = colour
            label.text = str(i + 1)
            markers += [ring, disc, label]
        self.marker_pub.publish(MarkerArray(markers=markers))

    def marker(self, ns, marker_id, marker_type):
        """Return an ADD marker in the map frame with an identity pose."""
        m = Marker()
        m.header.frame_id = 'map'  # stamp 0: RViz uses the latest map pose
        m.ns, m.id, m.type, m.action = ns, marker_id, marker_type, Marker.ADD
        m.pose.orientation.w = 1.0
        return m


def main(args=None):
    """Run the mission until shut down."""
    rclpy.init(args=args)
    node = DemoMission()
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
