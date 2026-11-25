#!/usr/bin/env python3
"""
Time-to-collision speed governor: a reactive speed limit on top of Nav2's plan.

For every tracked obstacle the robot and the obstacle are moved on with constant velocity; t* is
the time of their closest approach and d* the distance then, less both radii and 2 sigma(t*)
(sigma(t)^2 = position_sigma^2 + velocity_sigma^2 t^2). When d* < 0 with 0 < t* < horizon the
limit is clamp(t* / horizon, 0.2, 1.0), else 1.0; the smallest limit over the obstacles is
published as a percentage on speed_limit (nav2_msgs/SpeedLimit), which the controller server
applies. Drops apply at once; rises are limited to 20 % per second.

The robot's pose comes from TF (map <- base_footprint), its velocity from the odometry (body
frame, rotated into the map).
"""

import math
import sys
import time

HORIZON_S = 3.0
MIN_LIMIT = 0.2
RISE_PER_S = 0.2
ROBOT_RADIUS = 0.25


def closest_approach(rel_p, rel_v):
    """
    Return (t*, d*) for relative position rel_p and relative velocity rel_v (obstacle - robot).

    t* = -(p . v) / |v|^2 is the time of the closest approach, d* = |p + v t*|. Without relative
    motion t* = 0 and d* = |p|.
    """
    px, py = rel_p
    vx, vy = rel_v
    vv = vx * vx + vy * vy
    if vv < 1e-12:
        return 0.0, math.hypot(px, py)
    t = -(px * vx + py * vy) / vv
    return t, math.hypot(px + vx * t, py + vy * t)


def obstacle_limit(robot_p, robot_v, obstacle, robot_radius=ROBOT_RADIUS, horizon=HORIZON_S,
                   min_limit=MIN_LIMIT):
    """
    Return the speed limit (0..1) one obstacle asks for, and (t*, inflated d*).

    `obstacle` is a dict x, y, vx, vy, radius and optionally position_sigma, velocity_sigma.
    """
    t, d = closest_approach((obstacle['x'] - robot_p[0], obstacle['y'] - robot_p[1]),
                            (obstacle['vx'] - robot_v[0], obstacle['vy'] - robot_v[1]))
    sp = float(obstacle.get('position_sigma', 0.0))
    sv = float(obstacle.get('velocity_sigma', 0.0))
    sigma = math.sqrt(sp * sp + sv * sv * t * t)
    inflated = d - obstacle['radius'] - robot_radius - 2.0 * sigma
    if inflated < 0.0 and 0.0 < t < horizon:
        return min(1.0, max(min_limit, t / horizon)), (t, inflated)
    return 1.0, (t, inflated)


def speed_limit(robot_p, robot_v, obstacles, robot_radius=ROBOT_RADIUS, horizon=HORIZON_S,
                min_limit=MIN_LIMIT):
    """Return the smallest limit (0..1) over the obstacles (1.0 when there are none)."""
    limit = 1.0
    for o in obstacles:
        limit = min(limit, obstacle_limit(robot_p, robot_v, o, robot_radius, horizon,
                                          min_limit)[0])
    return limit


class RateLimiter:
    """Pass drops at once, let rises through at most `rise_per_s` per second."""

    def __init__(self, rise_per_s=RISE_PER_S, start=1.0):
        """Start at `start`."""
        self.rise_per_s = rise_per_s
        self.value = start

    def step(self, target, dt):
        """Move towards the target over dt seconds and return the new value."""
        if target <= self.value:
            self.value = target
        else:
            self.value = min(target, self.value + self.rise_per_s * max(0.0, dt))
        return self.value


def yaw_of(q):
    """Return the yaw of a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def make_node():
    """Build the ROS node (the ROS imports stay here so that the helpers import without ROS)."""
    from nav2_msgs.msg import SpeedLimit
    from nav_msgs.msg import Odometry
    from rapid_nav_msgs.msg import TrackedObstacleArray
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    class TtcGovernor(Node):
        """Publish a speed limit from the time to collision with the tracked obstacles."""

        def __init__(self):
            super().__init__('ttc_governor')
            defaults = {'rate': 10.0, 'horizon_s': HORIZON_S, 'min_limit': MIN_LIMIT,
                        'rise_per_s': RISE_PER_S, 'robot_radius': ROBOT_RADIUS,
                        'max_age_s': 0.5, 'enabled': True}
            for name, value in defaults.items():
                self.declare_parameter(name, value)
            self.declare_parameter('global_frame', 'map')
            self.declare_parameter('robot_frame', 'base_footprint')
            self.declare_parameter('odom_topic', '/odom')
            self.declare_parameter('tracks_topic', '/obstacles/tracked')
            self.declare_parameter('speed_limit_topic', 'speed_limit')
            self.p = {name: self.get_parameter(name).value for name in defaults}
            self.global_frame = self.get_parameter('global_frame').value
            self.robot_frame = self.get_parameter('robot_frame').value
            self.limiter = RateLimiter(float(self.p['rise_per_s']))
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self, spin_thread=True)
            self.pub = self.create_publisher(
                SpeedLimit, self.get_parameter('speed_limit_topic').value, 10)
            self.create_subscription(Odometry, self.get_parameter('odom_topic').value,
                                     self.on_odom, qos_profile_sensor_data)
            self.create_subscription(TrackedObstacleArray,
                                     self.get_parameter('tracks_topic').value, self.on_tracks, 10)
            self.body_v = (0.0, 0.0)
            self.tracks = None
            self.last_tick = None
            self.limited_s = 0.0
            self.total_s = 0.0
            self.min_seen = 1.0
            self.no_tf = 0
            self.last_report = time.monotonic()
            self.SpeedLimit, self.Time = SpeedLimit, Time
            self.TransformException = TransformException
            self.create_timer(1.0 / float(self.p['rate']), self.tick)

        def on_odom(self, msg):
            self.body_v = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

        def on_tracks(self, msg):
            self.tracks = msg

        def robot_state(self):
            """Return ((x, y), (vx, vy)) in the global frame, or None without TF."""
            try:
                t = self.buffer.lookup_transform(self.global_frame, self.robot_frame,
                                                 self.Time())
            except self.TransformException:
                return None
            d, q = t.transform.translation, t.transform.rotation
            yaw = yaw_of((q.x, q.y, q.z, q.w))
            c, s = math.cos(yaw), math.sin(yaw)
            vx, vy = self.body_v
            return (d.x, d.y), (c * vx - s * vy, s * vx + c * vy)

        def obstacles_now(self, now_s):
            """Return the tracked obstacles moved to now (none when the message is too old)."""
            msg = self.tracks
            if msg is None or msg.header.frame_id != self.global_frame:
                return []
            age = now_s - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            if age > float(self.p['max_age_s']):
                return []
            age = max(0.0, age)
            return [{'x': o.position.x + o.velocity.x * age,
                     'y': o.position.y + o.velocity.y * age,
                     'vx': o.velocity.x, 'vy': o.velocity.y, 'radius': o.radius,
                     'position_sigma': o.position_sigma, 'velocity_sigma': o.velocity_sigma}
                    for o in msg.obstacles]

        def tick(self):
            now = self.get_clock().now()
            now_s = now.nanoseconds * 1e-9
            dt = 0.0 if self.last_tick is None else max(0.0, now_s - self.last_tick)
            self.last_tick = now_s
            target = 1.0
            if self.p['enabled']:
                state = self.robot_state()
                if state is None:
                    self.no_tf += 1
                else:
                    target = speed_limit(state[0], state[1], self.obstacles_now(now_s),
                                         float(self.p['robot_radius']),
                                         float(self.p['horizon_s']),
                                         float(self.p['min_limit']))
            value = self.limiter.step(target, dt)
            self.total_s += dt
            if value < 1.0:
                self.limited_s += dt
            self.min_seen = min(self.min_seen, value)
            out = self.SpeedLimit()
            out.header.stamp = now.to_msg()
            out.header.frame_id = self.global_frame
            out.percentage = True
            out.speed_limit = 100.0 * value
            self.pub.publish(out)
            self.report()

        def report(self, force=False):
            wall = time.monotonic()
            if not force and wall - self.last_report < 5.0:
                return
            self.last_report = wall
            self.get_logger().info(
                'ttc_governor: limited_s={:.1f} total_s={:.1f} min_limit_pct={:.0f} '
                'no_tf={}'.format(self.limited_s, self.total_s, 100.0 * self.min_seen,
                                  self.no_tf))

    return TtcGovernor()


def main(args=None):
    """Run the node until shut down."""
    import rclpy
    rclpy.init(args=args)
    node = make_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report(force=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
