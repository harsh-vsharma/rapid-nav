"""
scene_viz: the markers of the minimal RViz view (lidar outline, flow instances and their mesh).

/scan -> /viz/scan_lines (Marker LINE_LIST in the scan frame): consecutive beams closer than
`scan_gap` are joined.

/perception/flow_frame -> /viz/flow (MarkerArray in `frame`, default odom): the flow samples with
the robot's own motion removed (egomotion.py), grouped into instances by velocity gradient
(scene_vis.py). Each frame replaces the last one (nothing is kept once it leaves the view).

A sample counts as moving only above a noise floor (flow error at its depth, pose and colour/depth
timing error from the robot's own motion) and where the picture has texture; samples on depth
edges are dropped. Moving pieces that are close and move alike are merged into one instance, and
an instance is shown as moving only when the previous frame had a consistent one, so flow noise
(whose velocity jumps between frames) stays grey.

    flow_mesh   the convex hull of each compact body (an instance of the velocity grouping; a
                mover's pieces are one body) in one solid colour it keeps across frames
    velocity    one arrow per moving instance, from its centroid, its mean velocity x arrow_time
    rotation    an arc around a moving instance that turns (omega x arc_time radians)
"""

import collections
import math
import sys

from geometry_msgs.msg import Point
import numpy as np
from rapid_nav_msgs.msg import FlowFrame, TrackedObstacleArray
from rapid_nav_perception.conversions import image_to_array
from rapid_nav_perception.egomotion import (
    block_texture, depth_speed_floor, sample_points, texture_mask, world_velocities)
from rapid_nav_perception.flow_tracker import matrix_of, stamp_seconds
from rapid_nav_perception.scene_vis import (
    body_hull, BODY_PALETTE, confirm_movers, depth_edges, depth_jumps, depth_slope, ego_speed,
    index_grid, instance_motion, merge_instances, scan_segments, segment_instances, stable_ids,
    suppress_noise, tile_indices)
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

SCAN_RGBA = (0.90, 0.92, 0.95, 0.95)


def rgba(r, g, b, a):
    """Return a std_msgs/ColorRGBA."""
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def point(x, y, z):
    """Return a geometry_msgs/Point."""
    return Point(x=float(x), y=float(y), z=float(z))


class SceneViz(Node):
    """Publish the lidar outline and the flow instances as markers."""

    DEFAULTS = {
        'frame': 'odom',
        'scan_gap': 0.20,          # m: beams closer than this are joined
        'scan_width': 0.015,       # m: line width of the outline
        'block': 4,                # px: one flow sample per block x block tile
        'min_depth': 0.3, 'max_depth': 8.0,
        'min_point_speed': 0.15,   # m/s: slower samples are static
        'max_point_speed': 3.0,    # m/s: faster samples are errors and dropped
        'flow_k': 2.0,             # sigmas of flow error a sample's speed must exceed
        'flow_sigma_px': 0.5,      # px: flow error
        'ego_fraction': 0.3,       # share of the ego-motion speed that counts as noise
        'sync_frames': 1.0,        # colour/depth misalignment, in frames of ego image motion
        'velocity_tol': 0.25,      # m/s: largest velocity difference between joined neighbours
        'distance_tol': 0.25,      # m: largest distance between joined neighbours
        'merge_distance': 0.6,     # m: moving pieces this close that move alike are one
        'min_samples': 12,         # samples an instance needs to be shown
        'moving_speed': 0.20,      # m/s: instances slower than this are static
        'floor_height': 0.06,      # m: samples lower than this are floor, not a body
        'max_hull_extent': 2.5,    # m: wider bodies (walls) get no hull
        'mesh_alpha': 0.7,         # opacity of the body colours
        'colour_match_m': 0.6,     # m: a body keeps its colour if it moved less than this
        'show_static': True,       # draw the static instances' mesh (faint grey)
        'arrow_time': 1.0,         # s: arrow length = speed x arrow_time
        'arc_time': 1.0,           # s: arc angle = omega x arc_time
        'min_omega': 0.5,          # rad/s: slower turns get no arc
        'max_gap': 0.5,            # s: longest gap between frames for the consistency check
        'min_texture': 6.0,        # structure-tensor texture below which flow is a guess
        'tf_timeout': 0.1,
        'tracks_topic': '/obstacles/tracked',
        'predict_horizon_s': 2.0,  # as the hazard layer (navigation.yaml): what the planner avoids
        'predict_min_speed': 0.05,
    }

    def __init__(self):
        """Declare the parameters, set up TF, the subscriptions and the publishers."""
        super().__init__('scene_viz')
        for name, value in self.DEFAULTS.items():
            self.declare_parameter(name, value)
        self.p = {name: self.get_parameter(name).value for name in self.DEFAULTS}
        # The candidate movers of the last flow frame and its time, for the consistency check.
        self.previous, self.previous_t = [], None
        # The bodies of the last frame (centroid, colour id), so each keeps its colour.
        self.colour_memory, self.next_colour = [], 0
        self.buffer = Buffer()
        # A node of its own for TF, so /tf keeps arriving while a lookup waits.
        self.listener = TransformListener(self.buffer, None, spin_thread=True)
        self.scan_pub = self.create_publisher(Marker, '/viz/scan_lines', 5)
        self.flow_pub = self.create_publisher(MarkerArray, '/viz/flow', 5)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(FlowFrame, '/perception/flow_frame', self.on_flow, 2)
        self.predict_pub = self.create_publisher(MarkerArray, '/viz/predictions', 5)
        self.create_subscription(TrackedObstacleArray, str(self.p['tracks_topic']),
                                 self.on_tracks, 5)
        self.images = collections.deque(maxlen=12)
        if self.p['min_texture'] > 0.0:
            self.create_subscription(
                Image, '/camera/image', self.on_image, qos_profile_sensor_data)

    # -- lidar ---------------------------------------------------------------------------------

    def on_scan(self, msg):
        """Publish the beams joined into an outline."""
        full_turn = abs(msg.angle_increment * len(msg.ranges)) >= 2.0 * math.pi - 1e-3
        segments = scan_segments(msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max, float(self.p['scan_gap']),
                                 wrap=full_turn)
        m = Marker()
        m.header = msg.header
        m.ns, m.id, m.type, m.action = 'scan', 0, Marker.LINE_LIST, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = float(self.p['scan_width'])
        m.color = rgba(*SCAN_RGBA)
        m.points = [point(x, y, 0.0) for x, y in segments.reshape(-1, 2)]
        self.scan_pub.publish(m)

    # -- predicted obstacles ---------------------------------------------------------------------

    def on_tracks(self, msg):
        """Draw where each moving track will be at the end of the hazard layer's horizon."""
        p = self.p
        horizon = float(p['predict_horizon_s'])
        lines = self.marker(msg.header, 'predicted', 0, Marker.LINE_LIST)
        lines.scale.x = 0.02
        lines.color = rgba(1.0, 0.70, 0.35, 0.55)
        ends = self.marker(msg.header, 'predicted', 1, Marker.SPHERE_LIST)
        ends.scale.x = ends.scale.y = ends.scale.z = 0.10
        ends.color = rgba(1.0, 0.70, 0.35, 0.85)
        for o in msg.obstacles:
            x, y, vx, vy = o.position.x, o.position.y, o.velocity.x, o.velocity.y
            if math.hypot(vx, vy) < float(p['predict_min_speed']):
                continue
            end = point(x + vx * horizon, y + vy * horizon, 0.03)
            lines.points += [point(x, y, 0.03), end]
            ends.points.append(end)
        out = [m for m in (lines, ends) if m.points]
        self.predict_pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)] + out))

    # -- flow ----------------------------------------------------------------------------------

    def pose(self, frame, stamp):
        """Return the matrix frame -> target at the stamp, or None."""
        try:
            t = self.buffer.lookup_transform(
                self.p['frame'], frame,
                Time(seconds=stamp_seconds(stamp), clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=float(self.p['tf_timeout'])))
        except TransformException:
            return None
        return matrix_of(t.transform)

    def on_image(self, msg):
        """Keep the grey picture of the recent camera frames, for the texture test."""
        grey = image_to_array(msg).astype(np.float32).mean(axis=2)
        self.images.append((stamp_seconds(msg.header.stamp), grey))

    def texture(self, stamp, shape, block):
        """Return the texture per tile of the picture at the stamp (flow size), or None."""
        if not self.images:
            return None
        seconds = stamp_seconds(stamp)
        when, grey = min(self.images, key=lambda e: abs(e[0] - seconds))
        if abs(when - seconds) > 0.05:
            return None
        factor = max(1, grey.shape[0] // shape[0])
        if factor > 1:
            h, w = shape[0] * factor, shape[1] * factor
            grey = grey[:h, :w].reshape(shape[0], factor, shape[1], factor).mean(axis=(1, 3))
        return block_texture(grey, block)

    def on_flow(self, msg):
        """Group the flow samples into instances and publish their mesh, arrows and arcs."""
        s0, s1 = msg.previous_depth.header.stamp, msg.depth.header.stamp
        dt = stamp_seconds(s1) - stamp_seconds(s0)
        if dt <= 0.0:
            return
        pose0, pose1 = self.pose(msg.header.frame_id, s0), self.pose(msg.header.frame_id, s1)
        if pose0 is None or pose1 is None:
            return
        p, k, block = self.p, msg.camera_info.k, int(self.p['block'])
        flow = image_to_array(msg.flow)
        pix, x0, x1 = sample_points(
            flow, image_to_array(msg.previous_depth), image_to_array(msg.depth),
            (k[0], k[4], k[2], k[5]), block, p['min_depth'], p['max_depth'], block)
        # Where the picture has no texture in two directions the flow is a guess: such samples
        # still make the mesh but count as static.
        textured = np.ones(x0.shape[0], dtype=bool)
        if p['min_texture'] > 0.0:
            texture = self.texture(msg.previous_stamp, flow.shape[:2], block)
            if texture is not None:
                textured = texture_mask(pix, texture, block, p['min_texture'])
        markers = [Marker(action=Marker.DELETEALL)]
        if x0.shape[0]:
            markers += self.flow_markers(msg, pix, x0, x1, textured, pose0, pose1, dt)
        self.flow_pub.publish(MarkerArray(markers=markers))

    def flow_markers(self, msg, pix, x0, x1, textured, pose0, pose1, dt):
        """Return the mesh, arrow and arc markers of one flow frame."""
        p, block = self.p, int(self.p['block'])
        shape = (msg.depth.height // block, msg.depth.width // block)
        rc = tile_indices(pix, block)
        inside = (rc[:, 0] >= 0) & (rc[:, 0] < shape[0]) & (rc[:, 1] >= 0) & (rc[:, 1] < shape[1])
        rc, x0, x1, textured = rc[inside], x0[inside], x1[inside], textured[inside]
        # Drop the samples on depth edges (flow there mixes two surfaces), those whose depth jumps
        # between the frames (background uncovered or covered: the flow is about zero there but
        # the depth switches between object and background) and the impossible ones.
        full_grid = index_grid(rc, shape)
        keep = ~depth_edges(full_grid, x1[:, 2]) & ~depth_jumps(x0[:, 2], x1[:, 2])
        slope = depth_slope(full_grid, x1[:, 2])[keep]
        points, velocities = world_velocities(x0[keep], x1[keep], pose0, pose1, dt)
        rc, x0, x1, textured = rc[keep], x0[keep], x1[keep], textured[keep]
        sane = np.linalg.norm(velocities, axis=1) <= float(p['max_point_speed'])
        rc, x0, x1, slope = rc[sane], x0[sane], x1[sane], slope[sane]
        textured = textured[sane]
        points, velocities = points[sane], velocities[sane]
        # Noise floor per sample: the speed flow error fakes at its depth, the share of the
        # robot's own apparent motion that pose error fakes, what a colour/depth misalignment of
        # sync_frames x the ego-motion image shift fakes on a slanted surface, and a fixed
        # minimum. Below it the sample is static, so the static scene stays one calm instance.
        k = msg.camera_info.k
        ego = ego_speed(x0, pose0, pose1, dt)
        tiles_per_s = k[0] * ego / (np.maximum(x1[:, 2], 1e-3) * block)
        floor = np.maximum.reduce([
            np.full(points.shape[0], float(p['min_point_speed'])),
            depth_speed_floor(x1[:, 2], k[0], dt, float(p['flow_k']), float(p['flow_sigma_px'])),
            float(p['ego_fraction']) * ego,
            float(p['sync_frames']) * slope * tiles_per_s,
        ])
        floor[~textured] = np.inf
        velocities = suppress_noise(velocities, floor)
        # The floor is not a body: without it a crate and the floor it stands on stay apart.
        above = points[:, 2] >= float(p['floor_height'])
        rc, points, velocities = rc[above], points[above], velocities[above]
        grid = index_grid(rc, shape)
        labels = segment_instances(grid, points, velocities, p['velocity_tol'],
                                   p['distance_tol'])
        instances = instance_motion(points, velocities, labels, int(p['min_samples']))

        # Bodies: each confirmed mover (its merged pieces) and each other instance. Every body is
        # drawn in one solid colour that it keeps from frame to frame (stable_ids).
        n_labels = int(labels.max()) + 1 if labels.size else 0
        body_of = np.full(n_labels, -1, dtype=int)
        bodies, moving = [], []
        # A fast instance is a mover only when the last frame had a consistent one (see
        # confirm_movers); a frame more than max_gap later starts afresh.
        t1 = stamp_seconds(msg.depth.header.stamp)
        candidates = merge_instances(
            [i for i in instances if math.hypot(*i['velocity'][:2]) >= p['moving_speed']],
            float(p['merge_distance']), float(p['velocity_tol']))
        gap = t1 - self.previous_t if self.previous_t is not None else None
        if gap is not None and 0.0 < gap <= float(p['max_gap']):
            confirmed = confirm_movers(candidates, self.previous, gap)
        else:
            confirmed = [False] * len(candidates)
        self.previous, self.previous_t = candidates, t1
        for mover, ok in zip(candidates, confirmed):
            if ok:
                body_of[mover['labels']] = len(bodies)
                bodies.append(mover)
                moving.append(mover)
        if p['show_static']:
            for inst in instances:
                if body_of[inst['label']] < 0:
                    body_of[inst['label']] = len(bodies)
                    bodies.append(inst)
        ids, self.next_colour = stable_ids([b['centroid'] for b in bodies], self.colour_memory,
                                           float(p['colour_match_m']), self.next_colour)
        self.colour_memory = [(b['centroid'], i) for b, i in zip(bodies, ids)]
        colours = [rgba(*BODY_PALETTE[i % len(BODY_PALETTE)], float(p['mesh_alpha']))
                   for i in ids]

        header = msg.depth.header
        header.frame_id = p['frame']
        out = []
        # One solid convex hull per compact body (walls are left to the lidar outline).
        sample_body = body_of[labels] if labels.size else np.zeros(0, dtype=int)
        for b in range(len(bodies)):
            body_points = points[sample_body == b]
            tris = body_hull(body_points, float(p['max_hull_extent']))
            if not tris.size:
                continue
            mesh = self.marker(header, 'flow_mesh', b, Marker.TRIANGLE_LIST)
            mesh.scale.x = mesh.scale.y = mesh.scale.z = 1.0
            mesh.color = colours[b]
            mesh.points = [point(*xyz) for xyz in body_points[tris.ravel()]]
            out.append(mesh)
        for index, mover in enumerate(moving):  # the movers are the first bodies
            out += self.instance_markers(header, index, mover, colours[index])
        return out

    def instance_markers(self, header, index, inst, colour):
        """Return the arrow (mean velocity) and the arc (rotation) of one moving instance."""
        p = self.p
        c, v = inst['centroid'], inst['velocity']
        solid = rgba(colour.r, colour.g, colour.b, 1.0)
        arrow = self.marker(header, 'velocity', index, Marker.ARROW)
        arrow.points = [point(*c), point(*(c + v * float(p['arrow_time'])))]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.035, 0.08, 0.10
        arrow.color = solid
        out = [arrow]
        omega = inst['omega']
        if abs(omega) >= p['min_omega']:
            sweep = max(-1.5 * math.pi, min(1.5 * math.pi, omega * float(p['arc_time'])))
            radius = max(0.15, min(0.6, 1.2 * inst['size']))
            arc = self.marker(header, 'rotation', index, Marker.LINE_STRIP)
            arc.scale.x = 0.02
            arc.color = solid
            arc.points = [point(c[0] + radius * math.cos(a), c[1] + radius * math.sin(a), c[2])
                          for a in np.linspace(0.0, sweep, 24)]
            # A short tick at the end of the arc shows which way it turns.
            end = sweep
            tip = arc.points[-1]
            back = end - math.copysign(0.25, sweep)
            arc.points.append(point(c[0] + 0.8 * radius * math.cos(back),
                                    c[1] + 0.8 * radius * math.sin(back), c[2]))
            arc.points.append(tip)
            out.append(arc)
        return out

    @staticmethod
    def marker(header, ns, marker_id, marker_type):
        """Return an ADD marker with an identity pose and a short lifetime."""
        m = Marker()
        m.header = header
        m.ns, m.id, m.type, m.action = ns, marker_id, marker_type, Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime = Duration(seconds=0.5).to_msg()
        return m


def main(args=None):
    """Run the node until shut down."""
    rclpy.init(args=args)
    node = SceneViz()
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
