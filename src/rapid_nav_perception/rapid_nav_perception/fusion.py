"""
Lidar and camera obstacle tracks fused per obstacle with a constant-velocity Kalman filter.

The lidar gives positions (no velocity) of clusters of scan points that are not part of the static
map; the camera tracks (flow_tracker) give position and velocity with their own sigmas. Both always
update the same filter, each weighted by its covariance: there is no switch between the sources.
New tracks start from lidar clusters only; camera tracks can update an existing track.

A measurement older than a track (the camera is late against the lidar) is used as a measurement
of the track's past state: with D = t_track - t_meas, z = H B x with B = [[I, -D I], [0, I]] (the
constant-velocity step back), and the process noise of the step is added to R.

Pure numpy and the standard library, no ROS imports.
"""

import math

import numpy as np

LINK = 0.15                # m: scan points closer than this belong to one cluster
MIN_POINTS = 3             # points a lidar cluster needs
STATIC_CLEARANCE = 0.15    # m: points this close to an occupied map cell are the static map
MIN_CLUSTER_RADIUS = 0.15  # m: a few points of an arc give a small spread; same floor as flow
Q_ACCEL = 0.5              # (m/s^2)^2: white-acceleration process noise
LIDAR_SIGMA = 0.05         # m: lidar position noise per axis
GATE = 9.21                # chi-square(2), 99 %: Mahalanobis gate on position
CONFIRM = 3                # updates before a track is published
MAX_AGE = 1.0              # s without an update before a track is dropped
INIT_VELOCITY_SIGMA = 1.0  # m/s: velocity sigma of a track born from one lidar cluster
MIN_FLOW_SIGMA = 0.05      # m, m/s: floor for camera sigmas (0 means unknown)


class StaticGrid:
    """The static map as a boolean mask of cells within the clearance of an occupied cell."""

    def __init__(self, occupied, origin_x, origin_y, resolution, clearance=STATIC_CLEARANCE):
        """
        Build the mask from `occupied` (rows = y, columns = x, True = occupied).

        The map's origin is the world position of the lower-left corner of cell (0, 0), with no
        rotation (as map_server publishes it).
        """
        occupied = np.asarray(occupied, dtype=bool)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.resolution = float(resolution)
        self.shape = occupied.shape
        reach = int(math.ceil(clearance / self.resolution))
        near = occupied.copy()
        height, width = occupied.shape
        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                if (di == 0 and dj == 0) or \
                        math.hypot(di, dj) * self.resolution > clearance + 1e-9:
                    continue
                src = occupied[max(0, -dj):height - max(0, dj), max(0, -di):width - max(0, di)]
                near[max(0, dj):height - max(0, -dj), max(0, di):width - max(0, -di)] |= src
        self.near = near

    def is_static(self, xy):
        """Return a mask of the points (N, 2) that are near the static map or outside the grid."""
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        i = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(int)
        j = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(int)
        inside = (i >= 0) & (i < self.shape[1]) & (j >= 0) & (j < self.shape[0])
        out = np.ones(xy.shape[0], dtype=bool)
        out[inside] = self.near[j[inside], i[inside]]
        return out


def lidar_clusters(scan_xy_map, static_grid, link=LINK, min_points=MIN_POINTS):
    """
    Return the clusters of scan points that are not the static map, as dicts x, y, radius, n.

    Points within the clearance of an occupied cell (or outside the map) are dropped; the rest are
    linked when closer than `link` (single linkage) and a group needs `min_points` points. The
    centre is the mean of the points, the radius the largest distance of a point from it, at least
    MIN_CLUSTER_RADIUS. Without a static grid (None) no point is dropped.
    """
    xy = np.asarray(scan_xy_map, dtype=np.float64).reshape(-1, 2)
    if static_grid is not None and xy.shape[0]:
        xy = xy[~static_grid.is_static(xy)]
    n = xy.shape[0]
    if n == 0:
        return []
    d2 = np.sum((xy[:, None, :] - xy[None, :, :]) ** 2, axis=2)
    linked = d2 < link * link
    label = -np.ones(n, dtype=int)
    clusters = []
    for start in range(n):
        if label[start] >= 0:
            continue
        label[start] = start
        stack, members = [start], []
        while stack:
            k = stack.pop()
            members.append(k)
            for m in np.nonzero(linked[k] & (label < 0))[0]:
                label[m] = start
                stack.append(int(m))
        if len(members) < min_points:
            continue
        pts = xy[members]
        centre = pts.mean(axis=0)
        reach = float(np.max(np.hypot(pts[:, 0] - centre[0], pts[:, 1] - centre[1])))
        clusters.append({'x': float(centre[0]), 'y': float(centre[1]),
                         'radius': max(MIN_CLUSTER_RADIUS, reach), 'n': len(members)})
    return clusters


def transition(dt):
    """Return the constant-velocity state transition F(dt) for [x, y, vx, vy]."""
    f = np.eye(4)
    f[0, 2] = f[1, 3] = dt
    return f


def process_noise(dt, q=Q_ACCEL):
    """Return the white-acceleration process noise Q(dt) for [x, y, vx, vy]."""
    dt = abs(dt)
    a, b, c = dt ** 3 / 3.0, dt ** 2 / 2.0, dt
    return q * np.array([[a, 0, b, 0], [0, a, 0, b], [b, 0, c, 0], [0, b, 0, c]])


H_LIDAR = np.hstack([np.eye(2), np.zeros((2, 2))])
H_FLOW = np.eye(4)


class Track:
    """One obstacle: constant-velocity Kalman filter on [x, y, vx, vy] with its time and radius."""

    def __init__(self, tid, x, y, radius, t, q=Q_ACCEL, sigma=LIDAR_SIGMA,
                 velocity_sigma=INIT_VELOCITY_SIGMA):
        """Start a track at a lidar position with zero velocity and a wide velocity sigma."""
        self.id = tid
        self.x = np.array([x, y, 0.0, 0.0], dtype=np.float64)
        self.P = np.diag([sigma ** 2, sigma ** 2, velocity_sigma ** 2, velocity_sigma ** 2])
        self.t = float(t)
        self.radius = float(radius)
        self.q = q
        self.updates = 1
        self.last_update = float(t)

    def predict(self, t):
        """Move the state forward to time t (a time before the track's is ignored)."""
        dt = float(t) - self.t
        if dt <= 0.0:
            return
        f = transition(dt)
        self.x = f @ self.x
        self.P = f @ self.P @ f.T + process_noise(dt, self.q)
        self.t = float(t)

    def predicted(self, t):
        """Return (state, covariance) at time t without changing the track (t <= now: as is)."""
        dt = float(t) - self.t
        if dt <= 0.0:
            return self.x.copy(), self.P.copy()
        f = transition(dt)
        return f @ self.x, f @ self.P @ f.T + process_noise(dt, self.q)

    def model(self, h, r, t):
        """
        Return (H, R) of a measurement taken at time t, after moving the track forward to t.

        For a measurement older than the track by D, H becomes H B(D) with B the step back and
        R gains H Q(D) H^T.
        """
        self.predict(t)
        lag = self.t - float(t)
        if lag <= 0.0:
            return h, r
        back = transition(-lag)
        return h @ back, r + h @ process_noise(lag, self.q) @ h.T

    def gate_distance(self, z, h, r):
        """Return the squared Mahalanobis distance of the measurement's position part."""
        innovation = z - h @ self.x
        s = h @ self.P @ h.T + r
        return float(innovation[:2] @ np.linalg.solve(s[:2, :2], innovation[:2]))

    def update(self, z, h, r, t):
        """Apply the Kalman update of the measurement z = H x + noise(R) taken at time t."""
        innovation = z - h @ self.x
        s = h @ self.P @ h.T + r
        gain = self.P @ h.T @ np.linalg.inv(s)
        self.x = self.x + gain @ innovation
        i_kh = np.eye(4) - gain @ h
        # Joseph form: stays symmetric and positive definite.
        self.P = i_kh @ self.P @ i_kh.T + gain @ r @ gain.T
        self.updates += 1
        self.last_update = max(self.last_update, float(t))


def sigma_of(block):
    """Return the square root of the largest eigenvalue of a 2 x 2 covariance block."""
    return float(math.sqrt(max(0.0, float(np.max(np.linalg.eigvalsh(block))))))


class Fusion:
    """All fused tracks: association, updates from both sensors, confirmation and dropping."""

    def __init__(self, q=Q_ACCEL, lidar_sigma=LIDAR_SIGMA, gate=GATE, confirm=CONFIRM,
                 max_age=MAX_AGE, init_velocity_sigma=INIT_VELOCITY_SIGMA):
        """Set the filter and track-management parameters."""
        self.q = q
        self.lidar_sigma = lidar_sigma
        self.gate = gate
        self.confirm = confirm
        self.max_age = max_age
        self.init_velocity_sigma = init_velocity_sigma
        self.tracks = {}
        self._next_id = 1

    def _drop_old(self, t):
        for tid in [k for k, tr in self.tracks.items() if t - tr.last_update > self.max_age]:
            del self.tracks[tid]

    def _associate(self, measurements, t):
        """
        Pair measurements (z, H, R) with tracks: nearest first, inside the gate, one to one.

        Return the pairs (measurement index, track id, H', R') with the models at the track.
        """
        candidates = []
        for tid, tr in self.tracks.items():
            for mi, (z, h, r) in enumerate(measurements):
                h_t, r_t = tr.model(h, r, t)
                d2 = tr.gate_distance(z, h_t, r_t)
                if d2 <= self.gate:
                    candidates.append((d2, mi, tid, h_t, r_t))
        candidates.sort(key=lambda item: item[0])
        used_m, used_t, pairs = set(), set(), []
        for _, mi, tid, h_t, r_t in candidates:
            if mi in used_m or tid in used_t:
                continue
            used_m.add(mi)
            used_t.add(tid)
            pairs.append((mi, tid, h_t, r_t))
        return pairs

    def update_lidar(self, clusters, t):
        """Update with lidar clusters (dicts x, y, radius) taken at time t; start new tracks."""
        t = float(t)
        self._drop_old(t)
        r = (self.lidar_sigma ** 2) * np.eye(2)
        measurements = [(np.array([c['x'], c['y']], dtype=np.float64), H_LIDAR, r)
                        for c in clusters]
        pairs = self._associate(measurements, t)
        used = set()
        for mi, tid, h_t, r_t in pairs:
            tr = self.tracks[tid]
            tr.update(measurements[mi][0], h_t, r_t, t)
            tr.radius = float(clusters[mi]['radius'])
            used.add(mi)
        for mi, c in enumerate(clusters):
            if mi in used:
                continue
            tid = 'fused_{}'.format(self._next_id)
            self._next_id += 1
            self.tracks[tid] = Track(tid, c['x'], c['y'], c['radius'], t, self.q,
                                     self.lidar_sigma, self.init_velocity_sigma)

    def update_flow(self, obstacles, t):
        """
        Update existing tracks with camera tracks valid at time t.

        The camera tracks are dicts x, y, vx, vy, position_sigma, velocity_sigma; sigmas below
        MIN_FLOW_SIGMA (0 = unknown) are raised to it. Unmatched camera tracks start nothing.
        """
        t = float(t)
        self._drop_old(t)
        measurements = []
        for o in obstacles:
            sp = max(MIN_FLOW_SIGMA, float(o.get('position_sigma', 0.0)))
            sv = max(MIN_FLOW_SIGMA, float(o.get('velocity_sigma', 0.0)))
            z = np.array([o['x'], o['y'], o['vx'], o['vy']], dtype=np.float64)
            measurements.append((z, H_FLOW, np.diag([sp * sp, sp * sp, sv * sv, sv * sv])))
        for mi, tid, h_t, r_t in self._associate(measurements, t):
            self.tracks[tid].update(measurements[mi][0], h_t, r_t, t)

    def confirmed(self, t):
        """
        Return the confirmed tracks predicted to time t.

        Each is a dict id, x, y, vx, vy, radius, position_sigma and velocity_sigma (square roots of
        the largest eigenvalues of the position and velocity blocks of P), and confidence =
        1 / (1 + velocity_sigma).
        """
        out = []
        for tr in self.tracks.values():
            if tr.updates < self.confirm or float(t) - tr.last_update > self.max_age:
                continue
            x, p = tr.predicted(t)
            sv = sigma_of(p[2:, 2:])
            out.append({'id': tr.id, 'x': float(x[0]), 'y': float(x[1]), 'vx': float(x[2]),
                        'vy': float(x[3]), 'radius': tr.radius,
                        'position_sigma': sigma_of(p[:2, :2]), 'velocity_sigma': sv,
                        'confidence': 1.0 / (1.0 + sv)})
        return out
