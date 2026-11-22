"""
Moving regions to tracks: clustering of moving samples on the floor and association over time.

Pure numpy and the standard library, no ROS imports.
"""

import math

import numpy as np


def cluster(xy, w_xy, cell=0.10, min_samples=6):
    """
    Group moving samples into obstacles; return a list of dicts x, y, vx, vy, radius, n.

    The samples are binned by position on a grid of `cell` metres; occupied cells that touch (8
    neighbours) form one cluster, which needs at least `min_samples` samples. The position is the
    median of the sample positions, the velocity the component-wise median of the sample
    velocities, the radius the largest distance of a sample from the position plus 0.05, clamped
    to 0.15..0.60.
    """
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    w_xy = np.asarray(w_xy, dtype=np.float64).reshape(-1, 2)
    if xy.shape[0] == 0:
        return []
    keys = np.floor(xy / cell).astype(int)
    cells = {}
    for index, key in enumerate(map(tuple, keys)):
        cells.setdefault(key, []).append(index)
    seen = set()
    clusters = []
    for start in cells:
        if start in seen:
            continue
        seen.add(start)
        stack = [start]
        members = []
        while stack:
            cx, cy = stack.pop()
            members.extend(cells[(cx, cy)])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in cells and neighbour not in seen:
                        seen.add(neighbour)
                        stack.append(neighbour)
        if len(members) < min_samples:
            continue
        pts = xy[members]
        centre = np.median(pts, axis=0)
        vel = np.median(w_xy[members], axis=0)
        reach = float(np.max(np.hypot(pts[:, 0] - centre[0], pts[:, 1] - centre[1])))
        clusters.append({'x': float(centre[0]), 'y': float(centre[1]), 'vx': float(vel[0]),
                         'vy': float(vel[1]), 'radius': min(0.60, max(0.15, reach + 0.05)),
                         'n': len(members)})
    return clusters


class Tracker:
    """Associate clusters over time; confirm a track after `confirm` matched frames."""

    def __init__(self, gate=0.6, alpha=0.5, max_track_age=1.0, confirm=2):
        """Set the association gate (m), the smoothing, the drop age (s) and the confirm count."""
        self.gate = gate
        self.alpha = alpha
        self.max_track_age = max_track_age
        self.confirm = confirm
        self.tracks = {}
        self._next_id = 1

    def update(self, clusters, t):
        """Update the tracks with the clusters measured at time t (s)."""
        pairs = []
        for tid, tr in self.tracks.items():
            dt = t - tr['t']
            px, py = tr['x'] + tr['vx'] * dt, tr['y'] + tr['vy'] * dt
            for ci, c in enumerate(clusters):
                dist = math.hypot(c['x'] - px, c['y'] - py)
                if dist <= self.gate:
                    pairs.append((dist, tid, ci, px, py))
        pairs.sort(key=lambda item: item[0])
        used_tracks, used_clusters = set(), set()
        a = self.alpha
        for _, tid, ci, px, py in pairs:
            if tid in used_tracks or ci in used_clusters:
                continue
            used_tracks.add(tid)
            used_clusters.add(ci)
            tr, c = self.tracks[tid], clusters[ci]
            tr['x'] = a * c['x'] + (1 - a) * px
            tr['y'] = a * c['y'] + (1 - a) * py
            tr['vx'] = a * c['vx'] + (1 - a) * tr['vx']
            tr['vy'] = a * c['vy'] + (1 - a) * tr['vy']
            tr['radius'] = c['radius']
            tr['t'] = t
            tr['last_matched'] = t
            tr['hits'] += 1
        for tid in [k for k, tr in self.tracks.items()
                    if k not in used_tracks and t - tr['last_matched'] > self.max_track_age]:
            del self.tracks[tid]
        for ci, c in enumerate(clusters):
            if ci in used_clusters:
                continue
            tid = 'flow_{}'.format(self._next_id)
            self._next_id += 1
            self.tracks[tid] = {'id': tid, 'x': c['x'], 'y': c['y'], 'vx': c['vx'], 'vy': c['vy'],
                                'radius': c['radius'], 't': t, 'last_matched': t, 'hits': 1}

    def confirmed(self, t=None):
        """
        Return the confirmed tracks as dicts with id, x, y, vx, vy, radius, confidence, t.

        With a time t, a track that was not matched in the latest frame is moved to t with its
        velocity, so every published position is valid at the stamp of the message.
        """
        out = []
        for tr in self.tracks.values():
            if tr['hits'] >= self.confirm:
                dt = 0.0 if t is None else t - tr['t']
                out.append({'id': tr['id'], 'x': tr['x'] + tr['vx'] * dt,
                            'y': tr['y'] + tr['vy'] * dt, 'vx': tr['vx'],
                            'vy': tr['vy'], 'radius': tr['radius'],
                            'confidence': min(1.0, tr['hits'] / 5.0),
                            't': tr['t'] if t is None else t})
        return out
