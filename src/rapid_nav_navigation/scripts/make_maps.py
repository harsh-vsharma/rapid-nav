"""
Generate the occupancy maps of the navigation stack: the arena and a synthetic maze.

Run it with numpy and Pillow installed (developers run it; the build never does):

    python3 scripts/make_maps.py [--out DIR] [--previews DIR]

DIR is the maps folder (default: maps/ of this package). Each map is written as DIR/<name>.pgm (P5, 0 =
occupied, 254 = free) and DIR/<name>.yaml (trinary mode, 0.05 m, the origin of the lower-left cell corner). The
output is deterministic: running the script again writes identical files. --previews writes PNG pictures of the
maps for the report; nothing is committed from it.

The arena map is rasterised from the arena's own geometry: the collision shapes of the arena model and of the
world's crates and pillars. A shape counts when its vertical extent overlaps the robot's height band 0.02 to
0.40 m (this drops the floor).

The maze is a fixed list of rectangles (MAZE_WALLS). The drawing shows it: north is up, one character is 0.5 m
(a block is '#' when any part of it is a wall), and x runs from -5 m (left) to 5 m (right). The divider at x = 0
has the 0.70 m door (y 0.8 to 1.5) and the 0.45 m door (y -1.8 to -1.35). The U-shaped trap opens to the west at
x = 2 m. The dead-end corridor north of the U is entered through its 0.9 m mouth at y = 2 m.

     y
   3.75 ####################
   3.25 #........##.####...#
   2.75 #........##.####...#
   2.25 #........##.####...#
   1.75 #........##.####...#
   1.25 #.............####.#
   0.75 #........##...####.#
   0.25 #........##.....##.#
  -0.25 #........##.....##.#
  -0.75 #........##...####.#
  -1.25 #........##...####.#
  -1.75 #........##........#
  -2.25 #........##........#
  -2.75 #........##........#
  -3.25 #........##........#
  -3.75 ####################
        x: -5 m ........................ 0 m ........................ 5 m
"""


import argparse
import math
import os
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

RESOLUTION = 0.05
ROBOT_HEIGHT_BAND = (0.02, 0.40)
ARENA_X = (-5.5, 5.5)
ARENA_Y = (-4.5, 4.5)
MAZE_X = (-5.0, 5.0)
MAZE_Y = (-4.0, 4.0)
OCCUPIED_PIXEL = 0
FREE_PIXEL = 254
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GAZEBO_DIR = os.path.join(PACKAGE_DIR, '..', 'rapid_nav_gazebo')
ARENA_MODEL = os.path.join(GAZEBO_DIR, 'models', 'rapid_nav_arena', 'model.sdf')
ARENA_WORLD = os.path.join(GAZEBO_DIR, 'worlds', 'arena.sdf')

# The maze: walls as (x0, y0, x1, y1) rectangles in metres. Doors: 0.70 m at y 0.8 to 1.5 and 0.45 m at y -1.8 to
# -1.35 in the divider at x = 0; a U-shaped trap that opens to the west at x 2.0 to 3.55; a dead-end corridor north
# of the U, entered through a 0.9 m mouth at y 2.0.
MAZE_WALLS = [
    # outer wall
    (-5.05, -4.05, 5.05, -3.95),
    (-5.05, 3.95, 5.05, 4.05),
    (-5.05, -4.05, -4.95, 4.05),
    (4.95, -4.05, 5.05, 4.05),
    # divider with two doors
    (-0.05, -4.0, 0.05, -1.8),
    (-0.05, -1.35, 0.05, 0.8),
    (-0.05, 1.5, 0.05, 4.0),
    # U-shaped trap, open to the west
    (3.45, -1.05, 3.55, 1.05),
    (2.0, 0.95, 3.55, 1.05),
    (2.0, -1.05, 3.55, -0.95),
    # dead-end corridor north of the U: a 0.9 m mouth, closed at the north by the outer wall
    (1.45, 1.95, 1.55, 2.05),
    (2.45, 1.95, 2.55, 2.05),
    (1.45, 2.05, 1.55, 3.95),
    (2.45, 2.05, 2.55, 3.95),
]


def dilate(mask, radius_cells):
    """Return the boolean mask grown by a disk of the given radius in cells (pure numpy)."""
    radius = int(radius_cells)
    padded = np.pad(mask, radius, mode='constant', constant_values=False)
    grown = np.zeros_like(mask)
    rows, cols = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue
            grown |= padded[radius + dy:radius + dy + rows, radius + dx:radius + dx + cols]
    return grown


def parse_pose(element):
    """Return (x, y, z, yaw) of an optional <pose> child; zeros when absent."""
    pose = element.find('pose') if element is not None else None
    if pose is None or not pose.text:
        return (0.0, 0.0, 0.0, 0.0)
    values = [float(value) for value in pose.text.split()]
    values += [0.0] * (6 - len(values))
    return (values[0], values[1], values[2], values[5])


def compose(parent, child):
    """Return the pose of child inside parent, for poses with yaw only (x, y, z, yaw)."""
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    cos_yaw, sin_yaw = math.cos(pyaw), math.sin(pyaw)
    return (px + cos_yaw * cx - sin_yaw * cy, py + sin_yaw * cx + cos_yaw * cy, pz + cz, pyaw + cyaw)


def collision_shapes(link_element, parent_pose):
    """Return the shapes of the <collision> elements of a link as dictionaries."""
    shapes = []
    for collision in link_element.findall('collision'):
        pose = compose(parent_pose, parse_pose(collision))
        geometry = collision.find('geometry')
        box = geometry.find('box') if geometry is not None else None
        cylinder = geometry.find('cylinder') if geometry is not None else None
        if box is not None:
            size = [float(value) for value in box.find('size').text.split()]
            shapes.append({'kind': 'box', 'pose': pose, 'half': (size[0] / 2.0, size[1] / 2.0),
                           'z': (pose[2] - size[2] / 2.0, pose[2] + size[2] / 2.0)})
        elif cylinder is not None:
            radius = float(cylinder.find('radius').text)
            length = float(cylinder.find('length').text)
            shapes.append({'kind': 'circle', 'pose': pose, 'radius': radius,
                           'z': (pose[2] - length / 2.0, pose[2] + length / 2.0)})
    return shapes


def arena_shapes():
    """Return the collision shapes of the arena model and of the world's crates and pillars."""
    shapes = []
    model_root = ET.parse(ARENA_MODEL).getroot().find('model')
    for link in model_root.findall('link'):
        shapes += collision_shapes(link, (0.0, 0.0, 0.0, 0.0))
    world_root = ET.parse(ARENA_WORLD).getroot().find('world')
    for model in world_root.findall('model'):
        model_pose = parse_pose(model)
        for link in model.findall('link'):
            shapes += collision_shapes(link, model_pose)
    low, high = ROBOT_HEIGHT_BAND
    return [shape for shape in shapes if shape['z'][1] > low and shape['z'][0] < high]


def maze_shapes():
    """Return the maze walls as axis-aligned box shapes."""
    shapes = []
    for x0, y0, x1, y1 in MAZE_WALLS:
        shapes.append({'kind': 'box', 'pose': ((x0 + x1) / 2.0, (y0 + y1) / 2.0, 0.0, 0.0),
                       'half': ((x1 - x0) / 2.0, (y1 - y0) / 2.0), 'z': (0.0, 2.0)})
    return shapes


def rasterise(shapes, x_range, y_range):
    """Return a boolean grid (rows = y, columns = x); a cell is occupied when its centre is inside a shape."""
    columns = int(round((x_range[1] - x_range[0]) / RESOLUTION))
    rows = int(round((y_range[1] - y_range[0]) / RESOLUTION))
    xs = x_range[0] + (np.arange(columns) + 0.5) * RESOLUTION
    ys = y_range[0] + (np.arange(rows) + 0.5) * RESOLUTION
    grid_x, grid_y = np.meshgrid(xs, ys)
    occupied = np.zeros((rows, columns), dtype=bool)
    for shape in shapes:
        cx, cy, _cz, yaw = shape['pose']
        dx = grid_x - cx
        dy = grid_y - cy
        if shape['kind'] == 'box':
            half_x, half_y = shape['half']
            u = dx * math.cos(yaw) + dy * math.sin(yaw)
            v = -dx * math.sin(yaw) + dy * math.cos(yaw)
            occupied |= (np.abs(u) <= half_x) & (np.abs(v) <= half_y)
        else:
            occupied |= dx * dx + dy * dy <= shape['radius'] ** 2
    return occupied


def write_map(name, occupied, origin, out_dir, previews_dir=None):
    """Write the PGM and YAML of one map, and optionally a preview PNG."""
    rows, columns = occupied.shape
    pixels = np.where(occupied, OCCUPIED_PIXEL, FREE_PIXEL).astype(np.uint8)
    pixels = pixels[::-1]  # the image's first row is the north (largest y) edge
    pgm_path = os.path.join(out_dir, name + '.pgm')
    with open(pgm_path, 'wb') as handle:
        handle.write('P5\n{} {}\n255\n'.format(columns, rows).encode('ascii'))
        handle.write(pixels.tobytes())
    yaml_text = (
        'image: {name}.pgm\n'
        'mode: trinary\n'
        'resolution: {resolution}\n'
        'origin: [{x0}, {y0}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n'
    ).format(name=name, resolution=RESOLUTION, x0=origin[0], y0=origin[1])
    with open(os.path.join(out_dir, name + '.yaml'), 'w', encoding='utf-8', newline='\n') as handle:
        handle.write(yaml_text)
    if previews_dir is not None:
        os.makedirs(previews_dir, exist_ok=True)
        Image.fromarray(pixels).save(os.path.join(previews_dir, name + '.png'))


def main(argv=None):
    """Write the arena and maze maps; return the exit status."""
    parser = argparse.ArgumentParser(description='Generate the navigation maps.')
    parser.add_argument('--out', default=os.path.join(PACKAGE_DIR, 'maps'),
                        help='folder for the PGM and YAML files (default: maps/ of this package)')
    parser.add_argument('--previews', default=None,
                        help='optional folder for PNG previews (for the report only)')
    args = parser.parse_args(argv)
    os.makedirs(args.out, exist_ok=True)
    arena = rasterise(arena_shapes(), ARENA_X, ARENA_Y)
    write_map('arena', arena, (ARENA_X[0], ARENA_Y[0]), args.out, args.previews)
    maze = rasterise(maze_shapes(), MAZE_X, MAZE_Y)
    write_map('maze', maze, (MAZE_X[0], MAZE_Y[0]), args.out, args.previews)
    print('wrote arena ({} x {} cells) and maze ({} x {} cells) to {}'.format(
        arena.shape[1], arena.shape[0], maze.shape[1], maze.shape[0], args.out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
