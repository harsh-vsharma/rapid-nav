"""Checks of the generated maps: deterministic output, the arena against its geometry, and the maze's doors."""

import filecmp
import importlib.util
import os
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.normpath(os.path.join(HERE, '..'))
GENERATOR = os.path.join(PACKAGE, 'scripts', 'make_maps.py')
MAPS = os.path.join(PACKAGE, 'maps')
RESOLUTION = 0.05


def load_generator():
    """Import scripts/make_maps.py from the source tree."""
    spec = importlib.util.spec_from_file_location('make_maps', GENERATOR)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_pgm(path):
    """Return the occupied cells of a PGM as a boolean array, with row 0 at the south edge."""
    with open(path, 'rb') as handle:
        assert handle.readline().strip() == b'P5'
        width, height = [int(value) for value in handle.readline().split()]
        assert handle.readline().strip() == b'255'
        pixels = np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)
    occupied = pixels[::-1] == 0
    return occupied


def read_origin(yaml_path):
    """Return the origin (x, y) of a map YAML."""
    with open(yaml_path, encoding='utf-8') as handle:
        for line in handle:
            if line.startswith('origin:'):
                values = line.split('[')[1].split(']')[0].split(',')
                return float(values[0]), float(values[1])
    raise AssertionError('no origin in ' + yaml_path)


def cell_of(point, origin):
    """Return (column, row) of the cell containing a world point."""
    column = int(np.floor((point[0] - origin[0]) / RESOLUTION))
    row = int(np.floor((point[1] - origin[1]) / RESOLUTION))
    return column, row


def dilate(mask, radius_cells):
    """Return the mask grown by a disk of the given radius in cells (a numpy helper for the test)."""
    radius = int(radius_cells)
    padded = np.pad(mask, radius, mode='constant', constant_values=False)
    grown = np.zeros_like(mask)
    rows, columns = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                grown |= padded[radius + dy:radius + dy + rows, radius + dx:radius + dx + columns]
    return grown


def test_generator_output_is_deterministic():
    """Running the generator again writes byte-identical PGMs and YAMLs."""
    generator = load_generator()
    with tempfile.TemporaryDirectory() as folder:
        assert generator.main(['--out', folder]) == 0
        for name in ('arena.pgm', 'arena.yaml', 'maze.pgm', 'maze.yaml'):
            assert filecmp.cmp(os.path.join(folder, name), os.path.join(MAPS, name), shallow=False), name


def test_arena_has_the_size_and_origin_of_the_plan():
    """The arena map covers x in [-5.5, 5.5] and y in [-4.5, 4.5] at 0.05 m: 220 by 180 cells."""
    occupied = read_pgm(os.path.join(MAPS, 'arena.pgm'))
    assert occupied.shape == (180, 220)
    assert read_origin(os.path.join(MAPS, 'arena.yaml')) == (-5.5, -4.5)


def test_arena_occupies_crates_pillars_and_walls():
    """Crate and pillar centres and the four wall mid-points are occupied; the origin and (1.2, 0) are free."""
    origin = read_origin(os.path.join(MAPS, 'arena.yaml'))
    occupied = read_pgm(os.path.join(MAPS, 'arena.pgm'))

    def is_occupied(point):
        column, row = cell_of(point, origin)
        return bool(occupied[row, column])

    for crate in [(2.0, 1.5), (-2.0, -1.5), (3.0, -2.5), (-3.0, 2.5)]:
        assert is_occupied(crate), crate
    for pillar in [(0.0, 2.5), (1.0, -2.5)]:
        assert is_occupied(pillar), pillar
    for wall in [(0.0, 4.05), (0.0, -4.05), (5.05, 0.0), (-5.05, 0.0)]:
        assert is_occupied(wall), wall
    assert not is_occupied((0.0, 0.0))
    assert not is_occupied((1.2, 0.0))


def test_maze_size_and_origin():
    """The maze covers x in [-5, 5] and y in [-4, 4] at 0.05 m: 200 by 160 cells."""
    occupied = read_pgm(os.path.join(MAPS, 'maze.pgm'))
    assert occupied.shape == (160, 200)
    assert read_origin(os.path.join(MAPS, 'maze.yaml')) == (-5.0, -4.0)


def test_maze_doors_after_the_robot_radius():
    """Dilated by 0.25 m, the 0.45 m door is closed and the 0.70 m door keeps a free centre cell."""
    origin = read_origin(os.path.join(MAPS, 'maze.yaml'))
    occupied = read_pgm(os.path.join(MAPS, 'maze.pgm'))
    grown = dilate(occupied, round(0.25 / RESOLUTION))
    narrow = [(0.0, y) for y in np.arange(-1.775, -1.36, 0.05)]
    for point in narrow:
        column, row = cell_of(point, origin)
        assert grown[row, column], point
    column, row = cell_of((0.0, 1.15), origin)
    assert not grown[row, column]
    column, row = cell_of((0.0, 1.15), origin)
    assert not occupied[row, column]


def test_yaml_describes_a_trinary_map():
    """The map YAMLs name the PGM, the trinary mode, the resolution and the thresholds."""
    for name in ('arena', 'maze'):
        with open(os.path.join(MAPS, name + '.yaml'), encoding='utf-8') as handle:
            text = handle.read()
        assert 'image: {}.pgm'.format(name) in text
        assert 'mode: trinary' in text
        assert 'resolution: 0.05' in text
        assert 'negate: 0' in text
        assert 'occupied_thresh: 0.65' in text
        assert 'free_thresh: 0.25' in text
