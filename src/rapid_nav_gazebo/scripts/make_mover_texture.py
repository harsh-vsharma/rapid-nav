"""
Generate the texture of the moving obstacles: a high-contrast pattern a camera can follow.

Run with: uv run --with numpy --with pillow python scripts/make_mover_texture.py
The output is models/rapid_nav_movers/materials/textures/mover.png (256 x 256, deterministic).
"""

import os

import numpy as np
from PIL import Image

SIZE = 256


def pattern():
    """Return a (SIZE, SIZE, 3) uint8 image: coarse checks, fine checks and random blobs."""
    rng = np.random.RandomState(7)
    v, u = np.mgrid[0:SIZE, 0:SIZE]
    coarse = ((u // 32 + v // 32) % 2).astype(np.float64)
    fine = ((u // 8 + v // 8) % 2).astype(np.float64)
    noise = rng.uniform(0.0, 1.0, (SIZE // 16, SIZE // 16))
    blobs = np.kron(noise, np.ones((16, 16)))
    gray = 0.45 * coarse + 0.25 * fine + 0.30 * blobs
    gray = (gray - gray.min()) / (gray.max() - gray.min())
    tint = np.array([1.0, 0.85, 0.7])
    image = 40.0 + 215.0 * gray[..., None] * tint
    return np.clip(image, 0, 255).astype(np.uint8)


def main():
    """Write the PNG next to the model."""
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, '..', 'models', 'rapid_nav_movers', 'materials', 'textures',
                       'mover.png')
    Image.fromarray(pattern(), 'RGB').save(out)
    print('wrote', os.path.normpath(out))


if __name__ == '__main__':
    main()
