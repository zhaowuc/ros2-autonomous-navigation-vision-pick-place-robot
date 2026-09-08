#!/usr/bin/env python3
"""Generate the binary occupancy image matching corridor_demo.world."""

from pathlib import Path


RESOLUTION = 0.05
ORIGIN = -5.0
SIZE = 200
WALLS = [
    (-4.05, 0.0, 0.10, 8.20),
    (4.05, 0.0, 0.10, 8.20),
    (0.0, -4.05, 8.20, 0.10),
    (0.0, 4.05, 8.20, 0.10),
    (-1.50, 0.0, 5.00, 0.12),
    (1.0, 1.0, 0.12, 2.00),
]


def occupied(x, y):
    return any(
        abs(x - cx) <= width / 2.0 and abs(y - cy) <= height / 2.0
        for cx, cy, width, height in WALLS
    )


def main():
    target = Path(__file__).resolve().parents[1] / 'maps' / 'corridor_demo.pgm'
    pixels = bytearray()
    for row in range(SIZE):
        y = ORIGIN + (SIZE - row - 0.5) * RESOLUTION
        for column in range(SIZE):
            x = ORIGIN + (column + 0.5) * RESOLUTION
            pixels.append(0 if occupied(x, y) else 254)
    target.write_bytes(f'P5\n{SIZE} {SIZE}\n255\n'.encode('ascii') + pixels)
    print(target)


if __name__ == '__main__':
    main()
