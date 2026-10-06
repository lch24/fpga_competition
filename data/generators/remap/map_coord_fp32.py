#!/usr/bin/env python3
"""FP32 numeric model for rtl/remap/map_coord_core.v.

Every helper rounds its result to IEEE-754 binary32, matching one result from
the abstract FP32 service. The operation order intentionally follows the RTL;
this is the reference for RTL vector generation, while the C++ implementation
remains the algorithm-level reference.
"""

from __future__ import annotations

import argparse
import struct
from dataclasses import dataclass

import numpy as np


F32 = np.float32


@dataclass(frozen=True)
class Camera:
    fx: float = 1375.42407227
    fy: float = 1370.59216309
    cx: float = 661.89544678
    cy: float = 209.18479919
    k1: float = -0.25408077
    k2: float = 0.04092772
    k3: float = 0.0
    p1: float = 0.01019786
    p2: float = -0.00072907


def add(a: F32, b: F32) -> F32:
    return F32(F32(a) + F32(b))


def sub(a: F32, b: F32) -> F32:
    return F32(F32(a) - F32(b))


def mul(a: F32, b: F32) -> F32:
    return F32(F32(a) * F32(b))


def div(a: F32, b: F32) -> F32:
    return F32(F32(a) / F32(b))


def fp32_bits(value: F32) -> int:
    return struct.unpack("<I", struct.pack("<f", float(F32(value))))[0]


def map_coordinate(dst_x: int, dst_y: int, camera: Camera = Camera()) -> tuple[F32, F32]:
    fx, fy, cx, cy = map(F32, (camera.fx, camera.fy, camera.cx, camera.cy))
    k1, k2, k3, p1, p2 = map(F32, (camera.k1, camera.k2, camera.k3, camera.p1, camera.p2))

    x_fp = F32(dst_x)
    y_fp = F32(dst_y)
    dx = sub(x_fp, cx)
    nx = div(dx, fx)
    dy = sub(y_fp, cy)
    ny = div(dy, fy)

    x2 = mul(nx, nx)
    y2 = mul(ny, ny)
    r2 = add(x2, y2)
    r4 = mul(r2, r2)
    r6 = mul(r4, r2)

    k1r2 = mul(k1, r2)
    k2r4 = mul(k2, r4)
    k3r6 = mul(k3, r6)
    radial_a = add(F32(1.0), k1r2)
    radial_b = add(radial_a, k2r4)
    radial = add(radial_b, k3r6)

    x_radial = mul(nx, radial)
    y_radial = mul(ny, radial)
    xy = mul(nx, ny)

    p1xy = mul(p1, xy)
    tx1 = add(p1xy, p1xy)
    two_x2 = add(x2, x2)
    r2_2x2 = add(r2, two_x2)
    tx2 = mul(p2, r2_2x2)
    xd_a = add(x_radial, tx1)
    xd = add(xd_a, tx2)

    two_y2 = add(y2, y2)
    r2_2y2 = add(r2, two_y2)
    ty1 = mul(p1, r2_2y2)
    p2xy = mul(p2, xy)
    ty2 = add(p2xy, p2xy)
    yd_a = add(y_radial, ty1)
    yd = add(yd_a, ty2)

    src_x = add(mul(fx, xd), cx)
    src_y = add(mul(fy, yd), cy)
    return src_x, src_y


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0:
        raise SystemExit("width and height must be positive")

    points = [
        (0, 0),
        (args.width - 1, 0),
        (args.width // 2, args.height // 2),
        (0, args.height - 1),
        (args.width - 1, args.height - 1),
    ]
    print("dst_x,dst_y,src_x_bits,src_y_bits,src_x,src_y")
    for x, y in points:
        sx, sy = map_coordinate(x, y)
        print(f"{x},{y},{fp32_bits(sx):08x},{fp32_bits(sy):08x},{sx:.9f},{sy:.9f}")


if __name__ == "__main__":
    main()
