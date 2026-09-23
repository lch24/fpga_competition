#!/usr/bin/env python3
"""Bit-oriented fixed-point check for the Brown reverse-map calculation.

The model mirrors the proposed RTL formats:
  * fx/fy/cx/cy: signed Q16.16
  * k1/k2/p1/p2/k3: signed Q4.28
  * normalized coordinates and polynomial intermediates: signed Q*.30
  * source coordinates: signed Q16.16

It compares the fixed-point result against the floating-point formula used by
algorithom/closer2fpga/closer2fpga/algo/remap_table.cpp.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np


Q16 = 16
Q28 = 28
Q30 = 30


@dataclass(frozen=True)
class CameraParams:
    fx: float
    fy: float
    cx: float
    cy: float
    k1: float
    k2: float
    p1: float
    p2: float
    k3: float


DEFAULT_CAMERA = CameraParams(
    fx=1375.42407227,
    fy=1370.59216309,
    cx=661.89544678,
    cy=209.18479919,
    k1=-0.25408077,
    k2=0.04092772,
    p1=0.01019786,
    p2=-0.00072907,
    k3=0.0,
)


def quantize(value: float, fractional_bits: int) -> int:
    scaled = value * (1 << fractional_bits)
    # Match C lround for values that are not exact half-way cases.
    return int(np.floor(scaled + 0.5) if scaled >= 0 else np.ceil(scaled - 0.5))


def trunc_div_array(numerator: np.ndarray, denominator: int) -> np.ndarray:
    """Signed integer division with truncation toward zero, like Verilog '/'."""
    if denominator <= 0:
        raise ValueError("this model expects a positive focal length")
    result = np.abs(numerator) // denominator
    return np.where(numerator < 0, -result, result)


def fixed_row(xs: np.ndarray, y: int, camera: CameraParams) -> tuple[np.ndarray, np.ndarray]:
    fx = quantize(camera.fx, Q16)
    fy = quantize(camera.fy, Q16)
    cx = quantize(camera.cx, Q16)
    cy = quantize(camera.cy, Q16)
    k1 = quantize(camera.k1, Q28)
    k2 = quantize(camera.k2, Q28)
    p1 = quantize(camera.p1, Q28)
    p2 = quantize(camera.p2, Q28)
    k3 = quantize(camera.k3, Q28)

    dx = (xs << Q16) - cx
    dy = (y << Q16) - cy
    nx = trunc_div_array(dx << Q30, fx)
    ny = int((abs(dy << Q30) // fy) * (-1 if dy < 0 else 1))

    x2 = (nx * nx) >> Q30
    y2 = (ny * ny) >> Q30
    xy = (nx * ny) >> Q30
    r2 = x2 + y2
    r4 = (r2 * r2) >> Q30
    r6 = (r4 * r2) >> Q30

    radial = (
        (1 << Q30)
        + ((k1 * r2) >> Q28)
        + ((k2 * r4) >> Q28)
        + ((k3 * r6) >> Q28)
    )

    xd = (
        ((nx * radial) >> Q30)
        + 2 * ((p1 * xy) >> Q28)
        + ((p2 * (r2 + 2 * x2)) >> Q28)
    )
    yd = (
        ((ny * radial) >> Q30)
        + ((p1 * (r2 + 2 * y2)) >> Q28)
        + 2 * ((p2 * xy) >> Q28)
    )

    src_x = ((fx * xd) >> Q30) + cx
    src_y = ((fy * yd) >> Q30) + cy
    return src_x, src_y


def float_row(xs: np.ndarray, y: int, camera: CameraParams) -> tuple[np.ndarray, np.ndarray]:
    x = xs.astype(np.float64)
    nx = (x - camera.cx) / camera.fx
    ny = (float(y) - camera.cy) / camera.fy
    r2 = nx * nx + ny * ny
    radial = 1.0 + camera.k1 * r2 + camera.k2 * r2 * r2 + camera.k3 * r2 * r2 * r2
    xd = nx * radial + 2.0 * camera.p1 * nx * ny + camera.p2 * (r2 + 2.0 * nx * nx)
    yd = ny * radial + camera.p1 * (r2 + 2.0 * ny * ny) + 2.0 * camera.p2 * nx * ny
    return camera.fx * xd + camera.cx, camera.fy * yd + camera.cy


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    if args.width <= 0 or args.height <= 0:
        raise SystemExit("width and height must be positive")

    camera = DEFAULT_CAMERA
    xs = np.arange(args.width, dtype=np.int64)
    max_x_error = 0.0
    max_y_error = 0.0
    sum_x_error = 0.0
    sum_y_error = 0.0
    samples: dict[tuple[int, int], tuple[int, int, float, float]] = {}
    sample_points = {
        (0, 0),
        (args.width - 1, 0),
        (args.width // 2, args.height // 2),
        (0, args.height - 1),
        (args.width - 1, args.height - 1),
    }

    for y in range(args.height):
        fixed_x, fixed_y = fixed_row(xs, y, camera)
        float_x, float_y = float_row(xs, y, camera)
        decoded_x = fixed_x.astype(np.float64) / (1 << Q16)
        decoded_y = fixed_y.astype(np.float64) / (1 << Q16)
        error_x = np.abs(decoded_x - float_x)
        error_y = np.abs(decoded_y - float_y)
        max_x_error = max(max_x_error, float(error_x.max()))
        max_y_error = max(max_y_error, float(error_y.max()))
        sum_x_error += float(error_x.sum())
        sum_y_error += float(error_y.sum())

        for point_x, point_y in sample_points:
            if point_y == y:
                samples[(point_x, point_y)] = (
                    int(fixed_x[point_x]),
                    int(fixed_y[point_x]),
                    float(float_x[point_x]),
                    float(float_y[point_x]),
                )

    pixel_count = args.width * args.height
    print("Parameter encoding:")
    for name in ("fx", "fy", "cx", "cy"):
        print(f"  {name:>2} = {quantize(getattr(camera, name), Q16):11d}  (Q16.16)")
    for name in ("k1", "k2", "p1", "p2", "k3"):
        print(f"  {name:>2} = {quantize(getattr(camera, name), Q28):11d}  (Q4.28)")

    print("\nCoordinate error against the floating-point reference:")
    print(f"  max |dx|  = {max_x_error:.9f} pixel")
    print(f"  max |dy|  = {max_y_error:.9f} pixel")
    print(f"  mean |dx| = {sum_x_error / pixel_count:.9f} pixel")
    print(f"  mean |dy| = {sum_y_error / pixel_count:.9f} pixel")

    print("\nSample vectors (fixed outputs are signed Q16.16):")
    for point in sorted(samples, key=lambda item: (item[1], item[0])):
        fixed_x, fixed_y, ref_x, ref_y = samples[point]
        print(
            f"  dst={point}: src_q=({fixed_x}, {fixed_y}), "
            f"src_ref=({ref_x:.6f}, {ref_y:.6f})"
        )


if __name__ == "__main__":
    main()
