# Undistortion RTL interface

The authoritative team contract is `main/README.md`. This branch implements the
work after a successful calibration parameter packet has been received.

## Input boundary

One accepted packet contains:

```text
calib_id[31:0], width[15:0], height[15:0], camera_valid,
fx, fy, cx, cy, k1, k2, k3, p1, p2
```

The nine numeric fields are raw IEEE-754 FP32 bit patterns. RMS is an optional
FP64 diagnostic and is not used by map generation. The top-level controller must
pair the packet with the successful calibration response before starting a map
job. Failed or partial calibration results never replace the last good map.

## Internal map-build boundary

`map_build_ctrl` locks the parameters for one complete job. It configures
`map_coord_core`, emits destination coordinates in raster order, and passes the
resulting FP32 `(src_x,src_y)` stream to `map_table_writer`.

The table writer stores two planes:

```text
map_x_addr = map_x_base + y*map_stride_bytes + 4*x
map_y_addr = map_y_base + y*map_stride_bytes + 4*x
```

It returns completion only after both planes' DDR write responses have arrived.
The published map descriptor binds the table to `calib_id`, width, height, and
the pixel-center coordinate convention.

## Protocol rule

Every command and stream uses ready/valid. When `valid=1` and `ready=0`, valid
and the complete payload remain stable. Counters advance only on
`valid && ready`. All current modules assume the same `core_clk`; crossing from
the camera clock requires a separate verified CDC FIFO.
