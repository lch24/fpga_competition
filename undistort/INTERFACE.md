# Undistortion RTL interface (draft 0.1)

## Ownership

- Calibration team: detect/calibrate and produce one complete camera-parameter set.
- Undistortion team: store the result, generate reverse-map coordinates, access DDR,
  perform bilinear interpolation, and send corrected pixels to the display path.

## Calibration result transaction

The nine parameters form one transaction. They do not have independent handshakes.
The transfer occurs on a rising clock edge when both `param_valid` and
`param_ready` are high.

The calibration producer must keep every parameter and all metadata stable while
`param_valid=1` and `param_ready=0`.

| Field | Format | Meaning |
| --- | --- | --- |
| `fx fy cx cy` | signed Q16.16 | Intrinsic parameters in pixels |
| `k1 k2 p1 p2 k3` | signed Q4.28 | Brown distortion coefficients |
| `calib_width/height` | unsigned integer | Resolution used for calibration |
| `calib_id` | unsigned integer | Monotonic result identifier |
| `rms_error` | unsigned Q16.16 | Reprojection RMS error in pixels |

The producer asserts `param_valid` only for a successful and usable calibration.
Failure status is a separate control/status path and must not overwrite the last
good parameter set.

## Parameter store behavior

`rtl/camera_param_store.v` provides a one-entry shadow bank and an active bank.
A newly received result may wait in the shadow bank while the remap generator is
busy. The entire result is switched to the active bank on one clock edge after
`map_busy` goes low. `active_update` pulses for one cycle on that edge.

Version 0.1 assumes the producer and the parameter store use the same clock. If
they use different clocks, place an asynchronous mailbox/FIFO in front of the
store; a bare ready/valid connection is not a clock-domain crossing solution.

## Next interface boundary

The remap generator will consume the active bank and produce, in raster order,
signed Q16.16 source coordinates for each destination pixel. DDR base addresses,
strides, RGB565 packing, border policy, and bilinear interpolation remain outside
the calibration RTL.
