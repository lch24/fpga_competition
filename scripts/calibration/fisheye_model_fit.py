# How many radial terms does the existing distortion model need for a fisheye lens?
#
# The RTL map core (map_coord_core.v) and the calibration solver (project.asm)
# both model distortion as a radial polynomial applied to the pinhole-normalised
# coordinate, so a corrected output pixel (u,v) samples the raw image at
#     normalised (nx,ny) = ((u-cx)/f_out, (v-cy)/f_out),  t = nx^2+ny^2
#     factor F(t) = 1 + k1*t + k2*t^2 + k3*t^3 + ...
#     raw pixel = (f_raw*F(t)*nx + cx, f_raw*F(t)*ny + cy)
# (the solver currently stops after k2; the map core already computes k3).
#
# This script asks how well that form fits an equidistant (rho = f*theta)
# fisheye whose corrected pinhole view spans the whole field of view, and
# reports the worst pixel error versus the number of radial terms, so the
# number of terms can be chosen from field of view instead of guessed.
#
#   python scripts/calibration/fisheye_model_fit.py
import numpy as np

R_CORNER = np.hypot(640.0, 360.0)   # 1280x720 half diagonal, pixels


def study(theta_max_deg, terms):
    th_max = np.deg2rad(theta_max_deg)
    f_raw = R_CORNER / th_max                  # raw radius = f_raw * theta
    f_out = R_CORNER / np.tan(th_max)          # corrected corner shows theta_max
    rad_max = R_CORNER / f_out                 # = tan(theta_max)
    rad = np.linspace(0.0, rad_max, 40001)
    t = rad**2
    # truth: raw_radius / (f_raw * rad_p) = atan(rad_p) / rad_p
    ratio = np.where(rad > 0, np.arctan(rad) / np.where(rad > 0, rad, 1.0), 1.0)
    basis = np.stack([t**k for k in range(1, terms + 1)], axis=1)
    coef, *_ = np.linalg.lstsq(basis, ratio - 1.0, rcond=None)
    fit = 1.0 + basis @ coef
    err_raw = np.abs(f_raw * rad * (fit - ratio))
    # Local warp scale converts a raw-pixel error into output-pixel displacement.
    scale = np.gradient(f_raw * np.arctan(rad), rad)
    err_out = err_raw / np.where(np.abs(scale) > 1e-6, np.abs(scale), 1e-6)
    return dict(f_raw=f_raw, f_out=f_out, coef=coef, ratio_end=fit[-1],
                true_end=ratio[-1], max_err_raw=err_raw.max(),
                max_err_out=err_out.max())


if __name__ == "__main__":
    print("equidistant fisheye, corrected pinhole view spans the whole field of view")
    print("error = worst deviation in RAW image pixels (output pixels)")
    print(" FOV  | f_raw  f_out |  1+k1t+k2t^2   |    +k3t^3      |    +k4t^4")
    for deg in (40, 45, 50, 55, 60, 65, 70, 75, 80):
        cells = []
        for terms in (2, 3, 4):
            a = study(deg, terms)
            cells.append(f"{a['max_err_raw']:7.2f} ({a['max_err_out']:5.2f})")
        f = study(deg, 2)
        print(f"{2*deg:4d}  | {f['f_raw']:6.1f} {f['f_out']:6.1f} | " + " | ".join(cells))

    print()
    for terms in (2, 3, 4):
        label = "F = 1 + " + "+".join(f"k{i}t^{i}" for i in range(1, terms + 1))
        print(label + ", coefficients per FOV:")
        for deg in (40, 50, 60, 70, 80):
            a = study(deg, terms)
            print(f"  {2*deg:3d} deg  "
                  + "  ".join(f"k{i+1}={v: .6f}" for i, v in enumerate(a['coef']))
                  + f"   F(end)={a['ratio_end']:.5f} true={a['true_end']:.5f}")
