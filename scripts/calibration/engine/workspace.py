"""Shared calibration RAM symbols and FP64 constants.

M contains 64-bit word addresses, C contains constant addresses. Their ordering
is the ABI used by all three programs and the RTL host adapters. Scratch aliases
q/rot/h intentionally overlap across mutually exclusive algorithm phases.
"""
import struct

NAMES = '''n nr ns maxiter maxtries desc current trial baseline plus minus column
 jac normal augmented scales delta gradient cost damping gmax threshold stepnorm
 newcost reduction tmp tmp2 h twice original perturb norm scale k t i j first last
 ja jb pa pb ptr count a b di dj ni nj attempt accepted outer converged status
 svc_dst svc_status svc_cost rowbase colbase endptr length
 camera poses viewrms views points fwidth fheight halfcols halfrows square bestcost
 in_converged usable metrics weak maxerr view row pair_a pair_b ix iy fxi fyi
 fp_points fp_total init_raw init_px init_py init_bx init_by init_seed
 check_reason check_warnings candidate_valid
 coverage_left coverage_right coverage_top coverage_bottom
 coverage_xlo coverage_xhi coverage_ylo coverage_yhi coverage_ok
 data_nr objective svc_objective lens_margin lens_weight barrier_weight'''.split()
assert len(NAMES) <= 128
M = {name: i for i, name in enumerate(NAMES)}
M.update({f'q{i}':128+i for i in range(34)})
M.update({f'rot{i}':256+i for i in range(32)})
M.update({f'h{i}':256+i for i in range(63)})
CONSTANTS = dict(zero=0x0000000000000000, one=0x3ff0000000000000,
 two=0x4000000000000000, initial_lambda=0x3f50624dd2f1a9fc,
 difference_step=0x3eb0c6f7a0b5ed8d, scale_floor=0x3d719799812dea11,
 cost_stop=0x3c9cd2b297d889bc, gradient_stop=0x3e45798ee2308c3a,
 lambda_down=0x3fd3333333333333, lambda_up=0x4024000000000000,
 reduction_stop=0x3da5fd7fe1796495, step_stop=0x3e112e0be826d695,
 relaxed_gradient=0x3ee4f8b588e368f1, infinity=0x7ff0000000000000,
 half=0x3fe0000000000000,three=0x4008000000000000,six=0x4018000000000000,
 twenty_four=0x4038000000000000,thirty_two=0x4040000000000000,
 focal_min=0x3fa999999999999a,focal_max=0x4034000000000000,
 normal_min=0x3f847ae147ae147b,normal_weak=0x3fc5c28f5c28f5c3,
 jacobian_min=0x3f1a36e2eb1c432d, sqrt_two=0x3ff6a09e667f3bcd,
 eigen_tiny=0x39b4484bfeebc2a0,eigen_tolerance=0x3d06849b86a12b9b,
 eigen_separation=0x3ddb7cdfd9d7bdbb)
CONSTANTS.update({name:struct.unpack('<Q',struct.pack('<d',value))[0] for name,value in
    dict(four=4.,five=5.,seven=7.,sqrt_three=3.**.5,
         lens_sigma=.05,barrier_sigma=.01,barrier_start=.1,
         barrier_epsilon=1e-5,barrier_slope=1e9).items()})
C = {name: 192+i for i, name in enumerate(CONSTANTS)}

