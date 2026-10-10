# Bounded calibration math. Shared FP64 arithmetic only, no polynomial hardware.
# A7 -> constants: [0, 1, 2, 0.5, ln(2), 1/ln(2), 700, -700, ...].
# A7+16 .. A7+32: reciprocal factorials 1/0! .. 1/16!.
# A7+40 .. A7+64: 1/(2*i+1), i=0..24.
# Inputs outside the declared finite range stop with 0xd1 rather than producing
# silently inaccurate values. These routines are not yet the board backend.
# F0 input/output; F1..F7 and A4..A6 clobbered, A0..A3/A7 preserved.

math_exp:
    CLASS A4,F0
    MOVI A5,8
    ICMP A4,A5
    BR.6 math_range_error
    LD F1,A7,6
    FCMP F0,F1
    BR.5 math_range_error
    LD F1,A7,7
    FCMP F0,F1
    BR.3 math_range_error
    LD F1,A7,1
    LD F2,A7,2
    LD F3,A7,1
    LD F4,A7,4
    FMOV F6,F0
    LD F7,A7,0
exp_reduce_down:
    FCMP F0,F4
    BR.3 exp_reduce_up
    FADD F7,F7,F3
    FMUL F5,F7,F4
    FSUB F0,F6,F5
    FMUL F1,F1,F2
    BR exp_reduce_down
exp_reduce_up:
    LD F5,A7,0
    FCMP F0,F5
    BR.6 exp_polynomial
    FSUB F7,F7,F3
    FMUL F5,F7,F4
    FSUB F0,F6,F5
    LD F5,A7,3
    FMUL F1,F1,F5
    BR exp_reduce_up
exp_polynomial:
    LD F2,A7,32
    ADDI A4,A7,31
    MOVI A5,16
exp_horner:
    FMUL F2,F2,F0
    LD F3,A4,0
    FADD F2,F2,F3
    ADDI A4,A4,-1
    DBNZ A5,exp_horner
    FMUL F0,F2,F1
    RET

math_log:
    CLASS A4,F0
    MOVI A5,8
    ICMP A4,A5
    BR.6 math_range_error
    LD F1,A7,0
    FCMP F0,F1
    BR.4 math_range_error
    LD F2,A7,1
    LD F3,A7,2
    LD F4,A7,3
    LD F5,A7,1
log_reduce_down:
    FCMP F0,F3
    BR.3 log_reduce_up
    FMUL F0,F0,F4
    FADD F1,F1,F5
    BR log_reduce_down
log_reduce_up:
    FCMP F0,F2
    BR.6 log_series
    FMUL F0,F0,F3
    FSUB F1,F1,F5
    BR log_reduce_up
log_series:
    LD F5,A7,4
    FMUL F1,F1,F5
    FSUB F3,F0,F2
    FADD F4,F0,F2
    FDIV F3,F3,F4
    FMUL F4,F3,F3
    LD F2,A7,64
    ADDI A4,A7,63
    MOVI A5,24
log_horner:
    FMUL F2,F2,F4
    LD F5,A4,0
    FADD F2,F2,F5
    ADDI A4,A4,-1
    DBNZ A5,log_horner
    FMUL F2,F2,F3
    FADD F2,F2,F2
    FADD F0,F1,F2
    RET

math_range_error:
    END 209
