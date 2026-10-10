# Brown projection after world-to-camera transformation, k3 fixed to zero.
# A0 -> [fx, fy, cx, cy, k1, k2, p1, p2], each one FP64 word.
# Input F0=Xc, F1=Yc, F2=Zc (caller checks finite and positive depth).
# Output F0=u, F1=v. Clobbers F2..F7, flags; preserves A registers.
# Same shared scalar ALU for every operation; no dedicated Brown datapath.
project:
    FDIV F0,F0,F2
    FDIV F1,F1,F2
    FMUL F2,F0,F0
    FMUL F3,F1,F1
    FADD F4,F2,F3
    FMUL F5,F0,F1
    LD F6,A0,5
    FMUL F6,F6,F4
    LD F7,A0,4
    FADD F6,F6,F7
    FMUL F6,F6,F4
    # Radial contribution: x*(1+s), y*(1+s), avoiding a constant load.
    FMUL F7,F0,F6
    FADD F0,F0,F7
    FMUL F7,F1,F6
    FADD F1,F1,F7
    # 2*x*y used by both tangential terms.
    FADD F5,F5,F5
    LD F6,A0,6
    FMUL F7,F6,F5
    FADD F0,F0,F7
    FADD F3,F3,F3
    FADD F3,F3,F4
    FMUL F3,F3,F6
    FADD F1,F1,F3
    LD F6,A0,7
    FMUL F7,F6,F5
    FADD F1,F1,F7
    FADD F2,F2,F2
    FADD F2,F2,F4
    FMUL F2,F2,F6
    FADD F0,F0,F2
    LD F2,A0,0
    FMUL F0,F0,F2
    LD F2,A0,2
    FADD F0,F0,F2
    LD F2,A0,1
    FMUL F1,F1,F2
    LD F2,A0,3
    FADD F1,F1,F2
    RET
