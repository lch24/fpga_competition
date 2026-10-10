# Partial-pivot Gaussian elimination, followed by back substitution.
# A0 matrix base, A1 dimension n, A2 row stride n+1, A3 solution base,
# A7 scratch base. Matrix is row-major [A | b], overwritten in place.
# Scratch 0..3: saved integer arguments. Scratch 24=tiny(1e-30),
# 25=relative pivot tolerance(1e-14), 26=+0, 27=max finite FP64.
# Matches old solver's pivot ordering, row-scale definition and separate
# multiply/subtract rounding. n is runtime, not tied to number of views.
# Caller owns disjoint writable matrix/solution/scratch regions. Bounds are
# still enforced by the workspace. Success RET; invalid/singular END 0xd2.
# All registers except A7 are clobbered. No hardware matrix solver is used.
solve:
    STI32 A0,A7,0
    STI32 A1,A7,1
    STI32 A2,A7,2
    STI32 A3,A7,3
    MOVI A4,0
    ICMP A1,A4
    BR.1 solve_error
    ADDI A4,A1,1
    ICMP A2,A4
    BR.2 solve_error
    # Validate every input before elimination writes any element.
    ADDI A4,A0,0
    ADDI A5,A1,0
    MOVI A1,8
solve_validate_row:
    ADDI A6,A2,0
solve_validate_cell:
    LD F0,A4,0
    CLASS A0,F0
    ICMP A0,A1
    BR.6 solve_error
    ADDI A4,A4,1
    DBNZ A6,solve_validate_cell
    DBNZ A5,solve_validate_row
    LDI32 A4,A7,0
    LDI32 A5,A7,1
    LD F4,A7,27
solve_column:
    LDI32 A1,A7,2
    ADDI A0,A4,0
    ADDI A2,A5,0
    ADDI A3,A4,0
    LD F0,A7,26
solve_pivot_scan:
    LD F1,A0,0
    FABS F1,F1
    FCMP F1,F0
    BR.4 solve_pivot_next
    FMOV F0,F1
    ADDI A3,A0,0
solve_pivot_next:
    IADD A0,A0,A1
    DBNZ A2,solve_pivot_scan
    LD F2,A7,24
    FCMP F0,F2
    BR.3 solve_error
    ADDI A0,A3,0
    ADDI A2,A5,0
    LD F2,A7,26
solve_scale_scan:
    LD F1,A0,0
    FABS F1,F1
    FCMP F1,F2
    BR.4 solve_scale_next
    FMOV F2,F1
solve_scale_next:
    ADDI A0,A0,1
    DBNZ A2,solve_scale_scan
    LD F3,A7,24
    FCMP F2,F3
    BR.3 solve_error
    LD F3,A7,25
    FMUL F3,F2,F3
    FCMP F0,F3
    BR.3 solve_error
    ICMP A3,A4
    BR.1 solve_pivot_ready
    ADDI A0,A4,0
    ADDI A1,A3,0
    ADDI A2,A5,1
solve_swap:
    LD F0,A0,0
    LD F1,A1,0
    ST F1,A0,0
    ST F0,A1,0
    ADDI A0,A0,1
    ADDI A1,A1,1
    DBNZ A2,solve_swap
solve_pivot_ready:
    LD F0,A4,0
    MOVI A3,1
    ICMP A5,A3
    BR.1 solve_back_start
    LDI32 A3,A7,2
    IADD A0,A4,A3
    ADDI A2,A5,-1
solve_row:
    LD F1,A0,0
    FABS F2,F1
    LD F3,A7,24
    FCMP F2,F3
    BR.3 solve_skip_row
    FDIV F1,F1,F0
    LD F2,A7,26
    ST F2,A0,0
    ADDI A0,A0,1
    ADDI A1,A4,1
    ADDI A6,A5,0
solve_eliminate:
    LD F2,A1,0
    FMUL F3,F1,F2
    LD F2,A0,0
    FSUB F2,F2,F3
    FABS F3,F2
    FCMP F3,F4
    BR.7 solve_error
    BR.5 solve_error
    ST F2,A0,0
    ADDI A0,A0,1
    ADDI A1,A1,1
    DBNZ A6,solve_eliminate
    # Inner loop advanced n-k+1 words; restore the next row's column.
    LDI32 A3,A7,2
    ISUB A3,A3,A5
    ADDI A3,A3,-1
    IADD A0,A0,A3
    BR solve_next_row
solve_skip_row:
    LDI32 A3,A7,2
    IADD A0,A0,A3
solve_next_row:
    DBNZ A2,solve_row
    LDI32 A3,A7,2
    IADD A4,A4,A3
    ADDI A4,A4,1
    ADDI A5,A5,-1
    BR solve_column
solve_back_start:
    LDI32 A6,A7,1
    MOVI A5,1
solve_back_row:
    IADD A0,A4,A5
    LD F0,A0,0
    LDI32 A1,A7,3
    LDI32 A3,A7,1
    IADD A1,A1,A3
    ISUB A1,A1,A5
    ADDI A1,A1,1
    ADDI A2,A5,-1
    MOVI A3,0
    ICMP A2,A3
    BR.1 solve_back_divide
    ADDI A0,A4,1
solve_back_accumulate:
    LD F1,A0,0
    LD F2,A1,0
    FMUL F1,F1,F2
    FSUB F0,F0,F1
    ADDI A0,A0,1
    ADDI A1,A1,1
    DBNZ A2,solve_back_accumulate
solve_back_divide:
    LD F1,A4,0
    FDIV F0,F0,F1
    FABS F3,F0
    FCMP F3,F4
    BR.7 solve_error
    BR.5 solve_error
    LDI32 A1,A7,3
    LDI32 A3,A7,1
    IADD A1,A1,A3
    ISUB A1,A1,A5
    ST F0,A1,0
    LDI32 A3,A7,2
    ISUB A4,A4,A3
    ADDI A4,A4,-1
    ADDI A5,A5,1
    DBNZ A6,solve_back_row
    RET
solve_error:
    END 210
