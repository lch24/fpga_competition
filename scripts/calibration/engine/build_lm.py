"""LM program builder: scalar control, finite differences, sparse J'J and solve.

Only residual evaluation is a HOST service. All iteration/retry decisions and
matrix operations execute on the same sequencer/workspace. Dimensions and
column descriptors are loaded by RTL, so ROM is independent of board/view size.
Generated RTL includes are checked in; listing goes to build for inspection.
"""
from assemble import ROOT, assemble
from compact_loads import compact_loads
from program import Program
from workspace import M, C, CONSTANTS
from build_lens import append_program as append_lens


class LMProgram(Program):
    def lens_service(self, dest, numeric):
        self.service(dest)
        self.check_service(numeric)
        self.emit('CALL lens_objective')


def program():
    p = LMProgram()
    initialize(p)
    jacobian(p)
    normal_equations(p)
    trial_step(p)
    accept_or_retry(p)
    finish(p)
    append_lens(p)
    append_solver(p)
    return compact_loads('\n'.join(p.lines))


def initialize(p):
    """Initialize counters, weights and the accepted-state objective."""
    e = p.emit
    e('# Generated LM assembly. All mutable scalars live in workspace RAM.',
      'MOVI A7,0','GUARD numeric_stop')
    for name in ('accepted','outer','attempt','converged','status'): p.seti(name,0)
    p.fcopy('cost','infinity')
    p.fcopy('damping','initial_lambda')
    p.unary('lens_weight','FSQRT','fp_total')
    p.fbin('barrier_weight','FMUL','lens_weight','barrier_sigma')
    p.fbin('lens_weight','FMUL','lens_weight','lens_sigma')
    p.copy('trial','current','ns')
    p.lens_service('baseline','numeric_stop')
    p.fcopy('cost','svc_cost')
    p.fcopy('objective','svc_objective')


def jacobian(p):
    """Emit the outer loop and normalized finite-difference columns."""
    e, L = p.emit, p.label
    L('outer_loop')
    e('GUARD numeric_stop')
    p.fb('objective',3,'cost_stop','converged')
    p.ib('outer',6,'maxiter','numeric_stop')
    p.addi('outer','outer',1)
    p.seti('k',0)
    L('column_loop')
    p.desc('k',('i','first','last','ja'))
    p.read('original','current','i')
    p.unary('tmp','FABS','original')
    p.fbin('tmp','FADD','one','tmp')
    p.fbin('h','FMUL','difference_step','tmp')
    p.fbin('perturb','FADD','original','h')
    p.copy('trial','current','ns')
    p.write('trial','perturb','i')
    p.lens_service('plus','numeric_stop')
    p.fbin('twice','FMUL','two','h')
    # Deliberately preserve old (p+h)-2h rounding, not p-h.
    p.fbin('perturb','FSUB','perturb','twice')
    p.write('trial','perturb','i')
    p.lens_service('minus','numeric_stop')
    p.il(0,'plus')
    p.il(1,'minus')
    p.il(2,'column')
    p.il(3,'nr')
    p.ld(3,'twice')
    p.ld(4,'zero')
    L('difference_loop')
    e('LD F0,A0,0','LD F1,A1,0','FSUB F0,F0,F1',
      'FDIV F0,F0,F3','ST F0,A2,0','FMUL F1,F0,F0','FADD F4,F4,F1',
      'ADDI A0,A0,1','ADDI A1,A1,1','ADDI A2,A2,1','DBNZ A3,difference_loop',
      'FSQRT F4,F4')
    p.ld(1,'scale_floor')
    e('FCMP F4,F1','BR.6 norm_ready','FMOV F4,F1')
    L('norm_ready')
    p.ld(0,'one')
    e('FDIV F3,F0,F4')
    p.st('scale',3)
    p.write('scales','scale','k')
    p.il(0,'column')
    p.il(1,'ja')
    p.il(2,'nr')
    p.il(3,'first')
    p.il(4,'last')
    e('MOVI A5,0')
    L('scale_loop')
    e('LD F0,A0,0','FMUL F0,F0,F3','ICMP A5,A3','BR.3 outside_column',
      'ICMP A5,A4','BR.6 outside_column','ST F0,A1,0','BR scale_next')
    L('outside_column')
    p.ld(1,'zero')
    e('FCMP F0,F1','BR.2 bad_config')
    L('scale_next')
    e('ADDI A0,A0,1','ADDI A1,A1,1','ADDI A5,A5,1','DBNZ A2,scale_loop')
    p.addi('k','k',1)
    p.ib('k',3,'n','column_loop')


def normal_equations(p):
    """Accumulate sparse J-transpose-J and the gradient in row order."""
    e, L = p.emit, p.label
    # Lower triangle, each dot accumulated in increasing residual order.
    p.seti('a',0)
    p.icopy('rowbase','normal')
    p.fcopy('gmax','zero')
    L('normal_row')
    p.seti('b',0)
    p.icopy('colbase','normal')
    p.desc('a',('i','di','ni','ja'))
    L('normal_column')
    p.desc('b',('j','dj','nj','jb'))
    p.icopy('first','di')
    p.ib('di',6,'dj','dot_first_ready')
    p.icopy('first','dj')
    L('dot_first_ready')
    p.icopy('last','ni')
    p.ib('ni',4,'nj','dot_last_ready')
    p.icopy('last','nj')
    L('dot_last_ready')
    p.ld(4,'zero')
    p.ib('first',6,'last','dot_done')
    p.il(0,'ja')
    p.il(1,'jb')
    p.il(2,'first')
    p.il(3,'last')
    e('IADD A0,A0,A2','IADD A1,A1,A2','ISUB A3,A3,A2')
    L('normal_dot')
    e('LD F0,A0,0','LD F1,A1,0','FMUL F0,F0,F1','FADD F4,F4,F0',
      'ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A3,normal_dot')
    L('dot_done')
    p.il(0,'rowbase')
    p.il(1,'b')
    e('IADD A0,A0,A1','ST F4,A0,0')
    p.il(0,'colbase')
    p.il(1,'a')
    e('IADD A0,A0,A1','ST F4,A0,0')
    p.ibin('colbase','IADD','colbase','n')
    p.addi('b','b',1)
    p.ib('b',4,'a','normal_column')
    # Gradient for the same row; view-local range is already available.
    p.il(0,'ja')
    p.il(1,'baseline')
    p.il(2,'di')
    p.il(3,'ni')
    e('IADD A0,A0,A2','IADD A1,A1,A2','ISUB A3,A3,A2')
    p.ld(4,'zero')
    L('gradient_dot')
    e('LD F0,A0,0','LD F1,A1,0','FMUL F0,F0,F1','FADD F4,F4,F0',
      'ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A3,gradient_dot')
    p.il(0,'gradient')
    p.il(1,'a')
    e('IADD A0,A0,A1','ST F4,A0,0','FABS F4,F4')
    p.ld(0,'gmax')
    e('FCMP F4,F0','BR.4 gradient_max_ready')
    p.st('gmax',4)
    L('gradient_max_ready')
    p.ibin('rowbase','IADD','rowbase','n')
    p.addi('a','a',1)
    p.ib('a',3,'n','normal_row')
    p.unary('threshold','FSQRT','objective')
    p.fbin('threshold','FADD','one','threshold')
    p.fbin('tmp','FMUL','gradient_stop','threshold')
    p.fb('gmax',3,'tmp','converged')
    p.seti('attempt',0)


def trial_step(p):
    """Build the damped system, solve it and form a trial state."""
    e, L = p.emit, p.label
    L('try_step')
    e('GUARD retry')
    p.il(0,'normal')
    p.il(1,'augmented')
    p.il(2,'gradient')
    p.il(3,'n')
    p.il(4,'n')
    e('MOVI A5,0')
    p.ld(3,'damping')
    L('damped_row')
    e('MOVI A6,0')
    L('damped_cell')
    e('LD F0,A0,0','ICMP A5,A6','BR.2 damped_store','FADD F0,F0,F3')
    L('damped_store')
    e('ST F0,A1,0','ADDI A0,A0,1','ADDI A1,A1,1','ADDI A6,A6,1',
      'ICMP A6,A4','BR.3 damped_cell','LD F0,A2,0','FNEG F0,F0','ST F0,A1,0',
      'ADDI A1,A1,1','ADDI A2,A2,1','ADDI A5,A5,1','DBNZ A3,damped_row')
    p.il(0,'augmented')
    p.il(1,'n')
    e('ADDI A2,A1,1')
    p.il(3,'delta')
    e('MOVI A7,256','CALL solve','MOVI A7,0','MOVI A1,0','ICMP A0,A1','BR.2 retry')
    p.copy('trial','current','ns')
    p.fcopy('stepnorm','zero')
    p.seti('k',0)
    L('update_loop')
    p.desc('k',('i','first','last','ja'))
    p.read('tmp','delta','k')
    p.read('tmp2','scales','k')
    p.fbin('tmp','FMUL','tmp','tmp2')
    p.read('original','current','i')
    p.fbin('perturb','FADD','original','tmp')
    p.write('trial','perturb','i')
    p.unary('tmp2','FABS','original')
    p.fbin('tmp2','FADD','one','tmp2')
    p.unary('tmp','FABS','tmp')
    p.fbin('tmp','FDIV','tmp','tmp2')
    p.fb('tmp',4,'stepnorm','stepnorm_ready')
    p.fcopy('stepnorm','tmp')
    L('stepnorm_ready')
    p.addi('k','k',1)
    p.ib('k',3,'n','update_loop')


def accept_or_retry(p):
    """Accept a feasible lower objective, or increase damping and retry."""
    e, L = p.emit, p.label
    # A numeric trial failure retries with greater damping. FD probes are not
    # rejected on shape: the smooth barrier needs their derivatives near bounds.
    e('GUARD retry')
    p.lens_service('minus','retry')
    p.fb('lens_margin',4,'jacobian_min','retry')
    p.fcopy('newcost','svc_objective')
    p.fb('newcost',6,'objective','retry')
    p.fbin('reduction','FSUB','objective','newcost')
    p.copy('current','trial','ns')
    p.fcopy('objective','newcost')
    p.fcopy('cost','svc_cost')
    p.addi('accepted','accepted',1)
    p.fbin('damping','FMUL','damping','lambda_down')
    p.copy('baseline','minus','nr')
    p.fb('damping',6,'scale_floor','lambda_floor_ready')
    p.fcopy('damping','scale_floor')
    L('lambda_floor_ready')
    p.fbin('tmp','FADD','one','objective')
    p.fbin('tmp','FMUL','reduction_stop','tmp')
    p.fb('stepnorm',3,'step_stop','converged')
    p.fb('reduction',3,'tmp','converged')
    e('BR outer_loop')
    L('retry')
    e('MOVI A7,0','GUARD numeric_stop')
    p.fbin('damping','FMUL','damping','lambda_up')
    p.addi('attempt','attempt',1)
    p.ib('attempt',3,'maxtries','try_step')
    p.fbin('tmp','FMUL','relaxed_gradient','threshold')
    p.fb('gmax',3,'tmp','converged')
    e('BR done')


def finish(p):
    """Publish termination status and recheck the accepted state."""
    e, L = p.emit, p.label
    L('numeric_stop')
    e('MOVI A7,0','BR done')
    L('bad_config')
    p.seti('status',1)
    e('BR done')
    L('converged')
    # Recheck the accepted state, not the last finite-difference/trial probe.
    p.copy('trial','current','ns')
    p.lens_service('baseline','numeric_stop')
    p.fb('lens_margin',4,'jacobian_min','done')
    p.seti('converged',1)
    L('done')
    e('END 0')


def append_solver(p):
    e = p.emit
    solve=(ROOT/'data/programs/calibration/solve.asm').read_text(encoding='utf-8')
    # The solver remains a callable subroutine. Singular pivots return a code;
    # GUARD handles floating exceptions and restores the caller's stack depth.
    solve=solve.replace('    RET','    MOVI A0,0\n    RET').replace('    END 210','    MOVI A0,1\n    RET')
    e(solve)

def build():
    source=program()
    words,labels=assemble(source,2048)
    out=ROOT/'build/calibration_engine'
    out.mkdir(parents=True,exist_ok=True)
    (out/'lm.asm').write_text(source,encoding='utf-8')
    (ROOT/'rtl/include/lm_program_init.vh').write_text(
        '// Generated by scripts/calibration/engine/build_lm.py.\n'+
        '\n'.join(f"program_memory[{i}]=32'h{x:08x};" for i,x in enumerate(words))+'\n',encoding='utf-8')
    (ROOT/'rtl/include/lm_program_defs.vh').write_text(
        '// Generated by build_lm.py; RAM word offsets, not byte addresses.\n'+
        f'`define LM_PROGRAM_WORDS {len(words)}\n'+
        '\n'.join(f'`define LM_{name.upper()} {index}' for name,index in M.items())+'\n',encoding='utf-8')
    lines=['// Generated initial scalar/constant RAM contents. All other words',
           '// are initialized before first read by the program or host adapter.']
    lines += [f"{address}: initial_data=64'h{CONSTANTS[name]:016x};" for name,address in C.items()]
    lines += ["280: initial_data=64'h39b4484bfeebc2a0;", # 1e-30
              "281: initial_data=64'h3d06849b86a12b9b;", # 1e-14
              "282: initial_data=0;", "283: initial_data=64'h7fefffffffffffff;"]
    (ROOT/'rtl/include/lm_constants.vh').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(f'LM ROM: {len(words)} words; columns/matrix/iteration loops execute as instructions')
    return words,labels

if __name__=='__main__':build()
