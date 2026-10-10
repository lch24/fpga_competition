"""LM program builder: scalar control, finite differences, sparse J'J and solve.

Only residual evaluation is a HOST service. All iteration/retry decisions and
matrix operations execute on the same sequencer/workspace. Dimensions and
column descriptors are loaded by RTL, so ROM is independent of board/view size.
Generated RTL includes are checked in; listing goes to build for inspection.
"""
from pathlib import Path
from assemble import ROOT, assemble

NAMES = '''n nr ns maxiter maxtries desc current trial baseline plus minus column
 jac normal augmented scales delta gradient cost damping gmax threshold stepnorm
 newcost reduction tmp tmp2 h twice original perturb norm scale k t i j first last
 ja jb pa pb ptr count a b di dj ni nj attempt accepted outer converged status
 svc_dst svc_status svc_cost rowbase colbase endptr length
 camera poses viewrms views points fwidth fheight halfcols halfrows square bestcost
 in_converged usable metrics weak maxerr view row pair_a pair_b ix iy fxi fyi
 fp_points fp_total init_raw init_px init_py init_bx init_by init_seed'''.split()
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
C = {name: 192+i for i, name in enumerate(CONSTANTS)}

class Program:
    def __init__(self): self.lines=[]; self.serial=0
    def emit(self,*lines): self.lines.extend(lines)
    def label(self,name): self.emit(name+':')
    def unique(self,name): self.serial+=1; return f'{name}_{self.serial}'
    def ld(self,reg,name): self.emit(f'LD F{reg},A7,{M.get(name,C.get(name))}')
    def st(self,name,reg=0): self.emit(f'ST F{reg},A7,{M[name]}')
    def il(self,reg,name): self.emit(f'LDI32 A{reg},A7,{M[name]}')
    def ist(self,name,reg=0): self.emit(f'STI32 A{reg},A7,{M[name]}')
    def seti(self,name,n): self.emit(f'MOVI A0,{n}'); self.ist(name)
    def icopy(self,d,s): self.il(0,s); self.ist(d)
    def fcopy(self,d,s): self.ld(0,s); self.st(d)
    def addi(self,d,s,n): self.il(0,s); self.emit(f'ADDI A0,A0,{n}'); self.ist(d)
    def ibin(self,d,op,a,b):
        self.il(0,a); self.il(1,b); self.emit(f'{op} A0,A0,A1'); self.ist(d)
    def fbin(self,d,op,a,b):
        self.ld(0,a); self.ld(1,b); self.emit(f'{op} F0,F0,F1'); self.st(d)
    def unary(self,d,op,a): self.ld(0,a); self.emit(f'{op} F0,F0'); self.st(d)
    def ib(self,a,cond,b,target):
        self.il(0,a)
        if isinstance(b,int): self.emit(f'MOVI A1,{b}')
        else: self.il(1,b)
        self.emit('ICMP A0,A1',f'BR.{cond} {target}')
    def fb(self,a,cond,b,target):
        self.ld(0,a); self.ld(1,b); self.emit('FCMP F0,F1',f'BR.{cond} {target}')
    def read(self,d,p,index=None):
        self.il(0,p)
        if index: self.il(1,index); self.emit('IADD A0,A0,A1')
        self.emit('LD F0,A0,0'); self.st(d)
    def write(self,p,s,index=None):
        self.il(0,p)
        if index: self.il(1,index); self.emit('IADD A0,A0,A1')
        self.ld(0,s); self.emit('ST F0,A0,0')
    def copy(self,d,s,count):
        loop=self.unique('copy'); self.il(0,d); self.il(1,s); self.il(2,count)
        self.label(loop); self.emit('LD F0,A1,0','ST F0,A0,0',
            'ADDI A0,A0,1','ADDI A1,A1,1',f'DBNZ A2,{loop}')
    def desc(self,col,which):
        # Descriptor = [state index, first owned residual, exclusive end, J base].
        self.il(0,col); self.emit('IADD A0,A0,A0','IADD A0,A0,A0')
        self.il(1,'desc'); self.emit('IADD A0,A0,A1')
        for offset,name in enumerate(which):
            self.emit(f'LDI32 A1,A0,{offset}'); self.ist(name,1)
    def service(self,dest):
        self.icopy('svc_dst',dest); self.emit('HOST 0')
    def check_service(self,numeric):
        self.ib('svc_status',1,4,numeric)
        ok=self.unique('service_ok'); self.ib('svc_status',1,0,ok)
        self.icopy('status','svc_status'); self.emit('BR done'); self.label(ok)

def program():
    p=Program(); e=p.emit; L=p.label
    e('# Generated LM assembly. All mutable scalars live in workspace RAM.',
      'MOVI A7,0','GUARD numeric_stop')
    for name in ('accepted','outer','attempt','converged','status'): p.seti(name,0)
    p.fcopy('cost','infinity'); p.fcopy('damping','initial_lambda')
    p.copy('trial','current','ns'); p.service('baseline'); p.check_service('numeric_stop')
    p.fcopy('cost','svc_cost')
    L('outer_loop'); e('GUARD numeric_stop')
    p.fb('cost',3,'cost_stop','converged'); p.ib('outer',6,'maxiter','numeric_stop')
    p.addi('outer','outer',1); p.seti('k',0)
    L('column_loop')
    p.desc('k',('i','first','last','ja'))
    p.read('original','current','i'); p.unary('tmp','FABS','original')
    p.fbin('tmp','FADD','one','tmp'); p.fbin('h','FMUL','difference_step','tmp')
    p.fbin('perturb','FADD','original','h')
    p.copy('trial','current','ns'); p.write('trial','perturb','i')
    p.service('plus'); p.check_service('numeric_stop')
    p.fbin('twice','FMUL','two','h')
    # Deliberately preserve old (p+h)-2h rounding, not p-h.
    p.fbin('perturb','FSUB','perturb','twice'); p.write('trial','perturb','i')
    p.service('minus'); p.check_service('numeric_stop')
    p.il(0,'plus'); p.il(1,'minus'); p.il(2,'column'); p.il(3,'nr'); p.ld(3,'twice'); p.ld(4,'zero')
    L('difference_loop'); e('LD F0,A0,0','LD F1,A1,0','FSUB F0,F0,F1',
      'FDIV F0,F0,F3','ST F0,A2,0','FMUL F1,F0,F0','FADD F4,F4,F1',
      'ADDI A0,A0,1','ADDI A1,A1,1','ADDI A2,A2,1','DBNZ A3,difference_loop',
      'FSQRT F4,F4')
    p.ld(1,'scale_floor'); e('FCMP F4,F1','BR.6 norm_ready','FMOV F4,F1')
    L('norm_ready'); p.ld(0,'one'); e('FDIV F3,F0,F4'); p.st('scale',3)
    p.write('scales','scale','k')
    p.il(0,'column'); p.il(1,'ja'); p.il(2,'nr'); p.il(3,'first'); p.il(4,'last'); e('MOVI A5,0')
    L('scale_loop'); e('LD F0,A0,0','FMUL F0,F0,F3','ICMP A5,A3','BR.3 outside_column',
      'ICMP A5,A4','BR.6 outside_column','ST F0,A1,0','BR scale_next')
    L('outside_column'); p.ld(1,'zero'); e('FCMP F0,F1','BR.2 bad_config')
    L('scale_next'); e('ADDI A0,A0,1','ADDI A1,A1,1','ADDI A5,A5,1','DBNZ A2,scale_loop')
    p.addi('k','k',1); p.ib('k',3,'n','column_loop')

    # Lower triangle, each dot accumulated in increasing residual order.
    p.seti('a',0); p.icopy('rowbase','normal'); p.fcopy('gmax','zero')
    L('normal_row'); p.seti('b',0); p.icopy('colbase','normal')
    p.desc('a',('i','di','ni','ja'))
    L('normal_column'); p.desc('b',('j','dj','nj','jb'))
    p.icopy('first','di'); p.ib('di',6,'dj','dot_first_ready'); p.icopy('first','dj')
    L('dot_first_ready'); p.icopy('last','ni'); p.ib('ni',4,'nj','dot_last_ready'); p.icopy('last','nj')
    L('dot_last_ready'); p.ld(4,'zero'); p.ib('first',6,'last','dot_done')
    p.il(0,'ja'); p.il(1,'jb'); p.il(2,'first'); p.il(3,'last')
    e('IADD A0,A0,A2','IADD A1,A1,A2','ISUB A3,A3,A2')
    L('normal_dot'); e('LD F0,A0,0','LD F1,A1,0','FMUL F0,F0,F1','FADD F4,F4,F0',
      'ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A3,normal_dot')
    L('dot_done'); p.il(0,'rowbase'); p.il(1,'b'); e('IADD A0,A0,A1','ST F4,A0,0')
    p.il(0,'colbase'); p.il(1,'a'); e('IADD A0,A0,A1','ST F4,A0,0')
    p.ibin('colbase','IADD','colbase','n'); p.addi('b','b',1); p.ib('b',4,'a','normal_column')
    # Gradient for the same row; view-local range is already available.
    p.il(0,'ja'); p.il(1,'baseline'); p.il(2,'di'); p.il(3,'ni')
    e('IADD A0,A0,A2','IADD A1,A1,A2','ISUB A3,A3,A2'); p.ld(4,'zero')
    L('gradient_dot'); e('LD F0,A0,0','LD F1,A1,0','FMUL F0,F0,F1','FADD F4,F4,F0',
      'ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A3,gradient_dot')
    p.il(0,'gradient'); p.il(1,'a'); e('IADD A0,A0,A1','ST F4,A0,0','FABS F4,F4')
    p.ld(0,'gmax'); e('FCMP F4,F0','BR.4 gradient_max_ready'); p.st('gmax',4)
    L('gradient_max_ready'); p.ibin('rowbase','IADD','rowbase','n'); p.addi('a','a',1)
    p.ib('a',3,'n','normal_row')
    p.unary('threshold','FSQRT','cost'); p.fbin('threshold','FADD','one','threshold')
    p.fbin('tmp','FMUL','gradient_stop','threshold'); p.fb('gmax',3,'tmp','converged')
    p.seti('attempt',0)
    L('try_step'); e('GUARD retry')
    p.il(0,'normal'); p.il(1,'augmented'); p.il(2,'gradient'); p.il(3,'n'); p.il(4,'n')
    e('MOVI A5,0'); p.ld(3,'damping')
    L('damped_row'); e('MOVI A6,0')
    L('damped_cell'); e('LD F0,A0,0','ICMP A5,A6','BR.2 damped_store','FADD F0,F0,F3')
    L('damped_store'); e('ST F0,A1,0','ADDI A0,A0,1','ADDI A1,A1,1','ADDI A6,A6,1',
      'ICMP A6,A4','BR.3 damped_cell','LD F0,A2,0','FNEG F0,F0','ST F0,A1,0',
      'ADDI A1,A1,1','ADDI A2,A2,1','ADDI A5,A5,1','DBNZ A3,damped_row')
    p.il(0,'augmented'); p.il(1,'n'); e('ADDI A2,A1,1'); p.il(3,'delta')
    e('MOVI A7,256','CALL solve','MOVI A7,0','MOVI A1,0','ICMP A0,A1','BR.2 retry')
    p.copy('trial','current','ns'); p.fcopy('stepnorm','zero'); p.seti('k',0)
    L('update_loop'); p.desc('k',('i','first','last','ja'))
    p.read('tmp','delta','k'); p.read('tmp2','scales','k'); p.fbin('tmp','FMUL','tmp','tmp2')
    p.read('original','current','i'); p.fbin('perturb','FADD','original','tmp')
    p.write('trial','perturb','i'); p.unary('tmp2','FABS','original')
    p.fbin('tmp2','FADD','one','tmp2'); p.unary('tmp','FABS','tmp')
    p.fbin('tmp','FDIV','tmp','tmp2'); p.fb('tmp',4,'stepnorm','stepnorm_ready'); p.fcopy('stepnorm','tmp')
    L('stepnorm_ready'); p.addi('k','k',1); p.ib('k',3,'n','update_loop')
    e('GUARD numeric_stop'); p.service('minus'); p.check_service('retry')
    p.fcopy('newcost','svc_cost'); p.fb('newcost',6,'cost','retry')
    p.fbin('reduction','FSUB','cost','newcost'); p.copy('current','trial','ns')
    p.fcopy('cost','newcost'); p.addi('accepted','accepted',1)
    p.fbin('damping','FMUL','damping','lambda_down'); p.copy('baseline','minus','nr')
    p.fb('damping',6,'scale_floor','lambda_floor_ready'); p.fcopy('damping','scale_floor')
    L('lambda_floor_ready'); p.fbin('tmp','FADD','one','cost'); p.fbin('tmp','FMUL','reduction_stop','tmp')
    p.fb('stepnorm',3,'step_stop','converged'); p.fb('reduction',3,'tmp','converged'); e('BR outer_loop')
    L('retry'); e('MOVI A7,0','GUARD numeric_stop')
    p.fbin('damping','FMUL','damping','lambda_up'); p.addi('attempt','attempt',1)
    p.ib('attempt',3,'maxtries','try_step')
    p.fbin('tmp','FMUL','relaxed_gradient','threshold'); p.fb('gmax',3,'tmp','converged'); e('BR done')
    L('numeric_stop'); e('MOVI A7,0','BR done')
    L('bad_config'); p.seti('status',1); e('BR done')
    L('converged'); p.seti('converged',1)
    L('done'); e('END 0')
    solve=(ROOT/'data/programs/calibration/solve.asm').read_text(encoding='utf-8')
    # The solver remains a callable subroutine. Singular pivots return a code;
    # GUARD handles floating exceptions and restores the caller's stack depth.
    solve=solve.replace('    RET','    MOVI A0,0\n    RET').replace('    END 210','    MOVI A0,1\n    RET')
    e(solve)
    return '\n'.join(p.lines)+'\n'

def build():
    source=program(); words,labels=assemble(source,1024)
    out=ROOT/'build/calibration_engine'; out.mkdir(parents=True,exist_ok=True)
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
