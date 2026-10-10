"""Final report/check program, preserving the former FP64 operation order.

Includes Rodrigues rotation, per-view error statistics, geometry checks and
the 33x25 mapping Jacobian scan. No matrix/rotation checker hardware is needed.
Only recomputing residuals invokes the external HOST service.
"""
from build_lm import Program, M
from assemble import ROOT,assemble

def rotation(p):
    e=p.emit;L=p.label
    L('rotation_forward')
    p.fbin('rot3','FMUL','rot0','rot0');p.fbin('rot4','FMUL','rot1','rot1')
    p.fbin('rot3','FADD','rot3','rot4');p.fbin('rot4','FMUL','rot2','rot2')
    p.fbin('rot3','FADD','rot3','rot4');p.fb('rot3',6,'scale_floor','rotation_large')
    p.fbin('rot5','FDIV','rot3','six');p.fbin('rot5','FSUB','one','rot5')
    p.fbin('rot6','FDIV','rot3','twenty_four');p.fbin('rot6','FSUB','half','rot6')
    e('BR rotation_matrix')
    L('rotation_large');p.unary('rot4','FSQRT','rot3');p.unary('rot5','FSIN','rot4')
    p.fbin('rot5','FDIV','rot5','rot4');p.unary('rot6','FCOS','rot4')
    p.fbin('rot6','FSUB','one','rot6');p.fbin('rot6','FDIV','rot6','rot3')
    L('rotation_matrix')
    for i,name in enumerate(('zero','rot2','rot1','rot2','zero','rot0','rot1','rot0','zero')):
        if i in (1,5,6):p.unary(f'rot{20+i}','FNEG',name)
        else:p.fcopy(f'rot{20+i}',name)
    for row in range(3):
        for col in range(3):
            p.fcopy('rot7','zero')
            for k in range(3):
                p.fbin('rot8','FMUL',f'rot{20+3*row+k}',f'rot{20+3*k+col}')
                p.fbin('rot7','FADD','rot7','rot8')
            p.fbin('rot8','FMUL','rot5',f'rot{20+3*row+col}')
            p.fbin('rot8','FADD','one' if row==col else 'zero','rot8')
            p.fbin('rot7','FMUL','rot6','rot7')
            p.fbin(f'rot{9+3*row+col}','FADD','rot8','rot7')
    e('RET')

def program():
    p=Program();e=p.emit;L=p.label
    e('MOVI A7,0','GUARD check_failed')
    for name in ('usable','metrics','weak','status'):p.seti(name,0)
    # Configuration errors take precedence over invalid state.
    for name in ('square','bestcost'):
        p.ld(0,name);e('CLASS A0,F0','MOVI A1,8','ICMP A0,A1','BR.6 bad_configuration')
    p.fb('square',4,'zero','bad_configuration');p.fb('bestcost',3,'zero','bad_configuration')
    p.il(0,'current');p.il(2,'ns');e('MOVI A3,8')
    L('state_finite');e('LD F0,A0,0','CLASS A1,F0','ICMP A1,A3','BR.6 check_failed',
      'ADDI A0,A0,1','DBNZ A2,state_finite')
    p.fcopy('q30','fwidth');p.fcopy('q31','fheight')
    for i in range(4):
        p.seti('i',i);p.read(f'q{i}','current','i')
        if i<2:p.unary(f'q{i}','FEXP',f'q{i}')
        else:p.fbin(f'q{i}','FMUL',f'q{i}','fwidth' if i==2 else 'fheight')
    for q,i in ((4,4),(5,5),(6,8),(7,6),(8,7)):
        p.seti('i',i);p.read(f'q{q}','current','i')
    e('MOVI A0,128');p.il(1,'camera');e('MOVI A2,9')
    L('quantize');e('LD F0,A0,0','FCVT.1 F1,F0','ST F1,A1,0','FCVT F0,F1',
      'ST F0,A0,0','ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A2,quantize')
    p.fbin('q23','FDIV','bestcost','fp_total');p.unary('q23','FSQRT','q23')
    p.fbin('q32','FMUL','focal_min','q30');p.fbin('q33','FMUL','focal_max','q30')
    p.copy('trial','current','ns');p.service('baseline')
    p.ib('svc_status',1,0,'residual_ok');p.icopy('status','svc_status');e('BR done')
    L('residual_ok');p.seti('view',0);p.icopy('ptr','baseline');p.fcopy('maxerr','zero')
    L('view_errors');p.seti('t',0);p.fcopy('q22','zero')
    L('point_error');p.read('q18','ptr');p.addi('ptr','ptr',1);p.read('q19','ptr');p.addi('ptr','ptr',1)
    p.unary('q18','FABS','q18');p.unary('q19','FABS','q19');p.fb('q18',6,'q19','hypot_sorted')
    p.fcopy('q20','q18');p.fcopy('q18','q19');p.fcopy('q19','q20')
    L('hypot_sorted');p.fcopy('q21','zero');p.fb('q18',1,'zero','hypot_ready')
    p.fbin('q20','FDIV','q19','q18');p.fbin('q20','FMUL','q20','q20')
    p.fbin('q20','FADD','one','q20');p.unary('q20','FSQRT','q20');p.fbin('q21','FMUL','q18','q20')
    L('hypot_ready');p.fbin('q20','FMUL','q21','q21');p.fbin('q22','FADD','q22','q20')
    p.fb('q21',4,'maxerr','maxerr_ready');p.fcopy('maxerr','q21')
    L('maxerr_ready');p.addi('t','t',1);p.ib('t',3,'points','point_error')
    p.fbin('q20','FDIV','q22','fp_points');p.unary('q20','FSQRT','q20');p.write('viewrms','q20','view')
    p.addi('view','view',1);p.ib('view',3,'views','view_errors')
    p.seti('view',0);p.seti('i',9);p.icopy('ptr','poses')
    L('pose_loop')
    for name in ('rot0','rot1','rot2','q27','q28','q24'):
        p.read(name,'current','i');p.addi('i','i',1)
    e('CALL rotation_forward')
    for k in range(9):p.write('ptr',f'rot{9+k}');p.addi('ptr','ptr',1)
    p.unary('q24','FEXP','q24')
    for row in range(3):
        p.fbin('q18','FMUL',f'rot{9+3*row}','halfcols')
        p.fbin('q18','FSUB',('q27','q28','q24')[row],'q18')
        p.fbin('q19','FMUL',f'rot{10+3*row}','halfrows')
        p.fbin('q18','FSUB','q18','q19');p.fbin('q18','FMUL','q18','square')
        p.write('ptr','q18');p.addi('ptr','ptr',1)
    p.addi('view','view',1);p.ib('view',3,'views','pose_loop')
    p.seti('pair_a',1);p.icopy('pa','poses');p.addi('pa','pa',12);p.fcopy('q26','zero')
    L('angle_outer');p.seti('pair_b',0);p.icopy('pb','poses')
    L('angle_inner');p.il(0,'pa');p.il(1,'pb')
    e('LD F0,A0,2','LD F1,A1,2','FMUL F2,F0,F1',
      'LD F0,A0,5','LD F1,A1,5','FMUL F0,F0,F1','FADD F2,F2,F0',
      'LD F0,A0,8','LD F1,A1,8','FMUL F0,F0,F1','FADD F2,F2,F0','FABS F2,F2')
    p.ld(0,'one');e('FCMP F2,F0','BR.4 angle_clamped','FMOV F2,F0')
    L('angle_clamped');e('FACOS F2,F2');p.ld(0,'q26');e('FCMP F2,F0','BR.4 angle_saved');p.st('q26',2)
    L('angle_saved');p.addi('pair_b','pair_b',1);p.addi('pb','pb',12);p.ib('pair_b',3,'pair_a','angle_inner')
    p.addi('pair_a','pair_a',1);p.addi('pa','pa',12);p.ib('pair_a',3,'views','angle_outer')
    p.seti('metrics',1);p.seti('weak',1);p.ib('views',3,5,'weak_ready');p.fb('q26',3,'normal_weak','weak_ready');p.seti('weak',0)
    L('weak_ready');p.ib('in_converged',1,0,'reject')
    for q in ('q0','q1'):p.fb(q,4,'q32','reject');p.fb(q,6,'q33','reject')
    for q,limit in (('q2','q30'),('q3','q31')):p.fb(q,3,'zero','reject');p.fb(q,6,limit,'reject')
    p.fb('q23',6,'three','reject');p.fb('q26',4,'normal_min','reject')
    e('GUARD reject');p.seti('ix',0);p.seti('iy',0);p.fcopy('fxi','zero');p.fcopy('fyi','zero')
    L('map_loop')
    p.fbin('q18','FSUB','fwidth','one');p.fbin('q18','FMUL','q18','fxi')
    p.fbin('q18','FDIV','q18','thirty_two');p.fbin('q18','FSUB','q18','q2');p.fbin('q9','FDIV','q18','q0')
    p.fbin('q18','FSUB','fheight','one');p.fbin('q18','FMUL','q18','fyi')
    p.fbin('q18','FDIV','q18','twenty_four');p.fbin('q18','FSUB','q18','q3');p.fbin('q10','FDIV','q18','q1')
    # Same evaluation order as validate_result MAP_8..MAP_58.
    operations='''MUL 9 9 18; MUL 10 10 19; ADD 18 19 11;
MUL 4 11 18; ADD one 18 12; MUL 5 11 18; MUL 18 11 18; ADD 12 18 12;
MUL 6 11 18; MUL 18 11 18; MUL 18 11 18; ADD 12 18 12;
MUL two 5 18; MUL 18 11 18; ADD 4 18 13;
MUL three 6 18; MUL 18 11 18; MUL 18 11 18; ADD 13 18 13;
MUL two 9 18; MUL 18 9 18; MUL 18 13 18; ADD 12 18 14;
MUL two 7 18; MUL 18 10 18; ADD 14 18 14;
MUL six 8 18; MUL 18 9 18; ADD 14 18 14;
MUL two 9 18; MUL 18 10 18; MUL 18 13 15;
MUL two 7 18; MUL 18 9 18; ADD 15 18 15;
MUL two 8 18; MUL 18 10 18; ADD 15 18 15;
MUL two 10 18; MUL 18 10 18; MUL 18 13 18; ADD 12 18 16;
MUL six 7 18; MUL 18 10 18; ADD 16 18 16;
MUL two 8 18; MUL 18 9 18; ADD 16 18 16;
MUL 14 16 18; MUL 15 15 19; SUB 18 19 17'''
    for text in operations.split(';'):
        op,a,b,d=text.split();a='q'+a if a.isdigit() else a;b='q'+b if b.isdigit() else b
        p.fbin('q'+d,'F'+op,a,b)
    p.fb('q14',4,'zero','reject');p.fb('q16',4,'zero','reject');p.fb('q17',4,'jacobian_min','reject')
    p.addi('ix','ix',1);p.fbin('fxi','FADD','fxi','one');p.ib('ix',4,32,'map_loop')
    p.seti('ix',0);p.fcopy('fxi','zero');p.addi('iy','iy',1);p.fbin('fyi','FADD','fyi','one');p.ib('iy',4,24,'map_loop')
    p.seti('usable',1);e('BR done')
    L('bad_configuration');p.seti('status',1);e('BR done')
    L('check_failed');e('MOVI A7,0');p.seti('metrics',0)
    L('reject');p.seti('status',4);p.seti('usable',0)
    L('done');e('END 0')
    rotation(p)
    return '\n'.join(p.lines)+'\n'

def build():
    source=program();words,labels=assemble(source,2048)
    out=ROOT/'build/calibration_engine';out.mkdir(parents=True,exist_ok=True)
    (out/'validate.asm').write_text(source,encoding='utf-8')
    (ROOT/'rtl/include/validate_program_init.vh').write_text('// Generated by build_validate.py.\n'+
        '\n'.join(f"program_memory[{i}]=32'h{w:08x};" for i,w in enumerate(words))+'\n',encoding='utf-8')
    (ROOT/'rtl/include/validate_program_defs.vh').write_text('// Generated by build_validate.py.\n'+
        f'`define VALIDATE_PROGRAM_WORDS {len(words)}\n',encoding='utf-8')
    print(f'validate ROM: {len(words)} words; includes rotation/report/map checks')
    return words,labels
if __name__=='__main__':build()
