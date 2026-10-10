"""Normalized DLT + Jacobi eigensolve + single focal seed, all instructions.

Host loads packed FP32 corners and centered unit-grid coordinates once.
Workspace: h[63] at 256; pose at 320; symmetric matrix at 464; eigenvectors
at 560; eigenvalues/order at 650/660; DLT row at 704; per-view H at 800.
Image/grid arrays and final state follow the packed corners from address 1024.
"""
from build_lm import Program,M
from assemble import ROOT,assemble

def addr(p,out,row,col,base):
    p.il(0,row);p.emit('IADD A1,A0,A0','IADD A1,A1,A1','IADD A1,A1,A1','IADD A0,A0,A1')
    p.il(1,col);p.emit('IADD A0,A0,A1',f'ADDI A0,A0,{base}');p.ist(out)

def eigen(p):
    e=p.emit;L=p.label
    L('eigen');p.seti('row',0);p.seti('j',0);e('MOVI A0,560','MOVI A1,0','MOVI A2,0','MOVI A3,9')
    p.ld(0,'zero');p.ld(1,'one')
    L('e_identity');e('ICMP A1,A2','BR.2 e_identity_zero','ST F1,A0,0','BR e_identity_next')
    L('e_identity_zero');e('ST F0,A0,0')
    L('e_identity_next');e('ADDI A0,A0,1','ADDI A2,A2,1','ICMP A2,A3','BR.3 e_identity',
      'MOVI A2,0','ADDI A1,A1,1','ICMP A1,A3','BR.3 e_identity')
    p.seti('attempt',0)
    L('e_search');p.ib('attempt',6,8100,'init_error')
    p.fcopy('maxerr','zero');p.fcopy('threshold','zero');p.seti('row',0);p.seti('j',0)
    p.seti('pair_a',0);p.seti('pair_b',1)
    L('e_scan');addr(p,'ptr','row','j',464);p.read('tmp','ptr');p.unary('tmp2','FABS','tmp')
    p.ib('row',2,'j','e_offscan')
    p.il(0,'row');e('ADDI A0,A0,650');p.ld(0,'tmp');e('ST F0,A0,0')
    p.fb('tmp2',4,'threshold','e_scan_next');p.fcopy('threshold','tmp2');e('BR e_scan_next')
    L('e_offscan');p.fb('tmp2',4,'maxerr','e_scan_next')
    p.fcopy('maxerr','tmp2');p.icopy('pair_a','row');p.icopy('pair_b','j')
    L('e_scan_next');p.addi('j','j',1);p.ib('j',3,9,'e_scan')
    p.addi('row','row',1);p.icopy('j','row');p.ib('row',3,9,'e_scan')
    p.fb('threshold',6,'eigen_tiny','e_bound_ready');p.fcopy('threshold','eigen_tiny')
    L('e_bound_ready');p.fbin('threshold','FMUL','threshold','eigen_tolerance')
    p.fb('maxerr',4,'threshold','e_sort')
    addr(p,'ptr','pair_a','pair_a',464);p.read('original','ptr')
    addr(p,'ptr','pair_b','pair_b',464);p.read('perturb','ptr')
    addr(p,'ptr','pair_a','pair_b',464);p.read('norm','ptr')
    p.fbin('q3','FMUL','two','norm');p.fbin('q4','FSUB','perturb','original')
    p.fbin('q5','FATAN2','q3','q4');p.fbin('q5','FMUL','half','q5')
    p.unary('q6','FCOS','q5');p.unary('q7','FSIN','q5');p.seti('k',0)
    L('e_offrow');p.ib('k',1,'pair_a','e_offnext');p.ib('k',1,'pair_b','e_offnext')
    addr(p,'pa','k','pair_a',464);addr(p,'pb','k','pair_b',464)
    p.read('q0','pa');p.read('q1','pb');e('CALL e_pair')
    p.write('pa','q10');p.write('pb','q11')
    addr(p,'pa','pair_a','k',464);addr(p,'pb','pair_b','k',464)
    p.write('pa','q10');p.write('pb','q11')
    L('e_offnext');p.addi('k','k',1);p.ib('k',3,9,'e_offrow')
    for op,a,b,d in [('FMUL','q6','q6','q8'),('FMUL','q7','q7','q9'),
      ('FMUL','two','q7','q10'),('FMUL','q10','q6','q10'),('FMUL','q10','norm','q10'),
      ('FMUL','q8','original','q11'),('FMUL','q9','perturb','q12'),('FSUB','q11','q10','q13'),
      ('FADD','q13','q12','q14'),('FMUL','q9','original','q11'),('FMUL','q8','perturb','q12'),
      ('FADD','q11','q10','q13'),('FADD','q13','q12','q15')]:p.fbin(d,op,a,b)
    for a,b,val in [('pair_a','pair_a','q14'),('pair_b','pair_b','q15'),('pair_a','pair_b','zero'),('pair_b','pair_a','zero')]:
        addr(p,'ptr',a,b,464);p.write('ptr',val)
    p.seti('k',0)
    L('e_vectors');addr(p,'pa','k','pair_a',560);addr(p,'pb','k','pair_b',560)
    p.read('q0','pa');p.read('q1','pb');e('CALL e_pair');p.write('pa','q10');p.write('pb','q11')
    p.addi('k','k',1);p.ib('k',3,9,'e_vectors');p.addi('attempt','attempt',1);e('BR e_search')
    L('e_pair')
    for op,a,b,d in [('FMUL','q6','q0','q8'),('FMUL','q7','q1','q9'),('FSUB','q8','q9','q10'),
                    ('FMUL','q7','q0','q8'),('FMUL','q6','q1','q9'),('FADD','q8','q9','q11')]:p.fbin(d,op,a,b)
    e('RET')
    L('e_sort');e('MOVI A0,660','MOVI A1,0','MOVI A2,9')
    L('e_order');e('STI32 A1,A0,0','ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A2,e_order')
    e('MOVI A6,8')
    L('e_sort_pass');e('MOVI A0,660','ADDI A5,A6,0')
    L('e_sort_cell');e('LDI32 A1,A0,0','LDI32 A2,A0,1','ADDI A3,A1,650','ADDI A4,A2,650',
      'LD F0,A3,0','LD F1,A4,0','FCMP F1,F0','BR.6 e_sort_next','STI32 A2,A0,0','STI32 A1,A0,1')
    L('e_sort_next');e('ADDI A0,A0,1','DBNZ A5,e_sort_cell','DBNZ A6,e_sort_pass',
      'MOVI A0,660','LDI32 A1,A0,0','LDI32 A2,A0,1','LDI32 A3,A0,8',
      'ADDI A2,A2,650','ADDI A3,A3,650','LD F0,A2,0','LD F1,A3,0')
    p.st('h49',0);p.st('h50',1)
    e('ADDI A1,A1,560','MOVI A0,296','MOVI A2,9')
    L('e_result');e('LD F0,A1,0','ST F0,A0,0','ADDI A1,A1,9','ADDI A0,A0,1','DBNZ A2,e_result','RET')

def homography(p):
    e=p.emit;L=p.label
    L('homography')
    for name in ('h2','h3','h4','h5'):p.fcopy(name,'zero')
    p.seti('t',0)
    L('h_load');p.il(0,'pa');e('LD F0,A0,0','FGET32 F1,F0','FGET32.1 F2,F0','FCVT F1,F1','FCVT F2,F2')
    p.st('h8',1);p.st('h9',2)
    p.fb('h8',3,'zero','init_error');p.fb('h9',3,'zero','init_error')
    p.fb('h8',6,'fwidth','init_error');p.fb('h9',6,'fheight','init_error')
    p.write('init_px','h8','t');p.write('init_py','h9','t')
    p.fbin('h2','FADD','h2','h8');p.fbin('h3','FADD','h3','h9')
    p.addi('pa','pa',1);p.addi('t','t',1);p.ib('t',3,'points','h_load')
    p.fbin('h2','FDIV','h2','fp_points');p.fbin('h3','FDIV','h3','fp_points');p.seti('t',0)
    L('h_distance')
    p.read('h10','init_px','t');p.read('h11','init_py','t')
    p.fbin('h10','FSUB','h10','h2');p.fbin('h11','FSUB','h11','h3')
    p.fbin('h10','FMUL','h10','h10');p.fbin('h11','FMUL','h11','h11')
    p.fbin('h10','FADD','h10','h11');p.unary('h10','FSQRT','h10');p.fbin('h4','FADD','h4','h10')
    p.read('h10','init_bx','t');p.read('h11','init_by','t')
    p.fbin('h10','FMUL','h10','h10');p.fbin('h11','FMUL','h11','h11')
    p.fbin('h10','FADD','h10','h11');p.unary('h10','FSQRT','h10');p.fbin('h5','FADD','h5','h10')
    p.addi('t','t',1);p.ib('t',3,'points','h_distance')
    p.fb('h4',3,'difference_step','init_error');p.fb('h5',3,'difference_step','init_error')
    p.fbin('h53','FMUL','sqrt_two','fp_points');p.fbin('h6','FDIV','h53','h4');p.fbin('h7','FDIV','h53','h5')
    e('MOVI A0,464','MOVI A1,81');p.ld(0,'zero')
    L('h_clear');e('ST F0,A0,0','ADDI A0,A0,1','DBNZ A1,h_clear');p.seti('t',0)
    L('h_point');p.read('h16','init_bx','t');p.read('h17','init_by','t')
    p.fbin('h16','FMUL','h16','h7');p.fbin('h17','FMUL','h17','h7')
    p.read('h18','init_px','t');p.fbin('h18','FSUB','h18','h2');p.fbin('h18','FMUL','h18','h6')
    p.read('h19','init_py','t');p.fbin('h19','FSUB','h19','h3');p.fbin('h19','FMUL','h19','h6')
    p.seti('row',0)
    L('h_constraint');e('MOVI A0,704','MOVI A1,9');p.ld(0,'zero')
    L('h_row_clear');e('ST F0,A0,0','ADDI A0,A0,1','DBNZ A1,h_row_clear')
    p.il(0,'row');e('IADD A1,A0,A0','IADD A0,A0,A1','ADDI A0,A0,704')
    p.ld(0,'h16');p.ld(1,'h17');p.ld(2,'one')
    e('FNEG F0,F0','FNEG F1,F1','FNEG F2,F2','ST F0,A0,0','ST F1,A0,1','ST F2,A0,2')
    p.il(0,'row');e('ADDI A0,A0,274','LD F0,A0,0') # h18 or h19
    p.ld(1,'h16');p.ld(2,'h17');e('FMUL F1,F0,F1','FMUL F2,F0,F2',
      'MOVI A0,704','ST F1,A0,6','ST F2,A0,7','ST F0,A0,8',
      'MOVI A0,464','MOVI A1,704','MOVI A3,9')
    L('h_outer_row');e('LD F0,A1,0','MOVI A2,704','MOVI A4,9')
    L('h_outer_col');e('LD F1,A2,0','FMUL F1,F0,F1','LD F2,A0,0','FADD F2,F2,F1',
      'ST F2,A0,0','ADDI A0,A0,1','ADDI A2,A2,1','DBNZ A4,h_outer_col',
      'ADDI A1,A1,1','DBNZ A3,h_outer_row')
    p.addi('row','row',1);p.ib('row',3,2,'h_constraint');p.addi('t','t',1);p.ib('t',3,'points','h_point')
    e('CALL eigen')
    p.fbin('h51','FMUL','h50','eigen_separation');p.fb('h49',3,'zero','init_error');p.fb('h49',3,'h51','init_error')
    p.fbin('h52','FDIV','one','h6')
    for row in range(2):
        for col in range(3):
            p.fbin('h10','FMUL','h52',f'h{40+3*row+col}')
            p.fbin('h11','FMUL',f'h{2+row}',f'h{46+col}')
            p.fbin(f'h{54+3*row+col}','FADD','h10','h11')
            if col<2:p.fbin(f'h{54+3*row+col}','FMUL',f'h{54+3*row+col}','h7')
    p.fbin('h60','FMUL','h46','h7');p.fbin('h61','FMUL','h47','h7');p.fcopy('h62','h48')
    p.unary('tmp','FABS','h62');p.fb('tmp',3,'scale_floor','init_error')
    for i in range(9):p.fbin(f'h{54+i}','FDIV',f'h{54+i}','h62')
    e('RET')

def program():
    p=Program();e=p.emit;L=p.label
    e('MOVI A7,0','GUARD init_error');p.seti('status',0);p.seti('view',0);p.icopy('pa','init_raw');p.seti('colbase',800)
    L('init_view');p.icopy('endptr','pa');e('CALL homography')
    # Subroutines clobber cursors; derive next view from saved raw pointer.
    p.ibin('pa','IADD','endptr','points')
    p.il(0,'colbase');e('MOVI A1,310','MOVI A2,9')
    L('save_h');e('LD F0,A1,0','ST F0,A0,0','ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A2,save_h')
    p.ist('colbase',0);p.addi('view','view',1);p.ib('view',3,'views','init_view')
    # Reject only if every other view repeats H0 (same as the old initializer).
    p.ld(4,'zero');e('MOVI A0,809');p.il(3,'views');e('ADDI A3,A3,-1')
    L('duplicate_view');e('MOVI A1,800','MOVI A2,8')
    L('duplicate_element');e('LD F0,A0,0','LD F1,A1,0','FSUB F0,F0,F1','FABS F0,F0','FADD F4,F4,F0',
      'ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A2,duplicate_element','ADDI A0,A0,1','DBNZ A3,duplicate_view')
    p.ld(0,'difference_step');e('FCMP F4,F0','BR.3 init_error')
    # Prepare the existing pose program's isolated 128-word work area at 320.
    e('MOVI A0,320');p.ld(0,'fwidth');p.ld(1,'fheight');e('ST F0,A0,0','ST F0,A0,1','ST F0,A0,4','ST F1,A0,5')
    p.ld(2,'one');p.ld(3,'half');e('FSUB F0,F0,F2','FMUL F0,F0,F3','ST F0,A0,2',
      'FSUB F1,F1,F2','FMUL F1,F1,F3','ST F1,A0,3')
    for offset,name in ((92,'zero'),(93,'one'),(94,'two'),(95,'scale_floor')):
        p.ld(0,name);e(f'ST F0,A0,{offset}')
    p.ld(0,'two');e('FADD F0,F0,F0','ST F0,A0,96','CALL pose_common','MOVI A7,0')
    p.il(0,'init_seed');e('MOVI A1,360','MOVI A2,4')
    L('save_intrinsics');e('LD F0,A1,0','ST F0,A0,0','ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A2,save_intrinsics')
    p.ld(0,'zero');e('MOVI A2,5')
    L('save_distortion');e('ST F0,A0,0','ADDI A0,A0,1','DBNZ A2,save_distortion');p.ist('ptr',0)
    p.seti('view',0);p.seti('pb',800)
    L('init_pose');p.il(1,'pb');e('MOVI A0,400','MOVI A2,9')
    L('load_pose_h');e('LD F0,A1,0','ST F0,A0,0','ADDI A0,A0,1','ADDI A1,A1,1','DBNZ A2,load_pose_h');p.ist('pb',1)
    e('CALL pose_view','MOVI A7,0');p.il(0,'ptr');e('MOVI A1,320')
    for offset in (55,56,57,30,31,54):e(f'LD F0,A1,{offset}','ST F0,A0,0','ADDI A0,A0,1')
    p.ist('ptr',0);p.addi('view','view',1);p.ib('view',3,'views','init_pose');e('END 0')
    L('init_error');e('MOVI A7,0');p.seti('status',4);e('END 0')
    homography(p);eigen(p)
    # Reuse the maintained pose source without duplicating its algorithm.
    source='\n'.join((ROOT/'data/programs/calibration'/name).read_text(encoding='utf-8') for name in ('pose.asm','rotation_inverse.asm'))
    source=source.replace('MOVI A0,0','MOVI A0,320').replace('END 0','RET').replace('END 210','BR init_error')
    e(source)
    return '\n'.join(p.lines)+'\n'

def build():
    source=program();words,labels=assemble(source,4096)
    (ROOT/'build/calibration_engine').mkdir(parents=True,exist_ok=True)
    (ROOT/'build/calibration_engine/init.asm').write_text(source,encoding='utf-8')
    (ROOT/'rtl/include/init_program_init.vh').write_text('// Generated by build_init.py.\n'+
        '\n'.join(f"program_memory[{i}]=32'h{w:08x};" for i,w in enumerate(words))+'\n',encoding='utf-8')
    (ROOT/'rtl/include/init_program_defs.vh').write_text('// Generated by build_init.py.\n'+f'`define INIT_PROGRAM_WORDS {len(words)}\n',encoding='utf-8')
    print(f'init ROM: {len(words)} words; DLT, Jacobi and pose are instructions')
    return words,labels
if __name__=='__main__':build()
