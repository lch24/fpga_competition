"""Generate scoped engine transactions and independent numerical expectations.

No board/camera simulation. Exercises real assembled loops, calls, floating
arithmetic, kernels and rejected transactions through the public host port.
"""
import math
import struct
from pathlib import Path
from assemble import ROOT, assemble, header, legality_function

OUT=ROOT/'build/calibration_engine'

def bits(value):
    return struct.unpack('<Q',struct.pack('<d',value))[0]

def generate():
    OUT.mkdir(parents=True,exist_ok=True)
    assert (ROOT/'rtl/include/calib_engine_defs.vh').read_text()==header()
    assert (ROOT/'rtl/include/calib_engine_decode.vh').read_text()==legality_function()
    words=[];commands=[];cases=[]
    def put(addr,value): commands.append(f'0 {addr:x} {value:016x}')
    def expect(addr,value): commands.append(f'2 {addr:x} {value:016x}')
    def near(addr,value): commands.append(f'5 {addr:x} {bits(value):016x}')
    def program(name,source,status=0,patch=None,cancel=False):
        code,labels=assemble(source)
        if patch: code[patch[0]]=patch[1]
        entry=len(words);words.extend(code)
        commands.append(f'{4 if cancel else 1} {entry:x} {status:x}')
        cases.append(dict(name=name,entry=entry,words=len(code),status=status))
    def descriptor(kind,src0=0,src1=0,dst=0,count=0,s0=1,s1=1,sd=1,alpha=0,n=0,status=0):
        for i,value in enumerate([src0|(src1<<32),dst|(count<<32),s0|(s1<<32),sd,n,bits(alpha),0,0]):put(800+i,value)
        program(f'kernel_{kind}_{len(cases)}',f'MOVI A0,800\nKEXEC A0,{kind}\nEND 0',status)
    for addr,value in enumerate([1.,2.,3.,4.,-2.,0.5]):put(addr,bits(value))
    program('scalar', '''
        MOVI A0,0
        LD F0,A0,0
        LD F1,A0,1
        FADD F2,F0,F1
        FSUB F3,F0,F1
        FMUL F4,F2,F3
        FDIV F5,F4,F1
        FSQRT F6,F1
        ST F2,A0,100
        ST F3,A0,101
        ST F4,A0,102
        ST F5,A0,103
        ST F6,A0,104
        END 0
    ''')
    for addr,value in enumerate([3.,-1.,-3.,-1.5,math.sqrt(2.)],100):expect(addr,bits(value))
    program('loop_and_call','''
        MOVI A0,0
        MOVI A1,5
        MOVI A2,0
    loop:
        CALL add
        DBNZ A1,loop
        STI32 A2,A0,105
        END 0
    add:
        ADDI A2,A2,3
        RET
    ''');expect(105,15)
    # Packed corner conversion and bit-preserving operations.
    put(6,0xc02000003fc00000)
    program('packed_corner','''
        MOVI A0,0
        LD F0,A0,6
        FGET32 F1,F0
        FGET32.1 F2,F0
        FCVT F1,F1
        FCVT F2,F2
        FNEG F3,F2
        FABS F4,F2
        FMOV F5,F4
        FCVT.1 F6,F1
        CLASS A1,F1
        ST F1,A0,106
        ST F2,A0,107
        ST F3,A0,108
        ST F5,A0,109
        ST F6,A0,110
        STI32 A1,A0,111
        END 0
    ''')
    for addr,value in [(106,bits(1.5)),(107,bits(-2.5)),(108,bits(2.5)),(109,bits(2.5)),(110,0x3fc00000),(111,4)]:expect(addr,value)
    descriptor(2,src0=0,src1=3,dst=120,count=3)
    expect(120,bits(1*4+2*(-2)+3*.5))
    descriptor(3,src0=0,src1=3,dst=130,count=3,alpha=2)
    for i,v in enumerate([6.,2.,6.5]):expect(130+i,bits(v))
    # AXPY may update exactly its second input, preserving each original item.
    descriptor(3,src0=0,src1=130,dst=130,count=3,alpha=-1)
    for i,v in enumerate([5.,0.,3.5]):expect(130+i,bits(v))
    for i in range(6):put(140+i,bits(float(i)))
    descriptor(4,src0=0,dst=140,count=3,n=3)
    for i,v in enumerate([1.,3.,5.,7.,10.,14.]):expect(140+i,bits(v))
    descriptor(1,src0=140,dst=150,count=6)
    for i,v in enumerate([1.,3.,5.,7.,10.,14.]):expect(150+i,bits(v))
    descriptor(0,dst=150,count=6)
    for i in range(6):expect(150+i,0)
    descriptor(2,src0=0xffffffff,src1=0xffffffff,dst=160,count=0);expect(160,0)
    # Before-write rejection: preserve sentinels after invalid bounds/overlap.
    put(170,bits(71.))
    descriptor(1,src0=4095,dst=170,count=2,status=0xe6);expect(170,bits(71.))
    descriptor(1,src0=0,dst=1,count=3,status=0xe6);expect(1,bits(2.))
    descriptor(4,src0=0,dst=3583,count=3,n=3,status=0xe6)
    # Actual local Jacobian width: 8 global + 6 pose variables. A whole
    # residual row updates 105 entries. Measure this, not a 3-element toy only.
    vec=[(i-6)*.125 for i in range(14)]
    for i,v in enumerate(vec):put(600+i,bits(v))
    for i in range(105):put(1500+i,0)
    descriptor(4,src0=600,dst=1500,count=14,n=14)
    offset=0
    for i in range(14):
        for j in range(i,14):
            expect(1500+offset,bits(vec[i]*vec[j]+0.));offset+=1
    aa=[(i-60)/16 for i in range(120)]
    bb=[(i%11-5)/8 for i in range(120)]
    for i,(a,b) in enumerate(zip(aa,bb)):put(600+i,bits(a));put(1000+i,bits(b))
    descriptor(2,src0=600,src1=1000,dst=1200,count=120)
    acc=0.
    for a,b in zip(aa,bb):acc+=a*b
    expect(1200,bits(acc))
    program('negative_address','MOVI A0,0\nLD F0,A0,-1\nEND 0',0xe4)
    program('readonly_write','MOVI A0,3584\nST F0,A0,0\nEND 0',0xe4)
    program('empty_loop','MOVI A0,0\nDBNZ A0,0\nEND 0',0xe7)
    program('stack_underflow','RET\nEND 0',0xe3)
    program('stack_overflow','again: CALL again\nEND 0',0xe3)
    program('reserved_bits','NOP\nEND 0',0xe1,patch=(0,1))
    program('unsupported_fp32','FADD F0,F0,F0\nEND 0',0xe1,patch=(0,(32<<26)|(1<<23)))
    program('pc_underflow','BR 0\nEND 0',0xe2,patch=(0,(7<<26)|0x2000))
    program('cancel_divide','MOVI A0,0\nLD F0,A0,0\nLD F1,A0,2\nFDIV F2,F0,F1\nEND 0',cancel=True)
    program('restart_after_cancel','END 0')
    # A zero/NaN comparison must not pass ordered inequalities.
    put(7,0x7ff8000000000000)
    program('nan_flags','''
        MOVI A0,0
        LD F0,A0,7
        FCMP F0,F0
        BR.7 unordered
        END 1
    unordered:
        BR.3 bad
        BR.6 bad
        BR.2 neq
    bad:
        END 2
    neq:
        END 0
    ''')
    # Algorithm-level tests are tolerance-based because operation order differs
    # from native math. Tolerance is set in the TB before evaluating results.
    import random
    rng=random.Random(271828)
    camera=[980.,975.,640.,360.,-.14,.027,.001,-.002]
    for i,v in enumerate(camera):put(200+i,bits(v))
    points=[(0.,0.,1.)]+[(rng.uniform(-2,2),rng.uniform(-1,1),rng.uniform(.5,8)) for _ in range(39)]
    for i,xyz in enumerate(points):
        for j,v in enumerate(xyz):put(250+3*i+j,bits(v))
    projection=(ROOT/'data/programs/calibration/project.asm').read_text()
    program('project_40_points',f'''
        MOVI A0,200
        MOVI A1,250
        MOVI A2,{len(points)}
        MOVI A3,450
    project_loop:
        LD F0,A1,0
        LD F1,A1,1
        LD F2,A1,2
        CALL project
        ST F0,A3,0
        ST F1,A3,1
        ADDI A1,A1,3
        ADDI A3,A3,2
        DBNZ A2,project_loop
        END 0
    '''+projection)
    for i,(X,Y,Z) in enumerate(points):
        x,y=X/Z,Y/Z;r2=x*x+y*y
        radial=1+camera[4]*r2+camera[5]*r2*r2
        xd=x*radial+2*camera[6]*x*y+camera[7]*(r2+2*x*x)
        yd=y*radial+camera[6]*(r2+2*y*y)+2*camera[7]*x*y
        near(450+2*i,camera[0]*xd+camera[2]);near(451+2*i,camera[1]*yd+camera[3])
    constants={0:0.,1:1.,2:2.,3:.5,4:math.log(2),5:1/math.log(2),6:700.,7:-700.}
    constants.update({16+i:1/math.factorial(i) for i in range(17)})
    constants.update({40+i:1/(2*i+1) for i in range(25)})
    for i,v in constants.items():put(3584+i,bits(v))
    math_source=(ROOT/'data/programs/calibration/math.asm').read_text()
    exp_values=[-700.,-100.,-10.,-1.,-1e-12,0.,1e-12,1.,10.,100.,700.]+[rng.uniform(-15,15) for _ in range(30)]
    log_values=[5e-324,1e-300,.01,.5,1.,1+2**-52,2.,100.,1e300]+[math.exp(rng.uniform(-15,15)) for _ in range(30)]
    for routine,values,function in [('exp',exp_values,math.exp),('log',log_values,math.log)]:
        for i,v in enumerate(values):put(250+i,bits(v))
        program('math_'+routine,f'''
            MOVI A7,3584
            MOVI A0,250
            MOVI A1,450
            MOVI A2,{len(values)}
        math_test_loop:
            LD F0,A0,0
            CALL math_{routine}
            ST F0,A1,0
            ADDI A0,A0,1
            ADDI A1,A1,1
            DBNZ A2,math_test_loop
            END 0
        '''+math_source)
        for i,v in enumerate(values):near(450+i,function(v))
    solver=(ROOT/'data/programs/calibration/solve.asm').read_text()
    solve_cases=[]
    for n in (1,3,8,26):
        matrix=[[(i*3+j*5)%7-3 for j in range(n)] for i in range(n)]
        for i in range(n):matrix[i][i]=sum(abs(x) for x in matrix[i])+2
        solution=[(i%5-2)/4 for i in range(n)]
        if n==3:matrix=[[0.,2.,1.],[1.,1.,0.],[2.,0.,3.]];solution=[1.,-2.,.5]
        rhs=[sum(a*x for a,x in zip(row,solution)) for row in matrix]
        solve_cases.append((matrix,rhs,solution,0))
    solve_cases.extend([([[1.,2.],[2.,4.]],[1.,2.],[],0xd2),([[math.nan,0.],[0.,1.]],[1.,1.],[],0xd2)])
    for index,(matrix,rhs,solution,status) in enumerate(solve_cases):
        n=len(matrix)
        for i,row in enumerate(matrix):
            for j,v in enumerate(row+[rhs[i]]):put(i*(n+1)+j,bits(v))
        for offset,v in [(24,1e-30),(25,1e-14),(26,0.),(27,float.fromhex('0x1.fffffffffffffp+1023'))]:put(2300+offset,bits(v))
        program(f'solve_{n}_{index}',f'''
            MOVI A0,0
            MOVI A1,{n}
            MOVI A2,{n+1}
            MOVI A3,1000
            MOVI A7,2300
            CALL solve
            END 0
        '''+solver,status)
        if status==0:
            for i,v in enumerate(solution):near(1000+i,v)
    # Generated aggregation contains independent programs with relative calls.
    if len(words)>2048:raise ValueError('test ROM capacity exceeded')
    (OUT/'checks.mem').write_text(''.join(f'{w:08x}\n' for w in words))
    (OUT/'checks.txt').write_text(f'3 {len(words):x} 0\n'+'\n'.join(commands)+'\n')
    import json
    (OUT/'checks.json').write_text(json.dumps(cases,indent=2))
    # Front-end diagnostics: no silent register/immediate/mode truncation.
    bad=['MOVI A8,1','MOVI A0,16384','ADDI A0,A1,-8193','FADD A0,F1,F2',
         'NOP 0','LD F0,A0,8192','BR absent','x:NOP\nx:END 0','FADD.1 F0,F0,F0','KEXEC A0,5']
    for source in bad:
        try: assemble(source)
        except ValueError: pass
        else: raise AssertionError(f'accepted {source}')
    print(f'{len(cases)} engine cases, {len(commands)} host actions, {len(words)} instructions; assembler rejection checks PASS')

if __name__=='__main__':generate()
