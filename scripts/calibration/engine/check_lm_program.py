"""Instruction-level LM checks with an independent, full-rank linear residual.

This isolates iteration/solve/rollback/control from projection. Real camera
fixtures are separately checked by check_lm.py with the actual RTL service.
"""
import math
from build_lm import build, M, C, CONSTANTS
from reference import Machine,bits,value

def layout(views,points):
    n=8+6*views;ns=9+6*views;nr=2*views*points
    sizes=[('desc',4*n),('current',ns),('trial',ns),('baseline',nr),
           ('plus',nr),('minus',nr),('column',nr),('jac',14*nr),
           ('normal',n*n),('augmented',n*(n+1)),('scales',n),('delta',n),('gradient',n)]
    at=288;result={}
    for name,size in sizes:result[name]=at;at+=size
    return result,1<<(at-1).bit_length()

def run_case(rom,views=3,points=40,mode='normal'):
    n=8+6*views;ns=n+1;nr=2*views*points;l,words=layout(views,points)
    calls=0;accepted_costs=[];snapshots=[]
    def service(m,opcode):
        nonlocal calls
        assert opcode==0;calls+=1
        state=[value(m.read(l['trial']+i)) for i in range(ns)]
        dst=m.read(M['svc_dst']);status=0
        if mode=='missing':status=5
        if mode=='invalid_baseline' and calls==1:status=4
        if mode=='invalid_difference' and calls==2:status=4
        total=0
        for row in range(nr):
            v=row//(2*points);axis=row%(2*points)%14
            index=axis if axis<8 else 9+6*v+axis-8
            r=state[index];total+=r*r;m.write(dst+row,bits(r))
        # Fail only trial evaluations, after initial and 2*n perturbations.
        if mode=='reject' and calls>1+2*n:total=1e200
        m.write(M['svc_status'],status);m.write(M['svc_cost'],bits(total))
        if dst==l['minus'] and calls>1+2*n:snapshots.append(state)
    m=Machine(rom,words,words,service,instruction_limit=5000000)
    m.mem.update({i:0 for i in range(288)})
    m.mem.update({C[k]:v for k,v in CONSTANTS.items()})
    m.mem.update({280:bits(1e-30),281:bits(1e-14),282:0,283:0x7fefffffffffffff})
    for key,val in dict(l,n=n,nr=nr,ns=ns,maxiter=8,maxtries=3).items():m.mem[M[key]]=val
    initial=[0.01*(i+1) for i in range(ns)];initial[8]=0
    for i,x in enumerate(initial):m.mem[l['current']+i]=bits(x)
    for k in range(n):
        state_index=k if k<4 else k+5 if k<4+6*views else k-6*views
        first=0;last=nr;packed=k if k<4 else k-6*views
        if 4<=k<4+6*views:
            view,axis=divmod(k-4,6);first=view*2*points;last=first+2*points;packed=8+axis
        for j,x in enumerate((state_index,first,last,l['jac']+packed*nr)):m.mem[l['desc']+4*k+j]=x
    status=m.run(0,len(rom));assert status==0,(mode,status)
    get=lambda k:m.mem[M[k]]
    result=[value(m.mem[l['current']+i]) for i in range(ns)]
    if mode=='normal':
        assert get('status')==0 and get('converged')==1
        assert max(abs(x) for x in result)<1e-7,(views,result)
        assert value(get('cost'))<1e-12
    elif mode=='missing':assert get('status')==5 and result==initial
    else:
        assert get('status')==0 and get('converged')==0 and get('accepted')==0 and result==initial
        if mode=='reject':assert get('attempt')==3 and len(snapshots)==3
    print(f'LM_PROGRAM_MODEL_PASS views={views} mode={mode} instructions={m.counts["instructions"]} evals={calls} cost={value(get("cost")):.9g}',flush=True)

if __name__=='__main__':
    rom,_=build()
    for mode in ('normal','missing','invalid_baseline','invalid_difference','reject'):run_case(rom,mode=mode)
    run_case(rom,views=4,points=42)
