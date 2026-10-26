"""Check lens ROM against independent scalar equations, including extrema.

No camera or artificial-distortion preset is used by the optimizer. Random
coefficients exercise zero/linear/quadratic derivatives and infeasible probes.
"""
import math
import random
from assemble import assemble
from program import Program
from workspace import M, C, CONSTANTS
from build_lens import append_program
from reference import Machine, bits, value
from lens_policy import regularization_residuals, shape_margins


def main():
    p=Program();p.emit('MOVI A7,0','CALL lens_objective','END 0');append_program(p)
    rom,_=assemble('\n'.join(p.lines),1024)
    rng=random.Random(174)
    cases=[]
    for k1,k2,k3 in ((0,0,0),(-.3,0,0),(0,.2,0),(0,0,-.1),(.2,-1,.4),(-1,1,0)):
        cases.append([math.log(640),math.log(650),.5,.5,k1,k2,.002,-.001,k3])
    for _ in range(150):
        cases.append([math.log(rng.uniform(250,1600)),math.log(rng.uniform(250,1600)),
                      rng.uniform(-.1,1.1),rng.uniform(-.1,1.1),
                      *[rng.uniform(-2,2) for _ in range(2)],
                      rng.uniform(-.05,.05),rng.uniform(-.05,.05),rng.uniform(-2,2)])
    instructions=0
    for state in cases:
        width,height=960,540
        m=Machine(rom,2048,2048)
        m.mem.update({C[k]:v for k,v in CONSTANTS.items()})
        for key,val in dict(trial=512,svc_dst=1024,data_nr=240,
                            fwidth=bits(width),fheight=bits(height),svc_cost=bits(7.),
                            lens_weight=bits(.05*math.sqrt(120)),
                            barrier_weight=bits(.01*math.sqrt(120))).items():m.mem[M[key]]=val
        for i,x in enumerate(state):m.mem[512+i]=bits(x)
        assert m.run(0,len(rom))==0
        expected=regularization_residuals(state,width,height,120)
        actual=[value(m.mem[1264+i]) for i in range(4)]
        for a,b in zip(actual,expected):assert abs(a-b)<1e-9*(1+abs(b)),(state,actual,expected)
        margin=value(m.mem[M['lens_margin']]);expected_margin=min(shape_margins(state,width,height))
        assert abs(margin-expected_margin)<1e-10*(1+abs(expected_margin))
        objective=value(m.mem[M['svc_objective']])
        assert abs(objective-(7+sum(x*x for x in expected)))<1e-9*(1+objective)
        instructions=max(instructions,m.counts['instructions'])
    print(f'LENS_PROGRAM_PASS cases={len(cases)} max_instructions_per_evaluation={instructions}')


if __name__=='__main__':main()
