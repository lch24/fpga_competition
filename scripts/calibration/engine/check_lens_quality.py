"""Independent holdout-grid quality regression for the full k3 fit.

Truth generates observations and scores results; it is never optimizer input.
The zero-distortion seed has perturbed poses/intrinsics. Fixed-k3 and legacy
unregularized fits are test controls, not production selection branches.
"""
import json
import math
import random
from pathlib import Path
import generate_lm_fisheye as model
from lens_policy import feasible

ROOT=Path(__file__).resolve().parents[3]


def mapping(state,u,v,width,height):
    fx,fy=math.exp(state[0]),math.exp(state[1]);cx=state[2]*width;cy=state[3]*height
    x,y=(u-cx)/fx,(v-cy)/fy;t=x*x+y*y
    a,b,p,q,c=state[4:9];s=1+a*t+b*t*t+c*t*t*t
    return fx*(x*s+2*p*x*y+q*(t+2*x*x))+cx,fy*(y*s+p*(t+2*y*y)+2*q*x*y)+cy


def mapping_error(state,truth,width,height):
    errors={'center':[],'edge':[]}
    for iy in range(25):
        for ix in range(33):
            u,v=ix*(width-1)/32,iy*(height-1)/24
            a,b=mapping(state,u,v,width,height),mapping(truth,u,v,width,height)
            region='edge' if ix<4 or ix>28 or iy<3 or iy>21 else 'center'
            errors[region].append(sum((x-y)**2 for x,y in zip(a,b)))
    return {k:math.sqrt(sum(v)/len(v)) for k,v in errors.items()}


def main():
    reports=[]
    for kind in ('wide640','crop960','positive_k3'):
        model.WIDTH,model.HEIGHT=(640,480) if kind=='wide640' else (960,540)
        truth=model.truth_state()
        if kind!='wide640':
            truth[0],truth[1]=math.log(620),math.log(630)
            truth[2:9]=[.48,.52,-.26,.11,.002,-.001,-.035 if kind=='crop960' else .035]
        initial=truth[:];initial[:4]=[math.log(model.WIDTH)]*2+[.5,.5];initial[4:9]=[0.]*5
        for view in range(3):
            for axis in range(3):initial[9+6*view+axis]*=.7
            initial[12+6*view]+=.2;initial[13+6*view]-=.15;initial[14+6*view]+=.25
        for noise,seed in ((0.,414),(.3,414),(.3,495)):
            rng=random.Random(seed)
            observed=[(u+rng.gauss(0,noise),v+rng.gauss(0,noise)) for u,v in model.predict(truth)]
            for method in ('legacy_full','fixed_k3','regularized_full'):
                active=[i for i in range(27) if method!='fixed_k3' or i!=8]
                state,accepted,converged=model.optimize(initial,observed,active,lens_shape=method=='regularized_full')
                error=mapping_error(state,truth,model.WIDTH,model.HEIGHT)
                row=dict(case=kind,noise=noise,seed=seed,method=method,k3=state[8],truth_k3=truth[8],
                         rms=math.sqrt(model.residual(state,observed)[1]/120),mapping_rms_px=error,
                         accepted=accepted,converged=converged)
                reports.append(row)
                if method=='regularized_full':
                    assert converged and feasible(state,model.WIDTH,model.HEIGHT),row
                    assert abs(state[8]-truth[8])<(.015 if noise else 1e-5),row
                    assert max(error.values())<(.6 if noise else .001),row
                    print(json.dumps(row),flush=True)
    out=ROOT/'build/calibration_engine/lens_quality.json';out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(reports,indent=2)+'\n')
    print(f'LENS_QUALITY_PASS fits={len(reports)} (9 full-k3 cases; legacy/fixed-k3 controls)')


if __name__=='__main__':main()
