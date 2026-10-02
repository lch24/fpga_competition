// Deterministic independent Number/Math reference; FP32 rounds every arithmetic operation.
(function generate(){
const fs=require('fs'),path=require('path'),base=__dirname;
function hex(x,w=64){let b=Buffer.alloc(w/8);if(w===32)b.writeFloatBE(x);else b.writeDoubleBE(x);return b.toString('hex')}
function pack(a,w=64){return a.slice().reverse().map(x=>hex(x,w)).join('')}
let seed=0x773ab421;function rand(){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed/4294967296}
function brown(a,w=64){let f=w===32?Math.fround:x=>x;let[x,y,k1,k2,k3,p1,p2]=a.map(f);let mul=(a,b)=>f(a*b),add=(a,b)=>f(a+b);let r2=add(mul(x,x),mul(y,y));let rad=add(add(add(1,mul(k1,r2)),mul(mul(k2,r2),r2)),mul(mul(mul(k3,r2),r2),r2));return[add(add(mul(x,rad),mul(mul(mul(2,p1),x),y)),mul(p2,add(r2,mul(mul(2,x),x)))),add(add(mul(y,rad),mul(p1,add(r2,mul(mul(2,y),y)))),mul(mul(mul(2,p2),x),y))]}
function rotation(v){let t2=v.reduce((s,x)=>s+x*x,0),t=Math.sqrt(t2),a=t2<1e-12?1-t2/6:Math.sin(t)/t,b=t2<1e-12?.5-t2/24:(1-Math.cos(t))/t2;let[x,y,z]=v,s=[0,-z,y,z,0,-x,-y,x,0],r=[];for(let i=0;i<3;i++)for(let j=0;j<3;j++){let ss=0;for(let k=0;k<3;k++)ss+=s[3*i+k]*s[3*k+j];r.push((i===j?1:0)+a*s[3*i+j]+b*ss)}return r}
function inverse(r){let q,s,tr=r[0]+r[4]+r[8];if(tr>0){s=2*Math.sqrt(tr+1);q=[s/4,(r[7]-r[5])/s,(r[2]-r[6])/s,(r[3]-r[1])/s]}else if(r[0]>r[4]&&r[0]>r[8]){s=2*Math.sqrt(1+r[0]-r[4]-r[8]);q=[(r[7]-r[5])/s,s/4,(r[1]+r[3])/s,(r[2]+r[6])/s]}else if(r[4]>r[8]){s=2*Math.sqrt(1+r[4]-r[0]-r[8]);q=[(r[2]-r[6])/s,(r[1]+r[3])/s,s/4,(r[5]+r[7])/s]}else{s=2*Math.sqrt(1+r[8]-r[0]-r[4]);q=[(r[3]-r[1])/s,(r[2]+r[6])/s,(r[5]+r[7])/s,s/4]}if(q[0]<0)q=q.map(x=>-x);let n=Math.sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]),scale=n>1e-12?2*Math.atan2(n,q[0])/n:2;return q.slice(1).map(x=>x*scale)}
function project(a){let[x,y]=a,r=a.slice(2,11),t=a.slice(11,14),k=a.slice(14,18),d=a.slice(18);let z=r[6]*x+r[7]*y+t[2];if(z<=1e-5)return [NaN,NaN];let b=brown([(r[0]*x+r[1]*y+t[0])/z,(r[3]*x+r[4]*y+t[1])/z,...d]);return[k[0]*b[0]+k[2],k[1]*b[1]+k[3]]}
const ident=[1,0,0,0,1,0,0,0,1];
let counts={};
for(let w of [32,64]){
let rows=[],add=a=>{a=a.map(x=>w===32?Math.fround(x):x);let out=brown(a,w),status=[...a,...out].every(Number.isFinite)?0:4;rows.push(status.toString(16)+' '+pack(a,w)+' '+pack(status?[0,0]:out,w));};
add([0,0,0,0,0,0,0]);add([.2,-.3,0,0,0,0,0]);for(let i=0;i<5;i++){let a=[.4,-.6,0,0,0,0,0];a[2+i]=.02;add(a)}
for(let i=0;i<100;i++)add([rand()*4-2,rand()*4-2,...Array.from({length:5},()=>rand()*.2-.1)]);
for(let i=0;i<7;i++)for(let val of [NaN,Infinity,-Infinity]){let a=[.1,.2,.01,.001,.0001,.02,.03];a[i]=val;add(a)}
add([w===32?3e38:1e308,1,1,1,1,1,1]);add([w===32?1.401298464324817e-45:5e-324,0,0,0,0,0,0]);
fs.writeFileSync(path.join(base,'brown'+w+'_vectors.txt'),rows.join('\n')+'\n');counts['brown'+w]=rows.length;
}
let rows=[],addRot=(mode,vec,r)=>{let valid=(mode?r:vec).every(Number.isFinite),out=mode?[...inverse(r),...r]:[...vec,...rotation(vec)];valid=valid&&out.every(Number.isFinite);rows.push((valid?'0':'4')+' '+mode.toString(16)+pack([...vec,...r])+' '+pack(valid?out:Array(12).fill(0)));};
for(let v of [[0,0,0],[1e-8,-2e-8,3e-8],[1e-6,0,0],[0,0,Math.PI/2],[Math.PI,0,0],[0,Math.PI,0],[0,0,Math.PI],[Math.PI-1e-9,0,0],[0,-Math.PI+1e-9,0],[2,2,1],[-2,-2,-1]]){
addRot(0,v,Array(9).fill(NaN));addRot(1,Array(3).fill(NaN),rotation(v));
}
for(let i=0;i<60;i++){let v=Array.from({length:3},()=>rand()*6-3);addRot(0,v,Array(9).fill(0));addRot(1,[0,0,0],rotation(v))}
for(let i=0;i<3;i++){let v=[0,0,0];v[i]=NaN;addRot(0,v,ident)}
for(let i=0;i<9;i++){let r=ident.slice();r[i]=Infinity;addRot(1,[0,0,0],r)}
addRot(0,[1e308,0,0],ident);
fs.writeFileSync(path.join(base,'rotation_vectors.txt'),rows.join('\n')+'\n');counts.rotation=rows.length;
rows=[];let addProj=a=>{let out=project(a),ok=[...a,...out].every(Number.isFinite);rows.push((ok?'0':'4')+' '+pack(a)+' '+pack(ok?out:[0,0]));};
let baseline=[1,2,...ident,0,0,10,800,810,320,240,0,0,0,0,0];
addProj(baseline);
for(let i=0;i<100;i++)addProj([rand()*7-3.5,rand()*4-2,...rotation([rand(),rand(),rand()]),rand(),rand(),5+rand()*15,300+rand()*700,300+rand()*700,320,240,...Array.from({length:5},()=>rand()*.02-.01)]);
for(let z of [-1,0,1e-5,1.0001e-5]){let a=baseline.slice();a[13]=z;addProj(a)}
for(let i=0;i<23;i++){let a=baseline.slice();a[i]=NaN;addProj(a)}
let a=baseline.slice();a[14]=1e308;a[11]=1e308;addProj(a);
fs.writeFileSync(path.join(base,'project_point_vectors.txt'),rows.join('\n')+'\n');counts.project_point=rows.length;
// Residual cases: explicit expected status/count, optional dropped read and malformed observations.
let cases=[];
function state(){return [Math.log(800),Math.log(820),.5,.5,.01,-.002,.0005,-.0008,.0001,.1,.2,-.1,.2,-.1,Math.log(12),-.2,.1,.3,-.3,.2,Math.log(15),.3,-.2,.1,.1,.4,Math.log(18)]}
function predictions(p,w,h){let out=[];for(let v=0;v<3;v++){let o=9+6*v,r=rotation(p.slice(o,o+3));for(let pt=0;pt<40;pt++)out.push(...project([pt%8-3.5,Math.floor(pt/8)-2,...r,p[o+3],p[o+4],Math.exp(p[o+5]),Math.exp(p[0]),Math.exp(p[1]),p[2]*w,p[3]*h,p[4],p[5],p[8],p[6],p[7]]));}return out}
function addRes(p,w=640,h=480,status=0,count=240,drop=-1,bad=-1){
let pred=predictions(p,w,h),obs=pred.map((x,i)=>Math.fround(Number.isFinite(x)?x+(i%7-3)*.03:0));if(bad>=0)obs[bad]=NaN;
let residual=pred.map((x,i)=>x-obs[i]),cost=0;for(let i=0;i<240;i+=2)cost+=residual[i]*residual[i]+residual[i+1]*residual[i+1];
cases.push([status,count,drop,w,h,pack(p),pack(obs,32),pack(residual.map(x=>Number.isFinite(x)?x:0)),hex(status?Infinity:cost)].join(' '));
}
addRes(state());for(let i=0;i<4;i++){let p=state();p[2]=.2+rand()*.6;p[3]=.2+rand()*.6;for(let v=0;v<3;v++)for(let j=0;j<3;j++)p[9+6*v+j]=rand()-.5;addRes(p,i%2?1920:65535,i%2?1080:2)}
let p=Array(27).fill(0);p[2]=p[3]=.5;p[14]=p[20]=p[26]=Math.log(8);addRes(p);
for(let i=0;i<27;i++){let p=state();p[i]=NaN;addRes(p,640,480,4,0)}
addRes(state(),1,480,1,0);addRes(state(),640,1,1,0);
for(let idx of [0,1])for(let log of [Math.log(.0001),Math.log(1e8),1000,-1000]){let p=state();p[idx]=log;addRes(p,640,480,4,0)}
for(let drop of [0,45])addRes(state(),640,480,5,drop*2,drop);
for(let bad of [0,91])addRes(state(),640,480,4,Math.floor(bad/2)*2,-1,bad);
p=state();p[14]=-1000;p[9]=p[10]=p[11]=0;addRes(p,640,480,4,0);
p=state();p[26]=1000;addRes(p,640,480,4,160);
fs.writeFileSync(path.join(base,'residual_engine_vectors.txt'),cases.join('\n')+'\n');counts.residual_engine=cases.length;
fs.writeFileSync(path.join(base,'model_vector_counts.json'),JSON.stringify(counts,null,2)+'\n');
console.log(counts);
})();
