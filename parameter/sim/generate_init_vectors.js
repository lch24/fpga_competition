// Deterministic independent software reference, translated from initialization.cpp/pose_init.cpp.
// No RTL state machine or RTL simulation outputs are used to generate expected results.
const fs=require('fs'),path=require('path');
const hex=(x,w=64)=>{const b=Buffer.alloc(w/8);w===64?b.writeDoubleBE(x):b.writeFloatBE(x);return b.toString('hex')};
const pack=(a,w=64)=>a.slice().reverse().map(x=>hex(x,w)).join('');
const matmul=(a,b)=>Array.from({length:9},(_,i)=>{let s=0;for(let k=0;k<3;k++)s+=a[3*Math.floor(i/3)+k]*b[3*k+i%3];return s});
function eigen(input,n){
 let a=input.slice(),v=Array.from({length:n*n},(_,i)=>+(Math.floor(i/n)===i%n));
 for(let it=0;it<100*n*n;it++){
  let p=0,q=1,largest=0,diagonal=0;
  for(let i=0;i<n;i++){diagonal=Math.max(diagonal,Math.abs(a[i*n+i]));for(let j=i+1;j<n;j++)if(Math.abs(a[i*n+j])>largest){largest=Math.abs(a[i*n+j]);p=i;q=j;}}
  if(largest<=1e-14*Math.max(diagonal,1e-30)){let order=Array.from({length:n},(_,i)=>i).sort((i,j)=>a[i*n+i]-a[j*n+j]);return {values:order.map(i=>a[i*n+i]),vector:Array.from({length:n},(_,i)=>v[i*n+order[0]])};}
  let phi=.5*Math.atan2(2*a[p*n+q],a[q*n+q]-a[p*n+p]),c=Math.cos(phi),s=Math.sin(phi),app=a[p*n+p],aqq=a[q*n+q],apq=a[p*n+q];
  for(let k=0;k<n;k++)if(k!==p&&k!==q){let akp=a[k*n+p],akq=a[k*n+q];a[k*n+p]=a[p*n+k]=c*akp-s*akq;a[k*n+q]=a[q*n+k]=s*akp+c*akq;}
  a[p*n+p]=c*c*app-2*s*c*apq+s*s*aqq;a[q*n+q]=s*s*app+2*s*c*apq+c*c*aqq;a[p*n+q]=a[q*n+p]=0;
  for(let k=0;k<n;k++){let x=v[k*n+p],y=v[k*n+q];v[k*n+p]=c*x-s*y;v[k*n+q]=s*x+c*y;}
 }return null;
}
function outer(a,row){for(let i=0;i<row.length;i++)for(let j=0;j<row.length;j++)a[i*row.length+j]+=row[i]*row[j];}
function homography(points){
 let mx=0,my=0,di=0,dw=0;for(let i=0;i<40;i++){mx+=points[2*i];my+=points[2*i+1];}mx/=40;my/=40;
 for(let i=0;i<40;i++){di+=Math.hypot(points[2*i]-mx,points[2*i+1]-my);dw+=Math.hypot(i%8-3.5,Math.floor(i/8)-2);}
 if(di<1e-6||dw<1e-6)return null;
 let si=Math.SQRT2*40/di,sw=Math.SQRT2*40/dw,a=Array(81).fill(0);
 for(let i=0;i<40;i++){let x=(i%8-3.5)*sw,y=(Math.floor(i/8)-2)*sw,u=(points[2*i]-mx)*si,v=(points[2*i+1]-my)*si;outer(a,[-x,-y,-1,0,0,0,u*x,u*y,u]);outer(a,[0,0,0,-x,-y,-1,v*x,v*y,v]);}
 let e=eigen(a,9);if(!e||e.values[1]<e.values[8]*1e-10)return null;
 let h=matmul(matmul([1/si,0,mx,0,1/si,my,0,0,1],e.vector),[sw,0,0,0,sw,0,0,0,1]);if(Math.abs(h[8])<1e-12)return null;return h.map(x=>x/h[8]);
}
function zhang(hom,w,h){
 let ata=Array(36).fill(0),t=[1/w,0,-.5,0,1/w,-h/(2*w),0,0,1];
 const vij=(a,i,j)=>[a[i]*a[j],a[i]*a[3+j]+a[3+i]*a[j],a[3+i]*a[3+j],a[6+i]*a[j]+a[i]*a[6+j],a[6+i]*a[3+j]+a[3+i]*a[6+j],a[6+i]*a[6+j]];
 for(let v=0;v<3;v++){let a=matmul(t,hom.slice(v*9,v*9+9)),v12=vij(a,0,1),v11=vij(a,0,0),v22=vij(a,1,1);outer(ata,v12);outer(ata,v11.map((x,i)=>x-v22[i]));}
 let e=eigen(ata,6);if(!e)return null;let b=e.vector;if(b[0]<0)b=b.map(x=>-x);let d=b[0]*b[2]-b[1]*b[1];if(b[0]<=0||d<=1e-14)return null;
 let cy=(b[1]*b[3]-b[0]*b[4])/d,lambda=b[5]-(b[3]*b[3]+cy*(b[1]*b[3]-b[0]*b[4]))/b[0];if(lambda<=0)return null;
 let fx=Math.sqrt(lambda/b[0]),fy=Math.sqrt(lambda*b[0]/d),skew=-b[1]*fx*fx*fy/lambda,cx=skew*cy/fy-b[3]*fx*fx/lambda;
 let k=[fx*w,fy*w,cx*w+w*.5,cy*w+h*.5];return k.every(Number.isFinite)&&k[0]>.05*w&&k[1]>.05*w&&k[0]<20*w&&k[1]<20*w&&Math.abs(k[2]-w*.5)<w&&Math.abs(k[3]-h*.5)<h?k:null;
}
function rotation(v){let t2=v[0]*v[0]+v[1]*v[1]+v[2]*v[2],t=Math.sqrt(t2),a=t2<1e-12?1-t2/6:Math.sin(t)/t,b=t2<1e-12?.5-t2/24:(1-Math.cos(t))/t2;let[x,y,z]=v,s=[0,-z,y,z,0,-x,-y,x,0],ss=matmul(s,s);return s.map((x,i)=>(i%4===0?1:0)+a*x+b*ss[i]);}
function inverse(r){let q,s,tr=r[0]+r[4]+r[8];if(tr>0){s=2*Math.sqrt(tr+1);q=[s/4,(r[7]-r[5])/s,(r[2]-r[6])/s,(r[3]-r[1])/s]}else if(r[0]>r[4]&&r[0]>r[8]){s=2*Math.sqrt(1+r[0]-r[4]-r[8]);q=[(r[7]-r[5])/s,s/4,(r[1]+r[3])/s,(r[2]+r[6])/s]}else if(r[4]>r[8]){s=2*Math.sqrt(1+r[4]-r[0]-r[8]);q=[(r[2]-r[6])/s,(r[1]+r[3])/s,s/4,(r[5]+r[7])/s]}else{s=2*Math.sqrt(1+r[8]-r[0]-r[4]);q=[(r[3]-r[1])/s,(r[2]+r[6])/s,(r[5]+r[7])/s,s/4]}if(q[0]<0)q=q.map(x=>-x);let n=Math.sqrt(q[1]*q[1]+q[2]*q[2]+q[3]*q[3]),scale=n>1e-12?2*Math.atan2(n,q[0])/n:2;return q.slice(1).map(x=>x*scale)}
const dot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
function pose(hom,k,w,h){
 if(k[0]<=0||k[1]<=0||![...hom,...k].every(Number.isFinite))return null;
 let p=[Math.log(k[0]),Math.log(k[1]),k[2]/w,k[3]/h,0,0,0,0,0];
 for(let i=0;i<3;i++){
  let a=hom.slice(i*9,i*9+9),kinv=c=>[(a[c]-k[2]*a[6+c])/k[0],(a[3+c]-k[3]*a[6+c])/k[1],a[6+c]],v1=kinv(0),v2=kinv(1),t=kinv(2),n1=Math.sqrt(dot(v1,v1)),n2=Math.sqrt(dot(v2,v2));if(n1<1e-12||n2<1e-12)return null;
  let scale=2/(n1+n2);if(t[2]<0)scale=-scale;t=t.map(x=>x*scale);let r1=v1.map(x=>x*(scale>0?1/n1:-1/n1)),parallel=dot(r1,v2);v2=v2.map((x,j)=>x-parallel*r1[j]);let norm=Math.sqrt(dot(v2,v2));if(norm<1e-12||t[2]<=0)return null;
  let r2=v2.map(x=>x*(scale>0?1/norm:-1/norm)),r3=r1.map((_,c)=>r1[(c+1)%3]*r2[(c+2)%3]-r1[(c+2)%3]*r2[(c+1)%3]),r=[r1[0],r2[0],r3[0],r1[1],r2[1],r3[1],r1[2],r2[2],r3[2]];
  p.push(...inverse(r),t[0],t[1],Math.log(t[2]));
 }return p.every(Number.isFinite)?p:null;
}
function cameraH(k,rv,t){let r=rotation(rv);return matmul([k[0],0,k[2],0,k[1],k[3],0,0,1],[r[0],r[1],t[0],r[3],r[4],t[1],r[6],r[7],t[2]]).map(x=>x/t[2]);}
function observations(h){let p=[];for(let i=0;i<40;i++){let x=i%8-3.5,y=Math.floor(i/8)-2,z=h[6]*x+h[7]*y+h[8];p.push(Math.fround((h[0]*x+h[1]*y+h[2])/z),Math.fround((h[3]*x+h[4]*y+h[5])/z));}return p;}
let random=0x883149ab;const rng=()=>{random=(Math.imul(random,1664525)+1013904223)>>>0;return random/4294967296;};
function run(){
 const lists={homography:[],zhang:[],pose_init:[],init_controller:[]},names={};for(let n in lists)names[n]=[];
 function add(n,name,row){lists[n].push(row);names[n].push(`${lists[n].length}: ${name}`);}
 let k=[800,820,320,240],hom=[cameraH(k,[.2,-.3,.1],[.3,-.2,14]),cameraH(k,[-.3,.1,-.15],[-.2,.3,16]),cameraH(k,[.1,.4,.2],[.1,.2,18])].flat(),pts=observations(hom.slice(0,9));
 function hcase(name,points=pts,w=640,h=480,view=0,drop=-1,override){let result=homography(points),status=w<2||h<2||view>2?1:drop>=0?5:!points.every((x,i)=>Number.isFinite(x)&&x>=0&&x<(i%2?h:w))||!result?4:0;if(override!==undefined)status=override;add('homography',name,[status,w,h,view,drop,pack(points,32),pack(status?Array(9).fill(0):result)].join(' '));}
 hcase('perspective');hcase('affine',observations([35,2,320,-1,38,240,0,0,1]),640,480,2);
 for(let i=0;i<8;i++)hcase('random_'+i,observations(cameraH(k,[rng()-.5,rng()-.5,rng()-.5],[rng()-.5,rng()-.5,14+rng()*8])),640,480,i%3);
 hcase('constant',Array(80).fill(200));hcase('collinear',Array.from({length:80},(_,i)=>i%2?240:100+Math.floor(i/2)*5));
 for(let [name,value,index] of [['nan',NaN,0],['inf',Infinity,79],['negative',-.01,31],['x_boundary',640,0],['y_boundary',480,1]]){let p=pts.slice();p[index]=value;hcase(name,p);}
 hcase('bad_width',pts,1);hcase('bad_height',pts,640,1);hcase('bad_view',pts,640,480,3);hcase('missing_first',pts,640,480,0,0);hcase('missing_last',pts,640,480,1,39);
 function zcase(name,a=hom,w=640,h=480){let result=a.every(Number.isFinite)?zhang(a,w,h):null,status=w<2||h<2?1:result?0:4;add('zhang',name,[status,w,h,pack(a),pack(status?Array(4).fill(0):result)].join(' '));}
 zcase('physical');for(let i=0;i<8;i++){let kk=[400+rng()*700,400+rng()*700,250+rng()*140,180+rng()*120];zcase('random_'+i,Array.from({length:3},()=>cameraH(kk,[rng()-.5,rng()-.5,rng()-.5],[rng()-.5,rng()-.5,12+rng()*10])).flat());}
 zcase('zero',Array(27).fill(0));zcase('front_parallel',Array.from({length:3},(_,i)=>cameraH(k,[0,0,.1*i],[0,0,12+i])).flat());
 for(let i of [0,13,26]){let a=hom.slice();a[i]=NaN;zcase('nan_'+i,a);}zcase('bad_width',hom,1);zcase('bad_height',hom,640,1);
 function pcase(name,a=hom,kk=k,w=640,h=480,forced){let result=pose(a,kk,w,h),status=w<2||h<2?1:result?0:4;if(forced!==undefined)status=forced;add('pose_init',name,[status,w,h,pack(a),pack(kk),pack(status?Array(27).fill(0):result)].join(' '));}
 pcase('physical');pcase('negative_H',hom.map(x=>-x));pcase('scaled_H',hom.map((x,i)=>x*[.01,10,-2][Math.floor(i/9)]));
 for(let i=0;i<12;i++)pcase('random_'+i,Array.from({length:3},()=>cameraH(k,[rng()*4-2,rng()*4-2,rng()*4-2],[rng()-.5,rng()-.5,10+rng()*10])).flat());
 for(let i=0;i<27;i++){let a=hom.slice();a[i]=NaN;pcase('nan_H_'+i,a);}
 for(let i=0;i<4;i++){let kk=k.slice();kk[i]=Infinity;pcase('inf_K_'+i,hom,kk);}
 for(let focal of [0,-800]){let kk=k.slice();kk[0]=focal;pcase('invalid_fx_'+focal,hom,kk);}
 for(let which of ['v1','v2','parallel','tz']){let a=hom.slice();if(which==='tz')a[8]=0;else for(let r=0;r<3;r++)a[3*r+(which==='v1'?0:1)]=which==='parallel'?a[3*r]:0;pcase(which,a);}
 pcase('bad_width',hom,k,1);pcase('bad_height',hom,k,640,1);
 // Controller vectors carry expected states by seed ID. Forced child errors exercise skip/all-failed policy.
 function icase(name,points,w=640,h=480,mode=0){
  let hs=[],status=w<2||h<2?1:0,seeds=[];if(!status){for(let v=0;v<3;v++){let ps=points.slice(v*80,v*80+80),hh=ps.every((x,i)=>x>=0&&x<(i%2?h:w))?homography(ps):null;if(!hh){status=4;break;}hs.push(...hh);}}
  if(!status){let diversity=0;for(let v=1;v<3;v++)for(let j=0;j<8;j++)diversity+=Math.abs(hs[v*9+j]-hs[j]);if(diversity<1e-6)status=4;}
  if(!status){let zk=zhang(hs,w,h);for(let id=0;id<5;id++){let kk=id===0?zk:[w*[.6,1,1.8,3][id-1],w*[.6,1,1.8,3][id-1],(w-1)*.5,(h-1)*.5];if(mode===1&&id===0)kk=null;let p=kk?pose(hs,kk,w,h):null;if(mode===2&&id===2||mode===3)p=null;if(p)seeds.push({id,p});}if(!seeds.length)status=4;}
  add('init_controller',name,[status,w,h,mode,seeds.length,pack(points,32),...Array.from({length:5},(_,id)=>pack((seeds.find(s=>s.id===id)||{p:Array(27).fill(0)}).p)),seeds.reduce((s,x)=>s|(1<<x.id),0)].join(' '));
 }
 let allpts=Array.from({length:3},(_,i)=>observations(hom.slice(i*9,i*9+9))).flat();
 icase('full_pipeline',allpts);icase('zhang_forced_failure',allpts,640,480,1);icase('pose_seed2_forced_failure',allpts,640,480,2);icase('all_pose_forced_failure',allpts,640,480,3);
 icase('all_repeated',[...pts,...pts,...pts]);icase('two_repeated',[...pts,...pts,...allpts.slice(160)]);icase('degenerate_view',[...pts,...Array(80).fill(200),...pts]);icase('bad_width',allpts,1);
 let front=Array.from({length:3},(_,i)=>observations(cameraH(k,[0,0,0],[.2*i,0,12+2*i]))).flat();icase('natural_zhang_failure',front);
 for(let name in lists){fs.writeFileSync(path.join(__dirname,name+'_vectors.txt'),lists[name].join('\n')+'\n');fs.writeFileSync(path.join(__dirname,name+'_cases.txt'),names[name].join('\n')+'\n');}
 let counts=Object.fromEntries(Object.entries(lists).map(([k,v])=>[k,v.length]));fs.writeFileSync(path.join(__dirname,'init_vector_counts.json'),JSON.stringify(counts,null,2)+'\n');console.log(counts);
}
if(require.main===module)run();
module.exports={homography,zhang,pose,cameraH,observations};
