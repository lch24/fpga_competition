// Independent test oracle: exact BigInt rational rounding for basic operations;
// JavaScript Math for transcendental reference values. Run with Node.js:
// node generate_fp_vectors.js (writes vectors32.txt and vectors64.txt beside this file).
function generateVectors() {
  const buf=new ArrayBuffer(8), dv=new DataView(buf);
  const bits=(x,w)=>{if(w===32){dv.setFloat32(0,x);return BigInt(dv.getUint32(0));}
    dv.setFloat64(0,x);return dv.getBigUint64(0);};
  const number=(x,w)=>{if(w===32){dv.setUint32(0,Number(x));return dv.getFloat32(0);}
    dv.setBigUint64(0,x);return dv.getFloat64(0);};
  const len=x=>x===0n?0:x.toString(2).length;
  function decode(a,w) {
    const f=w===32?23:52,bias=w===32?127:1023,mask=(1n<<BigInt(f))-1n;
    const e=Number(a>>BigInt(f)&BigInt(2*bias+1)), frac=a&mask, s=Number(a>>BigInt(w-1));
    return {s,e,frac,f,bias,n:frac+(e>0?1n<<BigInt(f):0n),
      p:(e===0?1-bias:e-bias)-f, nan:e===2*bias+1&&frac!==0n,
      snan:e===2*bias+1&&frac!==0n&&(frac&(1n<<BigInt(f-1)))===0n,
      inf:e===2*bias+1&&frac===0n,zero:e===0&&frac===0n};
  }
  function round(s,n,d,p,w) {
    const f=w===32?23:52,bias=w===32?127:1023,emin=1-bias;
    const sign=BigInt(s)<<BigInt(w-1), inf=BigInt(2*bias+1)<<BigInt(f);
    if(n===0n)return [sign,0];
    let e=len(n)-len(d)+p;
    const cmp=(ex)=>ex>=0?n-(d<<BigInt(ex)):(n<<BigInt(-ex))-d;
    if(cmp(e-p)<0)e--;
    let t=p-Math.max(e,emin)+f, nn=n,dd=d;
    if(t>=0)nn<<=BigInt(t);else dd<<=BigInt(-t);
    let q=nn/dd,r=nn%dd,fl=r?16:0;
    if(2n*r>dd||(2n*r===dd&&(q&1n)))q++;
    if(q>=1n<<BigInt(f+1)){q>>=1n;e++;}
    if(e>bias)return [sign|inf,20];
    const sub=q<(1n<<BigInt(f));
    if(sub&&r)fl|=8;
    const exp=sub?0:Math.max(e,emin)+bias;
    return [sign|(BigInt(exp)<<BigInt(f))|(q&((1n<<BigInt(f))-1n)),fl];
  }
  function oracle(op,a,b,w) {
    let x=decode(a,w),y=decode(b,w);
    const f=x.f, inf=BigInt(2*x.bias+1)<<BigInt(f),
      nan=inf|(1n<<BigInt(f-1)), sign=s=>BigInt(s)<<BigInt(w-1);
    let result=0n,flags=0,cmp=0;
    if(op===11||op===12) {
      if(w!==64)return [nan,1,0,0];
      let src=op===11?decode(a&0xffffffffn,32):x, dest=op===11?64:32;
      const df=dest===64?52:23,di=dest===64?0x7ff0000000000000n:0x7f800000n;
      if(src.nan)return [di|(1n<<BigInt(df-1)),src.snan?1:0,0,0];
      if(src.inf)return [di|(BigInt(src.s)<<BigInt(dest-1)),0,0,0];
      [result,flags]=round(src.s,src.n,1n,src.p,dest);return [result,flags,0,0];
    }
    if(op===13){
      if(x.nan||y.nan)return [0n,(x.snan||y.snan)?1:0,4,0];
      const nx=number(a,w),ny=number(b,w);return [0n,0,(nx<ny?1:0)|(nx===ny?2:0),0];
    }
    const binary=op<=3||op===7;
    if(x.nan||(binary&&y.nan))return [nan,(x.snan||(binary&&y.snan))?1:0,0,0];
    if(op===0||op===1){
      if(op===1)y={...y,s:1-y.s};
      if(x.inf&&y.inf&&x.s!==y.s)return [nan,1,0,0];
      if(x.inf)return [sign(x.s)|inf,0,0,0];
      if(y.inf)return [sign(y.s)|inf,0,0,0];
      let p=Math.min(x.p,y.p);
      let z=(x.n<<BigInt(x.p-p))*(x.s?-1n:1n)+(y.n<<BigInt(y.p-p))*(y.s?-1n:1n);
      let s=z<0n?1:z>0n?0:(x.s&&y.s?1:0);
      [result,flags]=round(s,z<0n?-z:z,1n,p,w);
    }else if(op===2){
      if((x.inf&&y.zero)||(y.inf&&x.zero))return [nan,1,0,0];
      if(x.inf||y.inf)return [sign(x.s^y.s)|inf,0,0,0];
      [result,flags]=round(x.s^y.s,x.n*y.n,1n,x.p+y.p,w);
    }else if(op===3){
      if((x.inf&&y.inf)||(x.zero&&y.zero))return [nan,1,0,0];
      if(x.inf)return [sign(x.s^y.s)|inf,0,0,0];
      if(y.inf||x.zero)return [sign(x.s^y.s),0,0,0];
      if(y.zero)return [sign(x.s^y.s)|inf,2,0,0];
      [result,flags]=round(x.s^y.s,x.n,y.n,x.p-y.p,w);
    }else if(op===4){
      if(x.s&&!x.zero)return [nan,1,0,0];
      if(x.inf||x.zero)return [a,0,0,0];
      result=bits(Math.sqrt(number(a,w)),w);
      const z=decode(result,w), p=Math.min(x.p,2*z.p);
      flags=((x.n<<BigInt(x.p-p))===(z.n*z.n<<BigInt(2*z.p-p)))?0:16;
    }else if(op>=5&&op<=10){
      const nx=number(a,w),ny=number(b,w);
      const val=op===5?Math.sin(nx):op===6?Math.cos(nx):op===7?Math.atan2(nx,ny):
                op===8?Math.exp(nx):op===9?Math.log(nx):Math.acos(nx);
      // Some host atan2 implementations lose signed zero on underflow.
      // Mathematical atan2(y,x>0) must preserve the sign of nonzero y.
      result=bits((op===7&&val===0&&nx<0)?-0:val,w);
      if(Number.isNaN(val))return [nan,1,0,0];
      if(op===9&&x.zero)return [result,2,0,0];
      let exact=(op===5&&x.zero)||(op===6&&x.zero)||(op===8&&(x.zero||x.inf))||
                (op===9&&(nx===1||x.inf))||(op===10&&nx===1)||
                (op===7&&val===0&&(x.zero||y.inf));
      flags=exact?0:16;
      const z=decode(result,w);
      if(op===8&&!x.inf&&z.inf)flags=20;
      if(!exact&&z.e===0)flags|=8;
      // ULP allowance for approximation, separately enforced against sign/NaN/Inf.
      return [result,flags,0,8];
    }else return [nan,1,0,0];
    return [result,flags,cmp,0];
  }
  let seed=0x9e3779b9;
  const rand=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;};
  const out={};
  for(const w of [32,64]){
    const lines=[],hex=x=>x.toString(16).padStart(16,"0");
    const emit=(op,a,b=0n)=>{
      const [r,f,c,t]=oracle(op,a,b,w);
      lines.push([op,hex(a),hex(b),hex(r),f,c,t].map((x,i)=>typeof x==="number"?x.toString(16):x).join(" "));
    };
    const f=w===32?23:52,bias=w===32?127:1023,inf=BigInt(2*bias+1)<<BigInt(f),
      sign=1n<<BigInt(w-1),max=inf-1n,minnorm=1n<<BigInt(f);
    const special=[0n,sign,1n,sign|1n,minnorm-1n,minnorm,minnorm+1n,
      bits(1,w),bits(-1,w),bits(2,w),bits(3,w),max,max|sign,inf,inf|sign,
      inf|1n,inf|(1n<<BigInt(f-1)),bits(0.5,w),bits(-0.5,w)];
    for(const op of [0,1,2,3,13])for(const a of special)for(const b of special)emit(op,a,b);
    for(const a of special)for(const op of [4,5,6,8,9,10])emit(op,a);
    for(const a of special)for(const b of special)emit(7,a,b);
    for(let i=0;i<700;i++){
      const rb=()=>w===32?BigInt(rand()):((BigInt(rand())<<32n)|BigInt(rand()));
      const a=rb(),b=rb();
      for(const op of [0,1,2,3,4,13])emit(op,a,b);
      if(i<250){emit(5,a);emit(6,a);emit(7,a,b);emit(9,a&~sign);}
      if(w===64){emit(11,BigInt(rand()));emit(12,a);}
    }
    for(let i=0;i<400;i++){
      const u=rand()/0x100000000;
      emit(8,bits((u*2-1)*(w===32?110:800),w));
      emit(9,bits(Math.pow(2,(u*2-1)*(w===32?120:1000)),w));
      emit(10,bits(2*u-1,w));
      emit(7,bits((u*2-1)*100,w),bits((rand()/0x100000000*2-1)*100,w));
    }
    for(let i=-100;i<=100;i++){
      const a=bits(1,w)+BigInt(i);
      emit(9,a);if(i<=0)emit(10,a);
    }
    for(const x of [Math.PI,Math.PI/2,Math.PI*2,1e20,1e100,1e300,-1e300,
                    1e-10,1e-20,1e-100,1e-300,709.782712893384,-745.1332191019411]){
      emit(5,bits(x,w));emit(6,bits(x,w));emit(8,bits(x,w));
    }
    if(w===64)for(const a of [0n,1n,0x7f800000n,0xff800000n,0x7f800001n,0x7fc00001n,0x80000000n])emit(11,a);
    out[w]=lines.join("\n")+"\n";
  }
  return out;
}
if (typeof module!=="undefined" && require.main===module) {
  const fs=require("fs"),path=require("path");
  for(const [w,text] of Object.entries(generateVectors()))
    fs.writeFileSync(path.join(__dirname,"../../calibration/vectors"+w+".txt"),text);
}
