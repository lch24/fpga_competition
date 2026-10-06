// node generate_gauss_vectors.js
// Reference follows common/matrix.cpp's partial-pivot operation order.
// Dense test systems also carry a residual check against the ORIGINAL A,b.
function generateGaussVectors() {
    const buf=new ArrayBuffer(8),dv=new DataView(buf);
    const hex=x=>{dv.setFloat64(0,x);return dv.getBigUint64(0).toString(16).padStart(16,"0");};
    let seed=0x9e3779b9;
    const rng=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return seed>>>0;};
    function solve(A,b) {
        const n=A.length,M=A.map((r,i)=>[...r,b[i]]);
        if(M.some(r=>r.some(x=>!Number.isFinite(x))))return null;
        for(let c=0;c<n;c++){
            let p=c,max=Math.abs(M[c][c]);
            for(let r=c+1;r<n;r++)if(Math.abs(M[r][c])>max){max=Math.abs(M[r][c]);p=r;}
            let scale=0;for(let k=c;k<n;k++)scale=Math.max(scale,Math.abs(M[p][k]));
            if(scale<1e-30||max<scale*1e-14)return null;
            if(p!==c)for(let k=c;k<=n;k++){const t=M[c][k];M[c][k]=M[p][k];M[p][k]=t;}
            const pivot=M[c][c];
            if(!Number.isFinite(pivot)||Math.abs(pivot)<1e-30)return null;
            for(let r=c+1;r<n;r++){
                if(Math.abs(M[r][c])<1e-30)continue;
                const factor=M[r][c]/pivot;
                if(!Number.isFinite(factor))return null;
                for(let k=c;k<=n;k++){
                    const product=factor*M[c][k];
                    M[r][k]=M[r][k]-product;
                    if(!Number.isFinite(M[r][k]))return null;
                }
            }
        }
        const x=Array(n).fill(0);
        for(let r=n-1;r>=0;r--){
            const pivot=M[r][r];
            if(!Number.isFinite(pivot)||Math.abs(pivot)<1e-30)return null;
            let value=M[r][n];
            for(let c=r+1;c<n;c++){const product=M[r][c]*x[c];value=value-product;}
            x[r]=value/pivot;
            if(!Number.isFinite(x[r]))return null;
        }
        return x;
    }
    const cases=[];
    function add(name,A,b,checkResidual=true){
        const x=solve(A,b);
        if(x&&checkResidual){
            let worst=0;
            for(let r=0;r<A.length;r++){
                let sum=0,scale=Math.abs(b[r]);
                for(let c=0;c<A.length;c++){sum+=A[r][c]*x[c];scale+=Math.abs(A[r][c]*x[c]);}
                const rel=Math.abs(sum-b[r])/Math.max(scale,1e-300);
                worst=Math.max(worst,rel);
            }
            if(worst>1e-12)throw new Error("Reference residual too large: "+name+" "+worst);
        }
        cases.push({name,A,b,x});
    }
    for(let n=1;n<=26;n++){
        let A=Array.from({length:n},()=>Array.from({length:n},()=>Number(rng()%11)-5));
        for(let r=0;r<n;r++)A[r][r]=A[r].reduce((s,v,c)=>s+(r===c?0:Math.abs(v)),0)+3;
        const known=Array.from({length:n},(_,i)=>(i%7)-3);
        let b=A.map(row=>row.reduce((s,v,c)=>s+v*known[c],0));
        if(n%2===0){A=A.reverse();b=b.reverse();}
        add("dense_"+n+(n%2===0?"_reverse_rows":""),A,b);
    }
    add("swap",[[0,2],[3,4]],[-2,2]);
    add("equal_absolute_pivots",[[1,2],[-1,3]],[5,5]);
    add("singular_zero",[[0,0],[0,0]],[0,0]);
    add("singular_dependent",[[1,2,3],[2,4,6],[3,6,9]],[1,2,3]);
    add("relative_pivot_reject",[[1e-15,1],[0,1]],[1,1]);
    add("relative_pivot_boundary",[[1e-14,1],[0,1]],[2+1e-14,2]);
    add("absolute_scale_reject",[[1e-31]],[1e-31]);
    add("absolute_scale_boundary",[[1e-30]],[2e-30]);
    add("skip_tiny_elimination",[[1,1],[1e-31,1]],[3,2]);
    add("rhs_not_in_row_scale",[[1,0],[0,1]],[1e100,-1e100]);
    add("scaled_down",[[3e-20,1e-20],[1e-20,4e-20]],[5e-20,9e-20]);
    add("scaled_up",[[3e100,1e100],[1e100,4e100]],[5e100,9e100]);
    add("input_nan",[[NaN,0],[0,1]],[1,2]);
    add("input_infinity_rhs",[[1,0],[0,1]],[1,Infinity]);
    add("input_negative_infinity",[[1,0],[0,-Infinity]],[1,2]);
    add("elimination_overflow",[[1e308,-1e308],[1e308,1e308]],[0,1]);
    add("solution_overflow",[[1e-30]],[1e308]);
    add("underflow_is_not_failure",[[1e300]],[1e-100],false);
    add("negative_scalar",[[-2]],[6]);
    let vectors="",names="";
    cases.forEach((c,i)=>{
        const n=c.A.length;
        names+=(i+1)+": "+c.name+"\n";
        vectors+=n+" "+(c.x?0:4).toString(16)+"\n";
        for(let r=0;r<n;r++)for(const x of [...c.A[r],c.b[r]])vectors+=hex(x)+"\n";
        for(let j=0;j<26;j++)vectors+=hex(c.x&&j<n?c.x[j]:0)+"\n";
    });
    return {vectors,names,count:cases.length};
}
if(typeof module!=="undefined"&&require.main===module){
 const fs=require("fs"),path=require("path"),data=generateGaussVectors();
 fs.writeFileSync(path.join(__dirname,'../../calibration/',"gauss_vectors.txt"),data.vectors);
 fs.writeFileSync(path.join(__dirname,'../../calibration/',"gauss_cases.txt"),data.names);
}
