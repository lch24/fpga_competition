// node generate_eigen_vectors.js
// Independent fixtures: A=Q*D*Q^T, known eigenvalues and eigenvectors,
// rather than copying the Jacobi iteration under test.
function generateEigenVectors(){
 const dv=new DataView(new ArrayBuffer(8));
 const hex=x=>{dv.setFloat64(0,x);return dv.getBigUint64(0).toString(16).padStart(16,"0");};
 let seed=0x51f0a981;const rng=()=>{seed^=seed<<13;seed^=seed>>>17;seed^=seed<<5;return (seed>>>0)/4294967296;};
 const all=[];
 const eye=n=>Array.from({length:n},(_,r)=>Array.from({length:n},(_,c)=>+(r===c)));
 function add(name,A,D,Q,compare=true,rot=-1,status=0){
   const order=D.map((_,i)=>i).sort((a,b)=>D[a]-D[b]);
   all.push({name,A,status,compare:compare?1:0,rot,
     values:[D[order[0]],D[order[1]],D[order[D.length-1]]],
     vec:Q.map(row=>row[order[0]])});
 }
 function diag(name,D){const n=D.length;add(name,D.map((v,r)=>D.map((_,c)=>r===c?v:0)),D,eye(n),true,0);}
 function dense(name,D,scale=1,compare=true){
   const n=D.length,Q=eye(n);
   for(let k=0;k<4*n;k++){
     const p=Math.floor(rng()*n),q=(p+1+Math.floor(rng()*(n-1)))%n,ang=(rng()*2-1)*2;
     const c=Math.cos(ang),s=Math.sin(ang);
     for(let r=0;r<n;r++){const a=Q[r][p],b=Q[r][q];Q[r][p]=c*a-s*b;Q[r][q]=s*a+c*b;}
   }
   D=D.map(x=>x*scale);
   const A=Array.from({length:n},()=>Array(n).fill(0));
   for(let r=0;r<n;r++)for(let c=r;c<n;c++){
     let sum=0;for(let k=0;k<n;k++)sum+=Q[r][k]*D[k]*Q[c][k];
     A[r][c]=sum;A[c][r]=sum;
   }
   add(name,A,D,Q,compare);
 }
 for(const n of [6,9]){
   diag("zero_"+n,Array(n).fill(0));
   diag("identity_"+n,Array(n).fill(1));
   diag("unsorted_diagonal_"+n,Array.from({length:n},(_,i)=>n-i));
   diag("negative_diagonal_"+n,Array.from({length:n},(_,i)=>-i-1));
   const D=Array.from({length:n},(_,i)=>i+1);
   dense("positive_dense_"+n,D);
   dense("indefinite_dense_"+n,D.map(x=>x-n/2));
   dense("rank_deficient_"+n,D.map(x=>x-1));
   dense("repeated_minimum_"+n,D.map((x,i)=>i<2?-2:x),1,false);
   dense("clustered_"+n,D.map((x,i)=>i<3?1+i*1e-8:x),1,false);
   dense("scaled_small_"+n,D,1e-20);
   dense("scaled_large_"+n,D,1e100);
   dense("below_scale_floor_"+n,D,1e-40,false);
   // An embedded 2x2 block has exact known eigenvalues 1 and 3.
   const A=eye(n).map((row,r)=>row.map(v=>v*(r+4))),Q=eye(n);
   A[0][0]=2;A[1][1]=2;A[0][1]=A[1][0]=1;
   Q[0][0]=Math.SQRT1_2;Q[1][0]=-Math.SQRT1_2;
   Q[0][1]=Math.SQRT1_2;Q[1][1]=Math.SQRT1_2;
   add("block_rotation_"+n,A,[1,3,...Array.from({length:n-2},(_,i)=>i+6)],Q,true,1);
 }
 const invalid=(name,kind,status)=>{
   const A=eye(6);
   if(kind===0)A[0][0]=NaN;
   if(kind===1)A[2][3]=A[3][2]=Infinity;
   if(kind===2)A[1][0]=0.125;
   if(kind===3){A[0][0]=A[1][1]=1e308;A[0][1]=A[1][0]=1e308;}
   add(name,A,Array(6).fill(0),eye(6),false,-1,status);
 };
 invalid("nan",0,4);invalid("infinity",1,4);invalid("asymmetric",2,1);invalid("angle_overflow",3,4);
 let vectors="",names="";
 all.forEach((t,i)=>{
   names+=(i+1)+": "+t.name+"\n";
   vectors+=t.A.length+" "+t.status+" "+t.compare+" "+t.rot+"\n";
   for(const row of t.A)for(const x of row)vectors+=hex(x)+"\n";
   for(const x of t.values)vectors+=hex(x)+"\n";
   for(let j=0;j<9;j++)vectors+=hex(j<t.vec.length?t.vec[j]:0)+"\n";
 });
 return {vectors,names,count:all.length};
}
if(typeof module!=="undefined"&&require.main===module){
 const fs=require("fs"),path=require("path"),v=generateEigenVectors();
 fs.writeFileSync(path.join(__dirname,'../../calibration/',"eigen_vectors.txt"),v.vectors);
 fs.writeFileSync(path.join(__dirname,'../../calibration/',"eigen_cases.txt"),v.names);
}
