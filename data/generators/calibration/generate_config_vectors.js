// Independent analytical pinhole fixture. No RTL outputs are used as references.
const fs=require('fs'),path=require('path');
const [dir,V,R,C]=process.argv.slice(2),views=+V,rows=+R,cols=+C;
if(!dir||views<3||views>16||rows<2||cols<2||rows*cols>256)throw Error('invalid geometry');
fs.mkdirSync(dir,{recursive:true});
function hex(x,n=8){const b=Buffer.alloc(n);n===8?b.writeDoubleBE(x):b.writeFloatBE(x);return b.toString('hex');}
function rotation(w){const a=Math.hypot(...w),u=w.map(x=>x/a),s=Math.sin(a),c=Math.cos(a),d=1-c;
 return [c+u[0]*u[0]*d,u[0]*u[1]*d-u[2]*s,u[0]*u[2]*d+u[1]*s,
 u[1]*u[0]*d+u[2]*s,c+u[1]*u[1]*d,u[1]*u[2]*d-u[0]*s,
 u[2]*u[0]*d-u[1]*s,u[2]*u[1]*d+u[0]*s,c+u[2]*u[2]*d];}
const state=[Math.log(800),Math.log(820),.5,.5,0,0,0,0,0],points=[],residual=[],poses=[],rms=[];
let cost=0,maxError=0;const depth=4*Math.max(rows,cols),square=2.5;
for(let v=0;v<views;v++){
 const rv=[-.32+.13*(v%5),.24-.11*(v%4),.02+.035*v],t=[-.5+.17*v,.3-.09*v,depth+.7*v];
 state.push(...rv,t[0],t[1],Math.log(t[2]));
 const rot=rotation(rv);let sum=0;
 for(let p=0;p<rows*cols;p++){
  const x=p%cols-(cols-1)/2,y=Math.floor(p/cols)-(rows-1)/2;
  const q=[0,1,2].map(i=>rot[3*i]*x+rot[3*i+1]*y+t[i]);
  const u=800*q[0]/q[2]+640,w=820*q[1]/q[2]+360;
  const a=Math.fround(u),b=Math.fround(w),du=u-a,dv=w-b;
  if(a<0||a>=1280||b<0||b>=720)throw Error('point outside image');
  points.push(hex(b,4)+hex(a,4));residual.push(du,dv);
  const d=Math.hypot(du,dv);sum+=d*d;cost+=du*du+dv*dv;maxError=Math.max(maxError,d);
 }
 rms.push(Math.sqrt(sum/(rows*cols)));poses.push(...rot,...t.map((x,i)=>(x-rot[3*i]*(cols-1)/2-rot[3*i+1]*(rows-1)/2)*square));
}
const metrics=[Math.sqrt(cost/(views*rows*cols)),...rms,maxError,...poses];
function write(name,values){fs.writeFileSync(path.join(dir,name),values.join('\n')+'\n');}
write('points.hex',points);write('state.hex',state.map(x=>hex(x)));write('residual.hex',residual.map(x=>hex(x)));write('metrics.hex',metrics.map(x=>hex(x)));write('cost.hex',[hex(cost)]);
fs.writeFileSync(path.join(dir,'fixture.json'),JSON.stringify({views,rows,cols,width:1280,height:720,square,depth,cost},null,2)+'\n');
console.log(`CONFIG_FIXTURE views=${views} board=${rows}x${cols} state=${state.length} residuals=${residual.length}`);
