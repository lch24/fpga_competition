// Independent pinhole-plane renderer, supersampled 4x4 into RGB565.
// No RTL outputs and no detected points are used to generate these images.
const fs=require('fs'),path=require('path');
const out=path.join(__dirname,'../build');fs.mkdirSync(out,{recursive:true});
const W=160,H=120,fx=150,fy=150,cx=80,cy=60;
function rotation(rx,ry,rz){const a=Math.hypot(rx,ry,rz),v=[rx/a,ry/a,rz/a],c=Math.cos(a),s=Math.sin(a),d=1-c;
 return [c+v[0]*v[0]*d,v[0]*v[1]*d-v[2]*s,v[0]*v[2]*d+v[1]*s,
 v[1]*v[0]*d+v[2]*s,c+v[1]*v[1]*d,v[1]*v[2]*d-v[0]*s,
 v[2]*v[0]*d-v[1]*s,v[2]*v[1]*d+v[0]*s,c+v[2]*v[2]*d];}
const poses=[[.25,-.30,.08,.1,.1,12],[-.32,.20,-.12,-.2,.2,13],[.12,.38,.16,.2,-.15,11.8]];
const all=[];
for(const p of poses){const R=rotation(...p),t=p.slice(3);
 for(let y=0;y<H;y++)for(let x=0;x<W;x++){
  let sum=0;
  for(let sy=0;sy<4;sy++)for(let sx=0;sx<4;sx++){
   const ray=[(x+(sx+.5)/4-.5-cx)/fx,(y+(sy+.5)/4-.5-cy)/fy,1];
   const n=[R[2],R[5],R[8]],z=(n[0]*t[0]+n[1]*t[1]+n[2]*t[2])/(n[0]*ray[0]+n[1]*ray[1]+n[2]);
   const q=ray.map((v,i)=>z*v-t[i]);
   const bx=R[0]*q[0]+R[3]*q[1]+R[6]*q[2]+3.5;
   const by=R[1]*q[0]+R[4]*q[1]+R[7]*q[2]+2;
   sum+=bx>=-1&&bx<8&&by>=-1&&by<5?((Math.floor(bx)+Math.floor(by))&1?224:32):144;
  }
  const v=Math.round(sum/16),pixel=((v>>3)<<11)|((v>>2)<<5)|(v>>3);
  all.push(pixel.toString(16).padStart(4,'0'));
 }
}
fs.writeFileSync(path.join(out,'board_images.hex'),all.join('\n')+'\n');
fs.writeFileSync(path.join(out,'board_fixture.json'),JSON.stringify({width:W,height:H,fx,fy,cx,cy,rows:5,cols:8,poses},null,2));
console.log(`Rendered ${poses.length} independent ${W}x${H} calibration images.`);
