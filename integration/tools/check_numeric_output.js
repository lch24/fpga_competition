// Independent software Brown mapping + integer bilinear reference for every
// output pixel. Uses the published camera packet, and separately checks that
// the estimated focal length agrees with the independent rendering fixture.
const fs=require('fs'),path=require('path');
const dir=path.resolve(process.argv[2]||path.join(__dirname,'../build'));
const words=name=>fs.readFileSync(path.join(dir,name),'utf8').trim().split(/\s+/).map(x=>parseInt(x,16));
const decode=u=>{const b=Buffer.alloc(4);b.writeUInt32LE(u);return b.readFloatLE();};
const params=words('numeric_camera.hex').map(decode),input=words('board_images.hex'),bytes=words('numeric_output.hex');
const cfg=JSON.parse(fs.readFileSync(path.join(dir,'board_fixture.json'),'utf8'));
const {width:W,height:H}=cfg,[fx,fy,cx,cy,k1,k2,k3,p1,p2]=params;
if(params.length!==9||params.some(x=>!Number.isFinite(x))||bytes.length!==W*H*2)throw Error('Invalid result files');
const f=Math.fround,add=(a,b)=>f(a+b),mul=(a,b)=>f(a*b);
function map(x,y){
 const nx=f(f(x-cx)/fx),ny=f(f(y-cy)/fy),x2=mul(nx,nx),y2=mul(ny,ny),r2=add(x2,y2),r4=mul(r2,r2),r6=mul(r4,r2);
 const radial=add(add(add(1,mul(k1,r2)),mul(k2,r4)),mul(k3,r6)),xy=mul(nx,ny);
 const xd=add(add(mul(nx,radial),add(mul(p1,xy),mul(p1,xy))),mul(p2,add(r2,add(x2,x2))));
 const yd=add(add(mul(ny,radial),mul(p1,add(r2,add(y2,y2)))),add(mul(p2,xy),mul(p2,xy)));
 return [add(mul(fx,xd),cx),add(mul(fy,yd),cy)];
}
function at(x,y){return x<0||y<0||x>=W||y>=H?0:input[2*W*H+y*W+x];}
function sample(x,y){
 const qx=Math.floor(x*65536),qy=Math.floor(y*65536),ix=Math.floor(qx/65536),iy=Math.floor(qy/65536),dx=qx-ix*65536,dy=qy-iy*65536;
 const ps=[at(ix,iy),at(ix+1,iy),at(ix,iy+1),at(ix+1,iy+1)],ws=[(65536-dx)*(65536-dy),dx*(65536-dy),(65536-dx)*dy,dx*dy];
 const ch=(shift,bits)=>Math.floor((ps.reduce((sum,p,i)=>{let v=(p>>shift)&((1<<bits)-1);v=(v<<(8-bits))|(v>>(2*bits-8));return sum+v*ws[i];},0)+2147483648)/4294967296);
 return ((ch(11,5)>>3)<<11)|((ch(5,6)>>2)<<5)|(ch(0,5)>>3);
}
let errors=0;const examples=[];
for(let y=0;y<H;y++)for(let x=0;x<W;x++){
 const expected=sample(...map(x,y)),i=(y*W+x)*2,actual=bytes[i]|(bytes[i+1]<<8);
 if(actual!==expected){errors++;if(examples.length<8)examples.push({x,y,actual,expected});}
}
const focalRelativeError=Math.max(Math.abs(fx/cfg.fx-1),Math.abs(fy/cfg.fy-1));
const result={pixels:W*H,pixelErrors:errors,examples,params,focalRelativeError};
fs.writeFileSync(path.join(dir,'numeric_comparison.json'),JSON.stringify(result,null,2)+'\n');
console.log(JSON.stringify(result,null,2));
if(errors||focalRelativeError>.2||Math.abs(cx-cfg.cx)>W*.2||Math.abs(cy-cfg.cy)>H*.2)process.exit(1);
console.log('NUMERIC_IMAGE_COMPARE_PASS');
