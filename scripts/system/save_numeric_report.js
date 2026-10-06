// Preserve compact, reviewable evidence from a completed numerical run.
const fs=require('fs'),path=require('path');
const root=path.resolve(__dirname,'../..'),dir=path.join(root,'build/system');
const j=JSON.parse(fs.readFileSync(path.join(dir,'numeric_comparison.json'),'utf8'));
const raw=fs.readFileSync(path.join(dir,'tb_vision_numeric.log'));
const log=raw.toString(raw[0]===255&&raw[1]===254?'utf16le':'utf8');
const m=log.match(/cycles=\s*(\d+) rms=([0-9a-f]{16})/);
if(!m||!log.includes('PASS vision numeric:')||j.pixelErrors)throw Error('Missing successful evidence');
j.cycles=Number(m[1]);j.rms=Buffer.from(m[2],'hex').readDoubleBE();
j.fixture='3 rendered 160x120 RGB565 views, 5x8 inner corners, fx=fy=150';j.algorithmMocks=false;
const counts=[0,0,0],lines=['view,index,x_fp32_hex,y_fp32_hex'];
for(const p of log.matchAll(/DETECTED view=\s*(\d+) x=([0-9a-f]+) y=([0-9a-f]+)/g)){
 const v=+p[1];lines.push([v,counts[v]++,p[2],p[3]].join(','));
}
if(counts.some(n=>n!==40))throw Error('Unexpected point counts: '+counts);
fs.mkdirSync(path.join(root,'data/system'),{recursive:true});
fs.writeFileSync(path.join(root,'data/system/numeric_160x120.json'),JSON.stringify(j,null,2)+'\n');
fs.writeFileSync(path.join(root,'data/system/detected_corners.csv'),lines.join('\n')+'\n');
console.log(j);
