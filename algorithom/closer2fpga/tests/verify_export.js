// Independently parse the exported text and reconstruct IEEE bits with Node buffers.
const fs=require('fs'), path=require('path'), assert=require('assert');
const dirs=fs.readFileSync('export_fixture_paths.txt','utf8').trim().split(/\r?\n/);
assert.equal(dirs.length,4);assert.equal(new Set(dirs).size,4);
let checks=0;
function pair(value,hex,size) {
  assert.match(hex,size===4?/^[0-9a-f]{8}$/:/^[0-9a-f]{16}$/);
  const b=Buffer.from(hex,'hex'), x=size===4?b.readFloatBE():b.readDoubleBE();
  assert.ok(Number.isFinite(x)?Object.is(value,x):value===null);++checks;
}
dirs.forEach((dir,caseId)=>{
  assert.equal(fs.readFileSync(path.join(dir,'COMPLETE.txt'),'utf8').split(/\r?\n/)[0],'closer2fpga.calibration.v1');
  assert.ok(!fs.existsSync(path.join(dir,'COMPLETE.tmp')));
  const j=JSON.parse(fs.readFileSync(path.join(dir,'calibration.json'),'utf8'));
  assert.equal(j.format,'closer2fpga.calibration.v1');
  assert.equal(j.rtl_input_ready,caseId<2);
  assert.equal(j.algorithm,"single_seed_schur_hybrid_lm_v2");
  assert.equal(j.rtl_algorithm_matches,false);
  assert.equal(j.max_iterations_per_stage,60);
  assert.equal(j.calibration_attempted,caseId!==2);
  assert.equal(j.views[0].path,'E:/测试/quoted"name\\line\n.jpg');
  pair(j.square_size,j.square_size_fp64_hex,8);
  const lines=fs.readFileSync(path.join(dir,j.corner_file),'utf8').trim().split(/\r?\n/);
  assert.equal(lines.shift(),'view_id,point_index,x,y,x_fp32_hex,y_fp32_hex');
  let n=0;
  j.views.forEach(view=>{for(let i=0;i<view.corner_count;++i){
    const row=lines[n++].split(',');assert.equal(+row[0],view.view_id);assert.equal(+row[1],i);
    pair(row[2]==='null'?null:Number(row[2]),row[4],4);
    pair(row[3]==='null'?null:Number(row[3]),row[5],4);
  }});
  assert.equal(n,caseId<2?120:87);assert.equal(n,lines.length);
  if(caseId===2){assert.equal(j.result,null);return;}
  const r=j.result;assert.equal(r.camera_valid,caseId===0);assert.equal(r.accepted_steps,25);
  assert.equal(r.message,'quotes" backslash\\ newline\n tab\t');
  assert.equal(Object.keys(r.camera).join(','),'fx,fy,cx,cy,k1,k2,k3,p1,p2');
  for(const k of Object.keys(r.camera))pair(r.camera[k],r.camera_fp32_hex[k],4);
  pair(r.rms,r.rms_fp64_hex,8);pair(r.max_error,r.max_error_fp64_hex,8);
  r.per_view_rms.forEach((x,i)=>pair(x,r.per_view_rms_fp64_hex[i],8));
  r.poses.forEach((p,v)=>{
    assert.equal(p.view_id,v);
    assert.equal(p.rotation_row_major.length,9);assert.equal(p.translation.length,3);
    p.rotation_row_major.forEach((x,i)=>pair(x,p.rotation_fp64_hex[i],8));
    p.translation.forEach((x,i)=>pair(x,p.translation_fp64_hex[i],8));
  });
});
console.log(`EXPORT_PASS cases=4 roundtrip_values=${checks}`);
