// File-format regression only. These mock outputs are NOT RTL or C++ results.
const fs=require('fs'),path=require('path'),cp=require('child_process'),assert=require('assert');
const root=path.join(__dirname,'../../build/calibration/build_config_adapter_test'),source=path.join(root,'mock_export'),build=path.join(root,'mock_output');
fs.mkdirSync(source,{recursive:true});fs.mkdirSync(build,{recursive:true});
const run=(file,args)=>cp.spawnSync(process.execPath,[path.join(__dirname,'../../data/generators/calibration',file),...args],{encoding:'utf8'});
let result=run('generate_config_vectors.js',[root,'5','7','10']);assert.equal(result.status,0,result.stderr);
const lines=f=>fs.readFileSync(path.join(root,f),'utf8').trim().split(/\r?\n/);
const points=lines('points.hex'),m=lines('metrics.hex');
const f64=h=>Buffer.from(h,'hex').readDoubleBE(),f32=h=>Buffer.from(h,'hex').readFloatBE();
const hex32=x=>{let b=Buffer.alloc(4);b.writeFloatBE(x);return b.toString('hex');};
const names=['fx','fy','cx','cy','k1','k2','k3','p1','p2'],cam=[800,820,640,360,0,0,0,0,0];
const meta={format:'closer2fpga.calibration.v1',calibration_attempted:true,rtl_input_ready:false,
 rows:7,cols:10,width:1280,height:720,square_size:2.5,square_size_fp64_hex:'4004000000000000',max_iterations_per_stage:1,estimate_k3:false,corner_file:'corners.csv',
 views:Array.from({length:5},(_,v)=>({view_id:v,image_loaded:true,board_valid:true,corner_count:70,width:1280,height:720})),
 result:{camera_valid:true,converged:true,weak_geometry:false,metrics_present:true,accepted_steps:3,
 camera:Object.fromEntries(names.map((n,i)=>[n,cam[i]])),camera_fp32_hex:Object.fromEntries(names.map((n,i)=>[n,hex32(cam[i])])),
 rms:f64(m[0]),rms_fp64_hex:m[0],per_view_rms:m.slice(1,6).map(f64),per_view_rms_fp64_hex:m.slice(1,6),max_error:f64(m[6]),max_error_fp64_hex:m[6],
 poses:Array.from({length:5},(_,v)=>{const a=m.slice(7+12*v,19+12*v);return {view_id:v,rotation_row_major:a.slice(0,9).map(f64),rotation_fp64_hex:a.slice(0,9),translation:a.slice(9).map(f64),translation_fp64_hex:a.slice(9)};})}};
const writeMeta=()=>fs.writeFileSync(path.join(source,'calibration.json'),JSON.stringify(meta));writeMeta();
fs.writeFileSync(path.join(source,'COMPLETE.txt'),meta.format+'\n');
const csv='view_id,point_index,x,y,x_fp32_hex,y_fp32_hex\n'+points.map((h,i)=>[Math.floor(i/70),i%70,f32(h.slice(8)),f32(h.slice(0,8)),h.slice(8),h.slice(0,8)].join(',')).join('\n')+'\n';
fs.writeFileSync(path.join(source,'corners.csv'),csv);
const invoke=mode=>run('calib_real_data.js',[mode,source,build]);
result=invoke('prepare');assert.equal(result.status,0,result.stderr);
const packed=fs.readFileSync(path.join(build,'real_vectors.txt'),'utf8').trim().split(/\s+/);
assert.equal(packed[9],[...points].reverse().join(''));assert.equal(packed[8],[...m].reverse().join(''));
fs.writeFileSync(path.join(build,'actual_camera.hex'),cam.map(hex32).join('\n')+'\n');
fs.writeFileSync(path.join(build,'actual_metrics.hex'),m.join('\n')+'\n');
fs.writeFileSync(path.join(build,'calib_top_results.txt'),'LM seed=2 stage=2 cycles=99 cost=1 converged=1 accepted=3 status=0\ncase=0 cycles=100 reads=350 seeds=1 lm_calls=1 status=0 seed=2 steps=3\nRESULT cases=1 errors=0\n');
result=invoke('compare');assert.equal(result.status,0,result.stderr);assert.match(result.stdout,/fields=76/);
const wrong=[...cam];wrong[0]+=1;fs.writeFileSync(path.join(build,'actual_camera.hex'),wrong.map(hex32).join('\n')+'\n');
result=invoke('compare');assert.equal(result.status,1);assert.match(result.stdout,/numerical_failures=1/);
meta.max_iterations_per_stage=256;writeMeta();assert.notEqual(invoke('prepare').status,0);meta.max_iterations_per_stage=1;
meta.rows=8;writeMeta();assert.notEqual(invoke('prepare').status,0);meta.rows=7;writeMeta();
fs.writeFileSync(path.join(source,'corners.csv'),csv.replace('\n0,0,','\n0,1,'));assert.notEqual(invoke('prepare').status,0);
console.log('CONFIG_ADAPTER_PASS points=350 fields=76 mismatch/config/order rejection checked (mock format test only)');
