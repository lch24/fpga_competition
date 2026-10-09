// Convert an actual C++ export into a bit-exact RTL input; compare returned bits.
// No calibration math is implemented here and no C++ camera value initializes RTL.
const fs=require('fs'),path=require('path'),crypto=require('crypto'),assert=require('assert');
const [mode,sourceArg,buildArg]=process.argv.slice(2);
assert.ok(['prepare','compare'].includes(mode)&&sourceArg&&buildArg,'usage: prepare|compare export_directory build_directory');
const source=path.resolve(sourceArg),build=path.resolve(buildArg);
assert.notEqual(source,build,'export and build must be separate');
const read=name=>fs.readFileSync(path.join(source,name));
const hash=b=>crypto.createHash('sha256').update(b).digest('hex');
const hashes={};for(const name of ['corners.csv','calibration.json','COMPLETE.txt'])hashes[name]=hash(read(name));
assert.equal(read('COMPLETE.txt').toString('utf8').split(/\r?\n/)[0],'closer2fpga.calibration.v1','incomplete export');
const meta=JSON.parse(read('calibration.json').toString('utf8')),r=meta.result;
assert.equal(meta.format,'closer2fpga.calibration.v1');
// Older exports used rtl_input_ready for only 3x5x8. Revalidate actual metadata below.
assert.ok(meta.calibration_attempted,'calibration was not attempted');
const views=meta.views.length,pointsPerView=meta.rows*meta.cols,totalPoints=views*pointsPerView,metricCount=2+13*views;
assert.ok(Number.isInteger(views)&&views>=3&&views<=16);
assert.ok(Number.isInteger(meta.rows)&&meta.rows>=2&&Number.isInteger(meta.cols)&&meta.cols>=2&&pointsPerView<=256);
assert.ok(Number.isInteger(meta.max_iterations_per_stage)&&meta.max_iterations_per_stage>=1&&meta.max_iterations_per_stage<=255);
assert.ok(r&&r.camera_valid&&r.metrics_present&&r.converged,'this success-comparison fixture requires a valid C++ result');
assert.equal(meta.estimate_k3,false);
assert.ok(Number.isInteger(meta.width)&&meta.width>=2&&meta.width<=65535);
assert.ok(Number.isInteger(meta.height)&&meta.height>=2&&meta.height<=65535);
assert.equal(meta.corner_file,'corners.csv');
function decoded(hex,size) {
 assert.ok(typeof hex==='string'&&new RegExp(`^[0-9a-fA-F]{${size*2}}$`).test(hex),'invalid IEEE hex');
 const bytes=Buffer.from(hex,'hex'),x=size===4?bytes.readFloatBE():bytes.readDoubleBE();
 assert.ok(Number.isFinite(x),'nonfinite reference/output');return x;
}
function same(value,hex,size) {assert.ok(Object.is(value,decoded(hex,size)),'decimal and IEEE bits disagree');return hex.toLowerCase();}
same(meta.square_size,meta.square_size_fp64_hex,8);assert.ok(meta.square_size>0);

meta.views.forEach((v,i)=>{assert.equal(v.view_id,i);assert.ok(v.image_loaded&&v.board_valid);assert.equal(v.corner_count,pointsPerView);assert.equal(v.width,meta.width);assert.equal(v.height,meta.height);});
const lines=read('corners.csv').toString('utf8').trim().split(/\r?\n/);
assert.equal(lines.shift(),'view_id,point_index,x,y,x_fp32_hex,y_fp32_hex');assert.equal(lines.length,totalPoints);
const points=lines.map((line,i)=>{
 const a=line.split(',');assert.equal(a.length,6);assert.equal(+a[0],Math.floor(i/pointsPerView));assert.equal(+a[1],i%pointsPerView);
 const x=same(Number(a[2]),a[4],4),y=same(Number(a[3]),a[5],4);
 assert.ok(+a[2]>=0&&+a[2]<meta.width&&+a[3]>=0&&+a[3]<meta.height);
 return y+x;
});
const cameraNames=['fx','fy','cx','cy','k1','k2','k3','p1','p2'];
const camera=cameraNames.map(n=>same(r.camera[n],r.camera_fp32_hex[n],4));assert.equal(decoded(camera[6],4),0);
const metricNames=['rms',...Array.from({length:views},(_,v)=>`view${v}_rms`),'max_error'];
assert.equal(r.per_view_rms.length,views);assert.equal(r.per_view_rms_fp64_hex.length,views);assert.equal(r.poses.length,views);
const metrics=[same(r.rms,r.rms_fp64_hex,8),...r.per_view_rms.map((x,i)=>same(x,r.per_view_rms_fp64_hex[i],8)),same(r.max_error,r.max_error_fp64_hex,8)];
r.poses.forEach((p,v)=>{
 assert.equal(p.view_id,v);assert.equal(p.rotation_row_major.length,9);assert.equal(p.rotation_fp64_hex.length,9);
 assert.equal(p.translation.length,3);assert.equal(p.translation_fp64_hex.length,3);
 p.rotation_row_major.forEach((x,i)=>{metrics.push(same(x,p.rotation_fp64_hex[i],8));metricNames.push(`view${v}_R${Math.floor(i/3)}${i%3}`);});
 p.translation.forEach((x,i)=>{metrics.push(same(x,p.translation_fp64_hex[i],8));metricNames.push(`view${v}_t${i}`);});
});
if(mode==='prepare') {
 fs.mkdirSync(build,{recursive:true});
 for(const name of ['actual_camera.hex','actual_metrics.hex','calib_top_results.txt','real_comparison.md','real_comparison.csv']) {
  const file=path.join(build,name);if(fs.existsSync(file))fs.unlinkSync(file);
 }
 const vector=`${meta.width} ${meta.height} ${meta.square_size_fp64_hex}\n7 ${r.accepted_steps} ${+r.converged} ${+r.weak_geometry} ${[...camera].reverse().join('')} ${[...metrics].reverse().join('')} ${[...points].reverse().join('')}\n`;
 fs.writeFileSync(path.join(build,'real_vectors.txt'),vector);
 fs.writeFileSync(path.join(build,'real_source.json'),JSON.stringify({source,hashes,width:meta.width,height:meta.height,square_size:meta.square_size},null,2)+'\n');
 console.log(`REAL_INPUT_READY points=${totalPoints} size=${meta.width}x${meta.height} square_size=${meta.square_size}`);
} else {
 const previous=JSON.parse(fs.readFileSync(path.join(build,'real_source.json'),'utf8'));
 assert.equal(previous.source,source);assert.deepEqual(previous.hashes,hashes,'export changed during simulation');
 const rawCamera=fs.readFileSync(path.join(build,'actual_camera.hex'),'utf8').trim().split(/\r?\n/);
 const rawMetrics=fs.readFileSync(path.join(build,'actual_metrics.hex'),'utf8').trim().split(/\r?\n/);
 assert.equal(rawCamera.length,9);assert.equal(rawMetrics.length,metricCount);
 const rows=[];let failures=0;
 function compare(name,expected,actual,size,tol) {
  const cpp=decoded(expected,size),rtl=decoded(actual,size),delta=Math.abs(cpp-rtl),limit=tol*(1+Math.abs(cpp));
  const pass=delta<=limit;if(!pass)++failures;
  const row={name,cpp,rtl,delta,limit,bit_equal:expected.toLowerCase()===actual.toLowerCase(),pass};rows.push(row);return row;
 }
 cameraNames.forEach((n,i)=>compare(n,camera[i],rawCamera[i],4,2e-5));
 metricNames.forEach((n,i)=>compare(n,metrics[i],rawMetrics[i],8,i<views+2?2e-7:2e-5));
 const log=fs.readFileSync(path.join(build,'calib_top_results.txt'),'utf8');
 const stages=[...log.matchAll(/^LM seed=(\d+) stage=(\d+) cycles=(\d+) cost=(\S+) converged=(\d+) accepted=(\d+) status=(\d+)$/gm)];
 assert.equal(stages.length,1,'single-seed RTL must issue one LM job');
 const m=stages[0];assert.equal(+m[1],2);assert.equal(+m[2],2);assert.equal(+m[7],0);
 const cost=Number(m[4]);assert.ok(Number.isFinite(cost)&&cost>=0,'invalid cost log');
 const best={seed:2,cost,total:+m[6]};
 const final=log.match(/^case=0 cycles=(\d+) reads=(\d+) seeds=(\d+) lm_calls=(\d+) status=(\d+) seed=(\d+) steps=(\d+)$/m);
 assert.ok(final,'no completed task');assert.equal(+final[5],0);assert.equal(+final[3],1);assert.equal(+final[4],stages.length);
 assert.ok(best,'no finite best result');assert.equal(+final[6],best.seed);assert.equal(+final[7],best.total);
 const summary=log.match(/^RESULT cases=1 errors=(\d+)$/m);assert.ok(summary,'simulation not completed');
 if(+summary[1]!==0)++failures;
 const table=rows.slice(0,11+views).map(x=>`| ${x.name} | ${x.cpp.toPrecision(12)} | ${x.rtl.toPrecision(12)} | ${x.delta.toExponential(4)} | ${x.pass?'PASS':'FAIL'} |`).join('\n');
 const poseMax=Math.max(...rows.slice(11+views).map(x=>x.delta));
 fs.writeFileSync(path.join(build,'real_comparison.csv'),'name,cpp,fpga,absolute_difference,allowed_difference,bit_equal,pass\n'+rows.map(x=>[x.name,x.cpp,x.rtl,x.delta,x.limit,x.bit_equal,x.pass].join(',')).join('\n')+'\n');
 fs.writeFileSync(path.join(build,'real_comparison.md'),`# 真实角点 FPGA / C++ 对比\n\n输入目录：\`${source}\`\n\n配置：${meta.width}×${meta.height}，${views}张${meta.rows}×${meta.cols}棋盘，共${totalPoints}角点，格长${meta.square_size}，k3固定0。全部为真实RTL，无结果注入。\n\n结果：${failures?'FAIL':'PASS'}；TB错误数${summary[1]}；候选${final[3]}个，LM调用${stages.length}次。RTL最佳seed=${best.seed}，接受${best.total}次，C++接受${r.accepted_steps}次。\n\n周期：${final[1]}；角点读取：${final[2]}。\n\n| 字段 | C++ | FPGA | 绝对差 | 检查 |\n| --- | ---: | ---: | ---: | --- |\n${table}\n\n${12*views}项R/t最大绝对差：${poseMax.toExponential(8)}。全部${9+metricCount}个字段和比较阈值见real_comparison.csv。\n\n相机阈值2e-5×(1+|参考值|)，RMS/最大误差2e-7×(1+|参考值|)，姿态2e-5×(1+|参考值|)；沿用合成测试阈值。最佳seed和接受数另按RTL自身全部候选的历史核对，不要求跨实现候选编号或迭代次数相同。源文件SHA256见real_source.json。\n`);
 console.log(`REAL_COMPARISON_${failures?'FAIL':'PASS'} fields=${9+metricCount} numerical_failures=${rows.filter(x=>!x.pass).length} tb_errors=${summary[1]} seed=${best.seed} cycles=${final[1]}`);
 if(failures)process.exitCode=1;
}
