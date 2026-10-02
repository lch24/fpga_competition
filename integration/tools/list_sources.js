const fs=require('fs'),path=require('path');
const root=path.resolve(__dirname,'../..');
function walk(p){return fs.readdirSync(path.join(root,p),{withFileTypes:true}).flatMap(e=>e.isDirectory()?walk(p+'/'+e.name):/\.(v|sv)$/.test(e.name)?[p+'/'+e.name]:[]);}
const files=['algorithom/closer2fpga/rtl','parameter/rtl','undistort/rtl','integration/rtl'].flatMap(walk).sort((a,b)=>{
 const pa=a.endsWith('/lround_pkg.sv'),pb=b.endsWith('/lround_pkg.sv');return pa!==pb?(pa?-1:1):a.localeCompare(b);
});
fs.writeFileSync(path.join(root,'integration/files.f'),[
 '+incdir+parameter/rtl/common','+incdir+parameter/rtl/math','+incdir+undistort/rtl/include',...files,''].join('\n'));
console.log(`Listed ${files.length} production RTL sources; compile from repository root.`);
