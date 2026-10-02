// Generate Q128 constants using integer series at 1536-bit precision.
// node generate_fp_constants.js
function generateConstants() {
const Q=1536n,unit=1n<<Q;
function atanInv(n){let sum=0n,den=n,ns=n*n;for(let k=0;k<2000;k++){let t=unit/(den*BigInt(2*k+1));if(!t)break;sum+=(k%2?-t:t);den*=ns;}return sum;}
const pi=16n*atanInv(5n)-4n*atanInv(239n);
const ln2=2n*(()=>{let sum=0n,p=3n;for(let k=0;k<1200;k++){let t=unit/(p*BigInt(2*k+1));if(!t)break;sum+=t;p*=9n;}return sum;})();
function isqrt(x){if(x<2n)return x;let a=1n<<BigInt(Math.ceil(x.toString(2).length/2));while(true){let b=(a+x/a)>>1n;if(b>=a)return a;a=b;}}
let gain=unit;for(let i=0;i<128;i++){let f=isqrt(unit*unit+(unit*unit>>(2n*BigInt(i))));gain=gain*unit/f;}
const to128=x=>(x+(1n<<(Q-129n)))>>(Q-128n),hx=(x,n)=>x.toString(16).padStart(n,"0");
let s="// Q128常量由整数高精度级数生成；三角范围缩减使用1280位2/pi。\n";
for(const [name,value] of [["FX_ONE",1n<<128n],["FX_PI",to128(pi)],["FX_HALF_PI",to128(pi/2n)],["FX_LN2",to128(ln2)],["FX_GAIN",to128(gain)]])
s+="localparam signed [191:0] "+name+" = 192'h"+hx(value,48)+";\n";
s+="localparam [1279:0] TWO_OVER_PI = 1280'h"+hx(((2n*unit)<<1280n)/pi,320)+";\n";
s+="function signed [191:0] cordic_angle;\n    input [6:0] index;\n    begin\n        case (index)\n";
for(let i=0;i<128;i++)s+="        7'd"+i+": cordic_angle=192'h"+hx(to128(i===0?pi/4n:atanInv(1n<<BigInt(i))),48)+";\n";
s+="        default: cordic_angle=192'd0;\n        endcase\n    end\nendfunction\n";
return s;
}
if(typeof module!=="undefined" && require.main===module){
require("fs").writeFileSync(require("path").join(__dirname,"../rtl/math/fp_constants.vh"),generateConstants());
}
