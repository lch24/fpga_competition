"""Independent instruction model: C++ seeds and configurable synthetic geometry."""
import math
import struct
from assemble import ROOT
from build_init import build
from build_lm import M,C,CONSTANTS
from reference import Machine,bits,value
from check_lm_program import layout

def run(rom,views,rows,cols,width,height,points):
    count=rows*cols;_,words=layout(views,count)
    machine=Machine(rom,words,words,instruction_limit=10000000)
    machine.mem.update({i:0 for i in range(288)})
    machine.mem.update({C[k]:v for k,v in CONSTANTS.items()})
    px=1024+views*count;py=px+count;bx=py+count;by=bx+count;seed=by+count
    assert seed+9+6*views<=words
    for name,n in dict(views=views,points=count,fp_points=bits(count),fwidth=bits(width),
                      fheight=bits(height),init_raw=1024,init_px=px,init_py=py,
                      init_bx=bx,init_by=by,init_seed=seed).items():machine.mem[M[name]]=n
    for i,packed in enumerate(points):machine.mem[1024+i]=packed
    for i in range(count):
        machine.mem[bx+i]=bits(i%cols-(cols-1)/2)
        machine.mem[by+i]=bits(i//cols-(rows-1)/2)
    assert machine.run(0,len(rom))==0
    result=[value(machine.mem.get(seed+i,0)) for i in range(9+6*views)]
    return machine.mem[M['status']],result,machine.counts['instructions']

def close(actual,expected,tolerance):
    assert all(math.isfinite(a) and abs(a-b)<=tolerance*(1+abs(b)) for a,b in zip(actual,expected)),(actual,expected)

def main():
    rom,_=build();tested=0
    for line in (ROOT/'data/calibration/init_controller_vectors.txt').read_text().splitlines():
        f=line.split();status,width,height,mode,count=map(int,f[:5])
        if mode in (2,3) or width<2 or height<2:continue # RTL host/error injection cases.
        raw=int(f[5],16);expected=int(f[8],16)
        points=[raw>>(64*i)&((1<<64)-1) for i in range(120)]
        code,result,instructions=run(rom,3,5,8,width,height,points)
        assert code==status,(tested,code,status)
        if not code:close(result,[value(expected>>(64*i)&((1<<64)-1)) for i in range(27)],3e-7)
        print(f'INIT_MODEL fixture={tested} status={code} instructions={instructions}');tested+=1
    # Four 6x7 views from an independently generated pinhole camera. No ROM changes.
    points=[];expected=[math.log(640),math.log(640),319.5/640,239.5/480]+[0.]*5
    pack=lambda x:struct.unpack('<I',struct.pack('<f',x))[0]
    for v,angle in enumerate((.2,-.25,.3,-.15)):
        tx=.1*v;ty=.2;tz=12+v
        expected.extend((angle,0.,0.,tx,ty,math.log(tz)))
        for row in range(6):
            for col in range(7):
                x=col-3;y=row-2.5;z=math.sin(angle)*y+tz
                u=640*(x+tx)/z+319.5;w=640*(math.cos(angle)*y+ty)/z+239.5
                points.append(pack(u)|(pack(w)<<32))
    status,result,instructions=run(rom,4,6,7,640,480,points)
    assert status==0
    close(result,expected,2e-5)
    print(f'INIT_MODEL_PASS fixtures={tested} configurable=4x6x7 instructions={instructions}')

if __name__=='__main__':main()
