"""Independent double inverse and FP32 iteration checks; no external packages."""
from pathlib import Path
import math,struct,json
ROOT=Path(__file__).resolve().parents[3]
def f(x):return struct.unpack('<f',struct.pack('<f',x))[0]
def solve(x,y,w=1280,h=720):
 focal=w*4//5;cx=(w-1)/2;cy=(h-1)/2
 xd=(x-cx)/focal;yd=(y-cy)/focal;rd=math.hypot(xd,yd);ru=rd
 for _ in range(30):ru-=(ru*(1-.18*ru*ru)-rd)/(1-.54*ru*ru)
 ratio=ru/rd if rd else 1
 return cx+focal*xd*ratio,cy+focal*yd*ratio

def rtl_model(x,y):
 xd=f(f(x-639.5)/1024);yd=f(f(y-359.5)/1024);nx,ny=xd,yd
 for _ in range(16):
  radius=f(f(nx*nx)+f(ny*ny));den=f(1+f(f(-.18)*radius));nx=f(xd/den);ny=f(yd/den)
 return f(f(1024*nx)+639.5),f(f(1024*ny)+359.5)
def main():
 out=ROOT/'build/demo';out.mkdir(parents=True,exist_ok=True)
 points=[(x,y) for y in range(0,720,19) for x in range(0,1280,23)]+[(0,0),(1279,719),(640,360)]
 worst=0
 with (out/'coordinates.txt').open('w') as stream:
  for x,y in points:
   sx,sy=solve(x,y);ax,ay=rtl_model(x,y);worst=max(worst,math.hypot(sx-ax,sy-ay));stream.write(f'{x} {y} {sx:.12f} {sy:.12f}\n')
 assert worst<.05,worst
 (out/'map_reference.json').write_text(json.dumps(dict(samples=len(points),max_coordinate_error_pixels=worst),indent=2))
 print('PASS independent inverse vs FP32 16 iterations:',len(points),'points; max error',worst)
if __name__=='__main__':main()
