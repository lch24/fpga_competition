"""Independent double inverse and FP32 iteration checks; no external packages."""
from pathlib import Path
import math,struct,json
ROOT=Path(__file__).resolve().parents[3]
def f(x):return struct.unpack('<f',struct.pack('<f',x))[0]
def solve(x,y,w=1280,h=720):
 focal=w*4//5;cx=(w-1)/2;cy=(h-1)/2
 xd=(x-cx)/focal;yd=(y-cy)/focal;nx,ny=xd,yd
 # Independent 2D Newton solve with analytic Brown Jacobian.
 for _ in range(30):
  r=nx*nx+ny*ny;d=1-.18*r+.04*r*r-.005*r*r*r
  g=-.18+.08*r-.015*r*r
  ex=nx*d+.004*nx*ny-.0015*(r+2*nx*nx)-xd
  ey=ny*d+.002*(r+2*ny*ny)-.003*nx*ny-yd
  a=d+2*nx*nx*g+.004*ny-.009*nx
  b=2*nx*ny*g+.004*nx-.003*ny
  c=d+2*ny*ny*g+.012*ny-.003*nx
  det=a*c-b*b
  assert det>0
  nx,ny=nx-(c*ex-b*ey)/det,ny-(a*ey-b*ex)/det
 assert math.hypot(ex,ey)<1e-12
 return cx+focal*nx,cy+focal*ny

def rtl_model(x,y):
 xd=f(f(x-639.5)/1024);yd=f(f(y-359.5)/1024);nx,ny=xd,yd
 for _ in range(16):
  x2=f(nx*nx);y2=f(ny*ny);r=f(x2+y2);r4=f(r*r);r6=f(r4*r)
  den=f(f(f(1+f(f(-.18)*r))+f(f(.04)*r4))+f(f(-.005)*r6))
  xy=f(nx*ny);p1xy=f(f(.002)*xy);p2xy=f(f(-.0015)*xy)
  tx1=f(p1xy+p1xy);tx2=f(f(-.0015)*f(r+f(x2+x2)))
  ty1=f(f(.002)*f(r+f(y2+y2)));ty2=f(p2xy+p2xy)
  nx=f(f(f(xd-tx1)-tx2)/den);ny=f(f(f(yd-ty1)-ty2)/den)
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
