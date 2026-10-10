"""Independent instruction interpreter and operation census, not a cycle oracle.

Python binary64 arithmetic supplies results; Fraction comparisons supply basic
IEEE exception flags. RTL timing/protocol is verified by the separate TB.
Run after generate_checks.py; reads the same transactions, never RTL internals.
"""
from collections import Counter
from fractions import Fraction
import json
import math
import struct
from assemble import ROOT, ISA

OUT=ROOT/'build/calibration_engine'
NAN=0x7ff8000000000000
MASK=(1<<64)-1

def value(bits):return struct.unpack('<d',struct.pack('<Q',bits))[0]
def bits(x):return struct.unpack('<Q',struct.pack('<d',x))[0]
def signed(x,width):return x-(1<<width) if x&(1<<(width-1)) else x
def snan(x):return (x>>52)&2047==2047 and x&((1<<52)-1)!=0 and not x&(1<<51)

def floating(op,a,b=0):
    x,y=value(a),value(b)
    if op in (42,44,45,47):
        if math.isnan(x):return NAN,int(snan(a)),None
        try:z={42:math.exp,44:math.sin,45:math.cos,47:math.acos}[op](x)
        except ValueError:return NAN,1,None
        except OverflowError:return bits(math.inf),20,None
        return bits(z),16,None
    if op==46:
        if math.isnan(x) or math.isnan(y):return NAN,int(snan(a) or snan(b)),None
        return bits(math.atan2(x,y)),16,None
    if op==43:
        if math.isnan(x):return NAN,int(snan(a)),None
        if x<0:return NAN,1,None
        if x==0:return 0xfff0000000000000,2,None
        return bits(math.log(x)),0 if x==1 or math.isinf(x) else 16,None
    if op==40:raise ValueError('FCVT needs explicit direction')
    if op==37:
        return 0,int(snan(a) or snan(b)),(x<y,x==y,math.isnan(x) or math.isnan(y))
    operands=(x,) if op==36 else (x,y)
    if any(math.isnan(v) for v in operands):
        return NAN,int(snan(a) or (op!=36 and snan(b))),None
    invalid=(op==32 and math.isinf(x) and x==-y) or (op==33 and math.isinf(x) and x==y)
    invalid|=op==34 and ((x==0 and math.isinf(y)) or (y==0 and math.isinf(x)))
    invalid|=op==35 and ((x==0 and y==0) or (math.isinf(x) and math.isinf(y)))
    invalid|=op==36 and x<0
    if invalid:return NAN,1,None
    if op==35 and y==0:
        return ((a^b)&(1<<63))|0x7ff0000000000000,0 if math.isinf(x) else 2,None
    z={32:lambda:x+y,33:lambda:x-y,34:lambda:x*y,35:lambda:x/y,36:lambda:math.sqrt(x)}[op]()
    flags=0
    if all(math.isfinite(v) for v in operands):
        if math.isinf(z):flags=20
        else:
            X,Y=Fraction(x),Fraction(y)
            if op==36:lost=Fraction(z)**2!=X
            else:
                exact={32:lambda:X+Y,33:lambda:X-Y,34:lambda:X*Y,35:lambda:X/Y}[op]()
                lost=Fraction(z)!=exact
            flags=(16 if lost else 0)|(8 if lost and abs(z)<2**-1022 else 0)
    return bits(z),flags,None

class Halt(Exception):
    def __init__(self,status):self.status=status

class Machine:
    def __init__(self,rom,ram_words=4096,const_base=3584,host=None,instruction_limit=1000000):
        self.rom=rom;self.mem={};self.counts=Counter();self.ops=Counter()
        self.ram_words=ram_words;self.const_base=const_base;self.host=host;self.instruction_limit=instruction_limit
    def read(self,address):
        if not 0<=address<self.ram_words:raise Halt(0xe4)
        if address not in self.mem:raise AssertionError(f'uninitialized read at {address}')
        self.counts['reads']+=1;return self.mem[address]
    def write(self,address,value):
        if not 0<=address<self.const_base:raise Halt(0xe4)
        self.counts['writes']+=1;self.mem[address]=value
    def fp(self,op,a,b=0):
        self.ops[str(op)]+=1
        result,flags,condition=floating(op,a,b)
        self.flags|=flags
        if condition is not None:self.condition=condition
        return result
    def kernel(self,kind,address):
        d=[self.read(address+i) for i in range(8)]
        low=lambda i:d[i]&0xffffffff
        high=lambda i:d[i]>>32
        s0,s1,dst,n,st0,st1,std=low(0),high(0),low(1),high(1),low(2),high(2),low(3)
        if n>4096 or high(3) or high(4) or d[6] or d[7]:raise Halt(0xe6)
        if kind==4:
            if low(4)!=n or std!=1 or s1 or d[5]:raise Halt(0xe6)
        elif low(4):raise Halt(0xe6)
        first=[s0+i*st0 for i in range(n)] if kind else []
        second=[s1+i*st1 for i in range(n)] if kind in (2,3) else []
        dest=[dst] if kind==2 else [dst+i*std for i in range(n*(n+1)//2 if kind==4 else n)]
        if any(a<0 or a>=4096 for a in first+second) or any(a<0 or a>=3584 for a in dest):raise Halt(0xe6)
        def overlap(a,b):return bool(a and b and min(a)<=max(b) and min(b)<=max(a))
        if overlap(first,dest) and not (kind==1 and first==dest):raise Halt(0xe6)
        if overlap(second,dest) and not (kind==3 and second==dest):raise Halt(0xe6)
        if kind==0:
            for a in dest:self.write(a,0)
        elif kind==1:
            for a,b in zip(first,dest):self.write(b,self.read(a))
        elif kind==2:
            acc=0
            for a,b in zip(first,second):acc=self.fp(32,self.fp(34,self.read(a),self.read(b)),acc)
            self.write(dst,acc)
        elif kind==3:
            for a,b,c in zip(first,second,dest):self.write(c,self.fp(32,self.fp(34,self.read(a),d[5]),self.read(b)))
        else:
            k=0
            for i,a in enumerate(first):
                for b in first[i:]:
                    self.write(dest[k],self.fp(32,self.fp(34,self.read(a),self.read(b)),self.read(dest[k])));k+=1
    def run(self,entry,limit):
        self.a=[0]*8;self.f=[0]*8;self.flags=0;self.condition=(False,False,False)
        self.counts=Counter();self.ops=Counter();stack=[];pc=entry;guard=None
        definitions={v[0]:v for v in ISA['instructions'].values()}
        try:
            while True:
                if not 0<=pc<limit:raise Halt(0xe2)
                self.counts['instructions']+=1
                if self.counts['instructions']>self.instruction_limit:raise AssertionError('instruction budget exceeded')
                w=self.rom[pc];op=w>>26;mode=(w>>23)&7;rd=(w>>20)&7;ra=(w>>17)&7;rb=(w>>14)&7;imm=w&16383
                if op not in definitions:raise Halt(0xe1)
                _,signature,modes=definitions[op]
                if mode not in modes:raise Halt(0xe1)
                allowed=0xff800000
                for operand in signature.split(',') if signature else []:
                    if operand[0] in 'AF':
                        offset,width=ISA['fields']['r'+operand[1]];allowed|=((1<<width)-1)<<offset
                    else:allowed|=(1<<(14 if operand=='rel' else int(operand[1:])))-1
                if w&(~allowed&MASK) or (op==48 and imm>4):raise Halt(0xe1)
                nxt=pc+1;offset=signed(imm,14)
                a,f=self.a,self.f
                if op==0:pass
                elif op==1:raise Halt(imm)
                elif op==2:a[rd]=imm
                elif op==3:a[rd]=(a[ra]+offset)&0xffffffff
                elif op==4:a[rd]=(a[ra]+a[rb])&0xffffffff
                elif op==5:a[rd]=(a[ra]-a[rb])&0xffffffff
                elif op==6:
                    x,y=(signed(a[ra],32),signed(a[rb],32)) if mode else (a[ra],a[rb]);self.condition=(x<y,x==y,False)
                elif op==7:
                    lt,eq,un=self.condition
                    if [True,eq,not eq,lt and not un,(lt or eq)and not un,not(lt or eq or un),not(lt or un),un][mode]:nxt+=offset
                elif op==8:
                    if len(stack)==4:raise Halt(0xe3)
                    if nxt>=limit:raise Halt(0xe2)
                    stack.append(nxt);nxt+=offset
                elif op==9:
                    if not stack:raise Halt(0xe3)
                    nxt=stack.pop()
                elif op==10:
                    if a[rd]==0:raise Halt(0xe7)
                    a[rd]-=1
                    if a[rd]:nxt+=offset
                elif op in (16,17,19,20):
                    address=a[ra]+offset
                    if op==16:f[rd]=self.read(address)
                    elif op==19:a[rd]=self.read(address)&0xffffffff
                    else:self.write(address,f[rd] if op==17 else a[rd])
                elif op==18:f[rd]=f[ra]
                elif op==21:f[rd]=(f[ra]>>(32*mode))&0xffffffff
                elif 32<=op<=37 or op in (42,43,44,45,46,47):
                    sticky=self.flags;self.flags=0
                    result=self.fp(op,f[ra],f[rb])
                    if guard and (self.flags&7 or (result>>52)&2047==2047):
                        nxt,depth=guard;stack=stack[:depth]
                    elif op!=37:f[rd]=result
                    self.flags|=sticky
                elif op==38:f[rd]=f[ra]&((1<<63)-1)
                elif op==39:f[rd]=f[ra]^(1<<63)
                elif op==40:
                    if mode==0:
                        x=f[ra]&0xffffffff;y=struct.unpack('<f',struct.pack('<I',x))[0]
                        f[rd]=NAN if math.isnan(y) else bits(y)
                        if (x&0x7f800000)==0x7f800000 and (x&0x7fffff) and not x&(1<<22):self.flags|=1
                    else:
                        x=value(f[ra]);old=f[ra]
                        if math.isnan(x):f[rd]=0x7fc00000;self.flags|=int(snan(old))
                        else:
                            try:raw=struct.pack('<f',x)
                            except OverflowError:raw=struct.pack('<f',math.copysign(math.inf,x));self.flags|=20
                            f[rd]=struct.unpack('<I',raw)[0];rounded=struct.unpack('<f',raw)[0]
                            if math.isfinite(x) and rounded!=x:self.flags|=16|(8 if abs(rounded)<2**-126 else 0)
                elif op==41:
                    width=23 if mode else 52;mask=(1<<(8 if mode else 11))-1
                    e=(f[ra]>>width)&mask;frac=f[ra]&((1<<width)-1)
                    a[rd]=16 if e==mask and frac else 8 if e==mask else 4 if e else 2 if frac else 1
                elif op==48:
                    if a[ra]>4088:raise Halt(0xe4)
                    self.kernel(imm,a[ra])
                elif op==49:a[rd]=self.flags if mode==0 else 0
                elif op==50:self.flags=0
                elif op==51:
                    if self.host is None:raise Halt(0xe1)
                    self.host(self,imm)
                elif op==52:guard=(nxt+offset,len(stack))
                pc=nxt
        except Halt as halt:return halt.status

def main():
    machine=Machine([int(s,16) for s in (OUT/'checks.mem').read_text().split()]);limit=0;reports=[]
    for line in (OUT/'checks.txt').read_text().splitlines():
        action,address,data=line.split();action=int(action);address=int(address,16);data=int(data,16)
        if action==0:machine.mem[address]=data
        elif action==3:limit=address
        elif action==1:
            status=machine.run(address,limit)
            assert status==data,(address,status,data)
            reports.append(dict(entry=address,status=status,counts=dict(machine.counts),fp_ops=dict(machine.ops)))
        elif action==2:assert machine.mem[address]==data,(address,hex(machine.mem[address]),hex(data))
        elif action==5:
            got,expected=value(machine.mem[address]),value(data)
            assert math.isfinite(got) and abs(got-expected)<=2e-11+2e-11*abs(expected),(address,got,expected)
        # Cancellation is a cycle/protocol property and is covered in RTL.
        elif action!=4:raise AssertionError(action)
    (OUT/'operation_counts.json').write_text(json.dumps(reports,indent=2))
    print(f'ENGINE_REFERENCE_PASS {len(reports)} complete jobs; operation census saved')

if __name__=='__main__':main()
