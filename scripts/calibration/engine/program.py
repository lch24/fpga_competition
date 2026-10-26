"""Assembly emitter shared by initialization, LM and final validation.

The methods append instructions; they do not perform floating-point arithmetic
on the host. A7 addresses the scalar workspace, A0/A1 and F0/F1 are scratch.
Loop labels are allocated in emission order, so callers share one Program.
"""
from workspace import M, C


class Program:

    def __init__(self):
        self.lines = []
        self.serial = 0

    def emit(self, *lines):
        self.lines.extend(lines)

    def label(self, name):
        self.emit(name + ':')

    def unique(self, name):
        self.serial += 1
        return f'{name}_{self.serial}'

    def ld(self, reg, name):
        self.emit(f'LD F{reg},A7,{M.get(name, C.get(name))}')

    def st(self, name, reg=0):
        self.emit(f'ST F{reg},A7,{M[name]}')

    def il(self, reg, name):
        self.emit(f'LDI32 A{reg},A7,{M[name]}')

    def ist(self, name, reg=0):
        self.emit(f'STI32 A{reg},A7,{M[name]}')

    def seti(self, name, n):
        self.emit(f'MOVI A0,{n}')
        self.ist(name)

    def icopy(self, d, s):
        self.il(0, s)
        self.ist(d)

    def fcopy(self, d, s):
        self.ld(0, s)
        self.st(d)

    def addi(self, d, s, n):
        self.il(0, s)
        self.emit(f'ADDI A0,A0,{n}')
        self.ist(d)

    def ibin(self, d, op, a, b):
        self.il(0, a)
        self.il(1, b)
        self.emit(f'{op} A0,A0,A1')
        self.ist(d)

    def fbin(self, d, op, a, b):
        self.ld(0, a)
        self.ld(1, b)
        self.emit(f'{op} F0,F0,F1')
        self.st(d)

    def unary(self, d, op, a):
        self.ld(0, a)
        self.emit(f'{op} F0,F0')
        self.st(d)

    def ib(self, a, cond, b, target):
        self.il(0, a)
        if isinstance(b, int):
            self.emit(f'MOVI A1,{b}')
        else:
            self.il(1, b)
        self.emit('ICMP A0,A1', f'BR.{cond} {target}')

    def fb(self, a, cond, b, target):
        self.ld(0, a)
        self.ld(1, b)
        self.emit('FCMP F0,F1', f'BR.{cond} {target}')

    def read(self, d, p, index=None):
        self.il(0, p)
        if index:
            self.il(1, index)
            self.emit('IADD A0,A0,A1')
        self.emit('LD F0,A0,0')
        self.st(d)

    def write(self, p, s, index=None):
        self.il(0, p)
        if index:
            self.il(1, index)
            self.emit('IADD A0,A0,A1')
        self.ld(0, s)
        self.emit('ST F0,A0,0')

    def copy(self, d, s, count):
        loop = self.unique('copy')
        self.il(0, d)
        self.il(1, s)
        self.il(2, count)
        self.label(loop)
        self.emit('LD F0,A1,0', 'ST F0,A0,0', 'ADDI A0,A0,1', 'ADDI A1,A1,1', f'DBNZ A2,{loop}')

    def desc(self, col, which):
        """Load [state index, first row, exclusive last row, Jacobian base]."""
        self.il(0, col)
        self.emit('IADD A0,A0,A0', 'IADD A0,A0,A0')
        self.il(1, 'desc')
        self.emit('IADD A0,A0,A1')
        for offset, name in enumerate(which):
            self.emit(f'LDI32 A1,A0,{offset}')
            self.ist(name, 1)

    def service(self, dest):
        self.icopy('svc_dst', dest)
        self.emit('HOST 0')

    def check_service(self, numeric):
        self.ib('svc_status', 1, 4, numeric)
        ok = self.unique('service_ok')
        self.ib('svc_status', 1, 0, ok)
        self.icopy('status', 'svc_status')
        self.emit('BR done')
        self.label(ok)
