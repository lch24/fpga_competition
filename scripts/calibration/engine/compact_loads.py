"""Remove proven redundant local loads within straight-line basic blocks.

No instruction reordering or floating-point changes. A cached value is valid
only while its register, its A7-relative RAM word and A7 itself are unchanged.
Unknown-address writes and every control-flow boundary invalidate the cache.
"""
from assemble import ISA


def compact_loads(source):
    cached={}
    lines=[]
    for raw in source.splitlines():
        text=raw.split('#',1)[0].strip()
        if not text:
            lines.append(raw);continue
        if ':' in text:
            cached.clear();lines.append(raw);continue
        head,*tail=text.split(None,1)
        op=head.split('.')[0].upper()
        operands=[s.strip() for s in tail[0].split(',')] if tail else []
        if op=='LD':
            reg,base,offset=operands
            if base=='A7' and cached.get(reg)==offset:
                continue
            cached.pop(reg,None)
            if base=='A7':cached[reg]=offset
        elif op in ('ST','STI32'):
            reg,base,offset=operands
            if base!='A7':cached.clear()
            else:
                cached={r:a for r,a in cached.items() if a!=offset}
                if op=='ST':cached[reg]=offset
        elif op in ('BR','CALL','RET','DBNZ','HOST','END','GUARD','KEXEC'):
            cached.clear()
        else:
            signature=ISA['instructions'][op][1].split(',')
            # Instructions with Fd/Ad change their first destination register.
            if signature[0]=='Fd':cached.pop(operands[0],None)
            if signature[0]=='Ad' and operands[0]=='A7':cached.clear()
        lines.append(raw)
    return '\n'.join(lines)+'\n'
