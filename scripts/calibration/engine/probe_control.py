"""Measure calibration control/RAM/residual logic with the FP pool external.

Creates an isolated wrapper, preserving every public port and every FP request
and response. No constant arithmetic model: synthesis cannot discard consumers.
The production board project is not changed. Forward options to the guarded
partition synthesis runner (for example --pds PATH --timeout 300).
"""
from pathlib import Path
import re
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[3]

def main():
    source=(ROOT/'rtl/control/calibration/calib_top.v').read_text(encoding='utf-8')
    signals=[('output','4:0','shared_req_valid'),('input','4:0','shared_req_ready'),
             ('output','24:0','shared_req_op'),('output','319:0','shared_req_a'),
             ('output','319:0','shared_req_b'),('output','4:0','shared_active'),
             ('input','4:0','shared_rsp_valid'),('output','4:0','shared_rsp_ready'),
             ('input','63:0','shared_rsp_result'),('input','4:0','shared_rsp_flags')]
    for direction,width,name in signals:
        source,n=re.subn(r'\s*wire\s+\['+re.escape(width)+r'\]\s+'+name+r';','',source)
        assert n==1,name
    ports='\n'.join(f'    {direction} wire [{width}] {name},' for direction,width,name in signals)
    source=source.replace('module calib_top #(parameter EXTERNAL_FP=0)(', 'module probe_calib_control #(parameter EXTERNAL_FP=0)(\n'+ports)
    begin=source.index('    fp_calibration_pool #(')
    end=source.index('    corner_store store(',begin)
    source=source[:begin]+source[end:]
    out=ROOT/'build/calibration_engine/probe_calib_control.v'
    out.parent.mkdir(parents=True,exist_ok=True);out.write_text(source,encoding='utf-8')
    command=[sys.executable,str(ROOT/'scripts/build/check_partition_synthesis.py'),
             '--tops','probe_calib_control','--extra-source',str(out),
             '--constraints',str(ROOT/'scripts/calibration/engine/probe_40m.fdc'),
             '--memory-gib','3','--timeout','300',*sys.argv[1:]]
    raise SystemExit(subprocess.call(command,cwd=ROOT))

if __name__=='__main__':main()
