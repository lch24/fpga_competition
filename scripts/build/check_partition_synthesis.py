"""Isolated PDS synthesis diagnostics with process-tree memory and time limits.
Leaves board project and production RTL untouched; writes build/system only.
Optional --constraints runs timing with a copied FDC in the isolated project.
"""
import argparse
import ctypes
from ctypes import wintypes
import json
import re
from pathlib import Path
import subprocess
import time
import shutil
import hashlib

ROOT = Path(__file__).resolve().parents[2]
class Counters(ctypes.Structure):
    _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD)] + [
        (name, ctypes.c_size_t) for name in ('PeakWorkingSetSize', 'WorkingSetSize',
        'QuotaPeakPagedPoolUsage', 'QuotaPagedPoolUsage', 'QuotaPeakNonPagedPoolUsage',
        'QuotaNonPagedPoolUsage', 'PagefileUsage', 'PeakPagefileUsage', 'PrivateUsage')]

memory_info = ctypes.WinDLL('psapi').GetProcessMemoryInfo
memory_info.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
memory_info.restype = wintypes.BOOL
# Windows job limits include PDS worker children, not just the launcher.
class BasicLimits(ctypes.Structure):
    _fields_ = [('ProcessTime', ctypes.c_int64), ('JobTime', ctypes.c_int64),
                ('Flags', wintypes.DWORD), ('MinWS', ctypes.c_size_t),
                ('MaxWS', ctypes.c_size_t), ('ActiveLimit', wintypes.DWORD),
                ('Affinity', ctypes.c_size_t), ('Priority', wintypes.DWORD),
                ('Scheduling', wintypes.DWORD)]

class JobLimits(ctypes.Structure):
    _fields_ = [('Basic', BasicLimits), ('IO', ctypes.c_uint64 * 6),
                ('ProcessMemory', ctypes.c_size_t), ('JobMemory', ctypes.c_size_t),
                ('PeakProcessMemory', ctypes.c_size_t), ('PeakJobMemory', ctypes.c_size_t)]

kernel = ctypes.WinDLL('kernel32', use_last_error=True)
kernel.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
kernel.CreateJobObjectW.restype = wintypes.HANDLE
kernel.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
kernel.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
kernel.TerminateJobObject.argtypes = [wintypes.HANDLE, wintypes.UINT]
kernel.QueryInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD, ctypes.c_void_p]
kernel.CloseHandle.argtypes = [wintypes.HANDLE]

class MemoryStatus(ctypes.Structure):
    _fields_ = [('Length', wintypes.DWORD), ('Load', wintypes.DWORD)] + [
        (name, ctypes.c_uint64) for name in ('TotalPhysical', 'AvailablePhysical',
        'TotalPage', 'AvailablePage', 'TotalVirtual', 'AvailableVirtual', 'Extended')]

kernel.GlobalMemoryStatusEx.argtypes = [ctypes.POINTER(MemoryStatus)]

def run_one(top, batch, args):
    case = batch / top
    case.mkdir()
    shutil.copytree(ROOT / 'data/rom', case / 'data/rom', dirs_exist_ok=True)
    sources = []
    for folder in ('rtl',):
        sources.extend(p for p in (ROOT / folder).rglob('*') if p.suffix in ('.v', '.sv') and 'board' not in p.parts and p.name != 'calibrated_view_top.v')
    # Freeze this run's RTL/headers. Later edits cannot change an in-flight run.
    hashes = {}
    for folder in ('rtl',):
        for source in (ROOT / folder).rglob('*'):
            if source.is_file() and source.suffix in ('.v', '.sv', '.vh', '.svh'):
                relative = source.relative_to(ROOT)
                destination = case / 'source' / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                content = source.read_bytes()
                destination.write_bytes(content)
                hashes[relative.as_posix()] = hashlib.sha256(content).hexdigest()
    (case / 'source_sha256.json').write_text(json.dumps(hashes, indent=2))
    sources = [case / 'source' / p.relative_to(ROOT) for p in sources]
    # Optional isolated reference implementations for matched A/B synthesis.
    for extra in args.extra_source:
        source = Path(extra).resolve()
        destination = case / 'source' / 'reference' / source.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        sources.append(destination)
        hashes['reference/' + source.name] = hashlib.sha256(destination.read_bytes()).hexdigest()
    (case / 'source_sha256.json').write_text(json.dumps(hashes, indent=2))
    # PDS changes its working directory during compile. Resolve ROM paths in
    # the frozen diagnostic copies only; production RTL remains portable.
    # Preserve hashes of both production inputs and the actual compiled copies.
    compiled_hashes = {}
    for source in sources:
        content=source.read_text(encoding='utf-8-sig')
        for rom in (case/'data/rom').glob('*.mem'):
            content=content.replace('"data/rom/'+rom.name+'"', '"'+rom.as_posix()+'"')
        source.write_text(content,encoding='utf-8')
        compiled_hashes[source.relative_to(case/'source').as_posix()]=hashlib.sha256(source.read_bytes()).hexdigest()
    (case/'compiled_sha256.json').write_text(json.dumps(compiled_hashes,indent=2))
    sources.sort(key=lambda p: (p.name != 'lround_pkg.sv', str(p)))
    quote = lambda p: '{' + Path(p).as_posix() + '}'
    script = [f'create_project {quote(case / "project" / "test.pds")} -synthesize_tool 2 -family Logos2 -device PG2L100H -package FBG676 -speedgrade -6 -in_process',
              'set_option verilog_standard SystemVerilog [get_filesets design_1]']
    script += [f'add_design -verilog {quote(p)}' for p in sources]
    profiles = {'probe_fp_basic': (0, 0, 0, 0), 'probe_fp_log': (0, 1, 0, 0),
                'probe_fp_exp': (1, 0, 0, 0), 'probe_fp_trig': (0, 0, 1, 1),
                'probe_fp_sincos': (0, 0, 1, 0), 'probe_fp_atan': (0, 0, 0, 1)}
    if top in profiles:
        exp, log, sincos, atan = profiles[top]
        wrapper = case / 'wrapper.v'
        wrapper.write_text(f'''module {top} (
input wire clk,rst_n,req_valid, output wire req_ready,
input wire [4:0] req_op, input wire [63:0] req_a,req_b,
output wire rsp_valid, input wire rsp_ready, output wire [63:0] rsp_result,
output wire [4:0] rsp_flags, output wire rsp_less,rsp_equal,rsp_unordered);
fp_operator #(.FP_W(64), .ENABLE_EXP({exp}), .ENABLE_LOG({log}),
.ENABLE_SINCOS({sincos}), .ENABLE_ATAN_ACOS({atan})) dut (
.clk(clk),.rst_n(rst_n),.req_valid(req_valid),.req_ready(req_ready),
.req_op(req_op),.req_a(req_a),.req_b(req_b),.rsp_valid(rsp_valid),
.rsp_ready(rsp_ready),.rsp_result(rsp_result),.rsp_flags(rsp_flags),
.rsp_less(rsp_less),.rsp_equal(rsp_equal),.rsp_unordered(rsp_unordered));
endmodule
''')
        script.append(f'add_design -verilog {quote(wrapper)}')
    if top == 'probe_gray_ram':
        wrapper = case / 'wrapper.v'
        wrapper.write_text('''module probe_gray_ram (
input wire clk,wr_en,rd_en, input wire [20:0] wr_addr,rd_addr,
input wire [7:0] wr_data, output reg [7:0] rd_data);
reg [7:0] gray_mem [0:2097151];
always @(posedge clk) begin
  if(wr_en) gray_mem[wr_addr] <= wr_data;
  if(rd_en) rd_data <= gray_mem[rd_addr];
end
endmodule
''')
        script.append(f'add_design -verilog {quote(wrapper)}')
    if top in ('probe_detector_compact','probe_detector_fixed'):
        source = (case / 'source/rtl/image/features/corner_detect_ddr_top.v').read_text(encoding='utf-8')
        start = source.index('module corner_detect_ddr_top')
        end = source.index('\n);', start) + 3
        header = source[start:end].replace('module corner_detect_ddr_top', 'module '+top, 1)
        wrapper = case / 'wrapper.v'
        fixed=',.FIXED_BILINEAR(1),.FIXED_ACCUM(1)' if top=='probe_detector_fixed' else ''
        wrapper.write_text(header + '\ncorner_detect_ddr_top #(.DDR_CANDIDATES(1),.DDR_GRAY(1),.REPLAY_RESP(1)'+fixed+') dut(.*);\nendmodule\n', encoding='utf-8')
        script.append(f'add_design -verilog {quote(wrapper)}')
    if top == 'probe_ddr_service3':
        source=(case/'source/rtl/memory/ddr/ddr_service.v').read_text(encoding='utf-8')
        start=source.index('module ddr_service');end=source.index('\n);',start)+3
        header=source[start:end].replace('module ddr_service','module probe_ddr_service3',1).replace('parameter CLIENTS=4','parameter CLIENTS=3',1)
        wrapper=case/'wrapper.v'
        wrapper.write_text(header+'\nddr_service #(.CLIENTS(3)) dut(.*);\nendmodule\n')
        script.append(f'add_design -verilog {quote(wrapper)}')
    if args.constraints:
        constraint=case/'probe.fdc'
        shutil.copy2(args.constraints,constraint)
        script.append(f'add_constraint {quote(constraint)}')

    includes = [case / 'source' / p for p in ('rtl/include', 'rtl/compute/float', 'rtl/include')]
    script += ['set_option include_path [list ' + ' '.join(map(quote, includes)) + '] [get_filesets design_1]',
               f'compile -top_module {top} -fsm_compiler FALSE']
    synthesis_command = 'synthesize -ads'
    if args.resource_sharing != 'default':
        synthesis_command += ' -resource_sharing ' + args.resource_sharing.upper()
    if args.partition:
        synthesis_command += ' -automatic_partition_block TRUE -automatic_compile_point TRUE -max_parallel_jobs 1'
    script += [synthesis_command, 'exit']
    tcl = case / 'run.tcl'
    tcl.write_text('\n'.join(script) + '\n')
    job = kernel.CreateJobObjectW(None, None)
    limits = JobLimits()
    limits.Basic.Flags = 0x2000 | 0x200
    limits.JobMemory = int(args.memory_gib * 1024**3)
    if not job or not kernel.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
        raise ctypes.WinError(ctypes.get_last_error())
    started = time.monotonic()
    reason = ''
    minimum_physical = minimum_commit = float('inf')
    with (case / 'console.log').open('w') as output:
        proc = subprocess.Popen([args.pds, '-file', str(tcl)], cwd=case,
                                stdout=output, stderr=subprocess.STDOUT,
                                creationflags=subprocess.CREATE_NO_WINDOW)
        try:
            if not kernel.AssignProcessToJobObject(job, int(proc._handle)):
                raise ctypes.WinError(ctypes.get_last_error())
            while proc.poll() is None:
                memory = MemoryStatus()
                memory.Length = ctypes.sizeof(memory)
                if not kernel.GlobalMemoryStatusEx(ctypes.byref(memory)):
                    raise ctypes.WinError(ctypes.get_last_error())
                minimum_physical = min(minimum_physical, memory.AvailablePhysical)
                minimum_commit = min(minimum_commit, memory.AvailablePage)
                # Some PDS workers remain alive after their first allocation
                # failure. Stop the job immediately rather than waiting minutes.
                with (case / 'console.log').open('rb') as current_log:
                    current_log.seek(0, 2)
                    current_log.seek(max(0, current_log.tell() - 65536))
                    allocation_failed = b'Memory alloc failed' in current_log.read()
                if allocation_failed:
                    reason = 'MEMORY_LIMIT_OR_ALLOC_FAILURE'
                elif time.monotonic() - started > args.timeout:
                    reason = 'TIME_LIMIT'
                elif memory.AvailablePhysical < 1536 * 1024**2 or memory.AvailablePage < 2 * 1024**3:
                    reason = 'SYSTEM_PHYSICAL_GUARD' if memory.AvailablePhysical < 1536 * 1024**2 else 'SYSTEM_COMMIT_GUARD'
                if reason:
                    kernel.TerminateJobObject(job, 1)
                    break
                time.sleep(0.5)
            proc.wait(timeout=15)
        finally:
            if proc.poll() is None:
                kernel.TerminateJobObject(job, 1)
                proc.kill()
                proc.wait()
            kernel.QueryInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits), None)
            kernel.CloseHandle(job)
    console = (case / 'console.log').read_text(errors='replace')
    logs = list((case / 'project').rglob('run.log'))
    all_text = console + '\n' + '\n'.join(p.read_text(errors='replace') for p in logs)
    status = reason or ('MISSING_ROM' if 'Cannot open specified data file' in all_text
                        else 'PASS' if proc.returncode == 0 and re.search(r'Executing : synthesize -ads[^\r\n]* successfully\.', console)
                        else 'UNSUPPORTED_RTL' if 'Unsupported RTL design' in all_text
                        else 'MEMORY_LIMIT_OR_ALLOC_FAILURE' if 'Memory alloc failed' in all_text
                        else 'TOOL_FAILURE')
    stages = re.findall(r'^Start ([^\r\n]+)', console, re.MULTILINE)
    errors = [line for line in console.splitlines() if line.startswith('E:')]
    result = dict(top=top, status=status, seconds=round(time.monotonic()-started, 1),
                  peak_job_mib=round(limits.PeakJobMemory/1024**2, 1), exit=proc.returncode,
                  min_available_physical_mib=round(minimum_physical/1024**2, 1),
                  min_available_commit_mib=round(minimum_commit/1024**2, 1),
                  memory_limit_gib=args.memory_gib, timeout=args.timeout,
                  resource_sharing=args.resource_sharing, automatic_partition=args.partition,
                  pre_mapping_done='Executing : pre-mapping successfully.' in all_text,
                  mod_gen_started='Start mod-gen.' in all_text,
                  last_stage=stages[-1] if stages else 'see console', errors=errors,
                  directory=str(case))
    (case / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result), flush=True)
    return result

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pds', default=__import__('os').environ.get('PDS_SHELL','pds_shell.exe'))
    parser.add_argument('--tops', nargs='+', default=['undistort_top', 'calib_top', 'corner_detect_ddr_top'])
    parser.add_argument('--memory-gib', type=float, default=6)
    parser.add_argument('--timeout', type=int, default=600)
    parser.add_argument('--resource-sharing', choices=['default','true','false'], default='default')
    parser.add_argument('--partition', action='store_true')
    parser.add_argument('--extra-source', nargs='*', default=[])
    parser.add_argument('--constraints', type=Path)
    args = parser.parse_args()
    batch = ROOT / 'build/system' / ('partition_pds_' + str(int(time.time())))
    batch.mkdir(parents=True)
    print('BATCH ' + str(batch), flush=True)
    results = []
    for top in args.tops:
        if not top.replace('_', '').isalnum():
            raise ValueError('Invalid module name')
        print('START ' + top, flush=True)
        results.append(run_one(top, batch, args))
        (batch / 'results.json').write_text(json.dumps(results, indent=2))

    if any(result["status"] != "PASS" for result in results):
        raise SystemExit(1)
