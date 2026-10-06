"""Bounded, isolated PDS check. Does not launch the full board design."""
import ctypes
from ctypes import wintypes
from pathlib import Path
import subprocess
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--pds', default=__import__('os').environ.get('PDS_SHELL','pds_shell.exe'))
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
build = root / 'build/system'
build.mkdir(exist_ok=True)

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
job = kernel.CreateJobObjectW(None, None)
limits = JobLimits()
limits.Basic.Flags = 0x2000 | 0x200  # KILL_ON_JOB_CLOSE | JOB_MEMORY
limits.JobMemory = 4 * 1024**3
if not job or not kernel.SetInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits)):
    raise ctypes.WinError(ctypes.get_last_error())
with (build / 'sqrt_pds_console.log').open('w') as output:
    proc = subprocess.Popen([args.pds, '-file', str(root / 'scripts/build/check_sqrt_pds.tcl')],
                            cwd=build, stdout=output, stderr=subprocess.STDOUT,
                            creationflags=subprocess.CREATE_NO_WINDOW)
    start = time.monotonic()
    peak = 0
    try:
        if not kernel.AssignProcessToJobObject(job, int(proc._handle)):
            raise ctypes.WinError(ctypes.get_last_error())
        while proc.poll() is None:
            stats = Counters()
            stats.cb = ctypes.sizeof(stats)
            if not memory_info(int(proc._handle), ctypes.byref(stats), stats.cb):
                raise RuntimeError('Cannot monitor isolated PDS memory')
            peak = max(peak, stats.PrivateUsage)
            if peak > 4 * 1024**3 or time.monotonic() - start > 120:
                raise RuntimeError('Isolated PDS exceeded 4 GiB / 120 second limit')
            time.sleep(0.25)
        if proc.returncode:
            raise RuntimeError(f'PDS exit {proc.returncode}; inspect sqrt_pds_console.log')
    finally:
        if proc.poll() is None:
            kernel.TerminateJobObject(job, 1)
            proc.kill()
        proc.wait()
        kernel.QueryInformationJobObject(job, 9, ctypes.byref(limits), ctypes.sizeof(limits), None)
        kernel.CloseHandle(job)
console = (build / 'sqrt_pds_console.log').read_text(errors='replace')
if 'Executing : synthesize -ads successfully.' not in console:
    raise RuntimeError('PDS did not report successful synthesis')
print(f'Isolated PDS exit=0 elapsed={time.monotonic()-start:.1f}s peak_job_MiB={limits.PeakJobMemory/1024**2:.1f}')
