"""RTL elaboration in pyslang, not vendor synthesis/timing/physical CDC sign-off."""
from pathlib import Path
import json
import shlex
import xml.etree.ElementTree as ET
import pyslang
from pyslang.driver import Driver

ROOT = Path(__file__).resolve().parents[2]
VENDOR = {'pll', 'ddr3_test', 'GTP_GPLL', 'GTP_INBUFGDS'}


def check(enabled):
    project = ROOT/'OV5640_DualView_100H/CornerReview.pds'
    files = [(project.parent/x.get('file')).resolve() for x in ET.parse(project).findall(
        ".//task[@name='DESIGN_SET']/action[@name='design']/inputs/item[@type='FILE']")]
    if not enabled:
        files = [ROOT/'rtl/top/calibrated_view_top.v' if p.name == 'project_top.v' else p for p in files]
    driver = Driver()
    driver.addStandardArgs()
    args = ['slang', '-I'+(ROOT/'rtl/include').as_posix(), '--timescale=1ns/1ps', '--top', 'calibrated_view_top']
    args += [p.as_posix() for p in files]
    assert driver.parseCommandLine(shlex.join(args)) and driver.processOptions() and driver.parseAllSources()
    diagnostics = driver.createCompilation().getAllDiagnostics()
    failures, blackboxes = [], set()
    for d in diagnostics:
        if d.isError():
            if str(d.code) == 'DiagCode(UnknownModule)' and str(d.args[0]) in VENDOR:
                blackboxes.add(str(d.args[0]))
            else:
                failures.append(d)
    name = 'enabled' if enabled else 'disabled'
    (ROOT/f'build/review/elaboration_{name}.log').write_text(
        pyslang.DiagnosticEngine.reportAll(driver.sourceManager, diagnostics), encoding='utf-8')
    result = dict(profile=name, errors=len(failures), vendor_blackboxes=sorted(blackboxes),
                  warnings=sum(not d.isError() for d in diagnostics))
    print(json.dumps(result, indent=2))
    if failures:
        print(pyslang.DiagnosticEngine.reportAll(driver.sourceManager, failures[:20]))
        raise SystemExit(1)


if __name__ == '__main__':
    check(False)
    check(True)
    print('PASS corner review disabled/enabled board RTL elaboration; vendor IP black-boxed')
