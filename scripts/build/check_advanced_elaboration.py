"""Slang board RTL elaboration, with explicit vendor-IP black boxes; no EDA run.

This is not a vendor synthesis/timing/CDC sign-off. Requires pyslang. Generate
the advanced PDS first. Both default-disabled and advanced-enabled tops checked.
"""
from pathlib import Path
import json
import shlex
import xml.etree.ElementTree as ET

import pyslang
from pyslang.driver import Driver

ROOT = Path(__file__).resolve().parents[2]
# Implementations provided by PDS IP libraries, intentionally not simulated here.
VENDOR_MODULES = {'pll', 'ddr3_test', 'GTP_INBUFGDS', 'GTP_GPLL'}


def check(advanced):
    project = ROOT/'OV5640_Advanced_100H/Advanced_DynamicGamma.pds'
    tree = ET.parse(project)
    files = [(project.parent/x.get('file')).resolve() for x in tree.findall(
        ".//task[@name='DESIGN_SET']/action[@name='design']/inputs/item[@type='FILE']")]
    if not advanced:
        files = [ROOT/'rtl/top/calibrated_view_top.v' if x.name == 'project_top.v' else x for x in files]
    driver = Driver()
    driver.addStandardArgs()
    # parseCommandLine consumes argv[0]; retain a program-name token.
    args = ['slang', '-I'+(ROOT/'rtl/include').as_posix(), '--timescale=1ns/1ps',
            '--top', 'calibrated_view_top']+[x.as_posix() for x in files]
    assert driver.parseCommandLine(shlex.join(args))
    assert driver.processOptions()
    assert driver.parseAllSources()
    compilation = driver.createCompilation()
    diagnostics = compilation.getAllDiagnostics()
    failures, blackboxes = [], set()
    for diag in diagnostics:
        if not diag.isError():
            continue
        if str(diag.code) == 'DiagCode(UnknownModule)' and str(diag.args[0]) in VENDOR_MODULES:
            blackboxes.add(str(diag.args[0]))
        else:
            failures.append(diag)
    name = 'enabled' if advanced else 'disabled'
    log = ROOT/f'build/advanced/elaboration_{name}.log'
    log.write_text(pyslang.DiagnosticEngine.reportAll(driver.sourceManager, diagnostics), encoding='utf-8')
    report = dict(profile=name, errors=len(failures), vendor_blackboxes=sorted(blackboxes),
                  warnings=sum(not x.isError() for x in diagnostics), diagnostic_log=str(log))
    print(json.dumps(report, indent=2))
    if failures:
        print(pyslang.DiagnosticEngine.reportAll(driver.sourceManager, failures[:30]))
        raise SystemExit(1)
    return report


def main():
    result = [check(False), check(True)]
    (ROOT/'build/advanced/elaboration.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('PASS board RTL disabled/enabled elaboration (vendor IP black-boxed, not synthesized)')


if __name__ == '__main__':
    main()
