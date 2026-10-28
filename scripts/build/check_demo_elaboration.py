"""Slang board RTL elaboration for the combined demo profile; no EDA run.

Checks the canonical top (all optional features off) and the generated demo top
(corner review + advanced on) with vendor IP black-boxed. This is not synthesis,
timing or CDC sign-off.
"""
from pathlib import Path
import json
import shlex
import xml.etree.ElementTree as ET

import pyslang
from pyslang.driver import Driver

ROOT = Path(__file__).resolve().parents[2]
VENDOR_MODULES = {'pll', 'ddr3_test', 'GTP_INBUFGDS', 'GTP_GPLL'}


def check(demo):
    project = ROOT/'OV5640_Demo_100H/Demo_CornerReview_Advanced.pds'
    tree = ET.parse(project)
    files = [(project.parent/x.get('file')).resolve() for x in tree.findall(
        ".//task[@name='DESIGN_SET']/action[@name='design']/inputs/item[@type='FILE']")]
    if not demo:
        files = [ROOT/'rtl/top/calibrated_view_top.v' if x.name == 'project_top.v' else x for x in files]
    driver = Driver()
    driver.addStandardArgs()
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
    name = 'demo' if demo else 'canonical'
    log = ROOT/f'build/demo/elaboration_{name}.log'
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
    (ROOT/'build/demo/elaboration.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print('PASS demo board RTL canonical/demo elaboration (vendor IP black-boxed, not synthesized)')


if __name__ == '__main__':
    main()
