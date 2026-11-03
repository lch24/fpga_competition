"""Generate the combined demo PDS OFFLINE (corner review + advanced); no EDA.

The canonical top keeps every optional feature off. This generated profile sets
ENABLE_CORNER_REVIEW=1, ENABLE_ADVANCED=1 and ENABLE_DISTORT_DEMO=0 while keeping
module and instance names used by the FDC. Generated outputs are not sources.
"""
from datetime import datetime
from pathlib import Path
import hashlib
import json
import os
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT/'OV5640_DualView_100H'
OUT = ROOT/'OV5640_Demo_100H'
PROFILE = [('parameter ENABLE_CORNER_REVIEW = 0,', 'parameter ENABLE_CORNER_REVIEW = 1,'),
           ('parameter ENABLE_ADVANCED = 0,', 'parameter ENABLE_ADVANCED = 1,'),
           ('parameter ENABLE_DISTORT_DEMO = 1,', 'parameter ENABLE_DISTORT_DEMO = 0,')]


def local(path):
    path = path.resolve()
    if not path.is_relative_to(ROOT):
        raise ValueError(f'input escapes this worktree: {path}')
    if not path.is_file():
        raise FileNotFoundError(path)
    return os.path.relpath(path, OUT).replace('\\', '/')


def main():
    OUT.mkdir(exist_ok=True)
    generated = ROOT/'build/demo/project_top.v'
    generated.parent.mkdir(parents=True, exist_ok=True)
    canonical = (ROOT/'rtl/top/calibrated_view_top.v').read_text(encoding='utf-8-sig')
    profile = canonical
    for old, new in PROFILE:
        if profile.count(old) != 1:
            raise ValueError('canonical top changed; review demo profile generation: '+old)
        profile = profile.replace(old, new)
    generated.write_text('// GENERATED demo profile; regenerate, do not edit.\n'+profile, encoding='utf-8')
    excluded = {x.strip() for x in (ROOT/'scripts/build/pds_excluded_sources.txt').read_text().splitlines()
                if x.strip() and not x.startswith('#')}
    paths = [ROOT/x for x in (ROOT/'rtl/files.f').read_text().splitlines()
             if x and not x.startswith('+') and x not in excluded]
    paths += [ROOT/x for x in ['rtl/clock/algorithm_clock.v', 'rtl/video/board/board_ms72xx_ctl.v',
                              'rtl/video/board/board_power_on_delay.v']]
    paths += list((ROOT/'rtl/video/board').glob('*.v'))
    paths += [generated]
    paths = sorted(set(paths))
    # The copied project keeps its IP section relative to the project directory.
    shutil.copytree(BASE/'ipcore', OUT/'ipcore', dirs_exist_ok=True)
    tree = ET.parse(BASE/'DualView_OV5640.pds')
    project = tree.getroot()
    project.set('status', 'initial')
    design = project.find(".//task[@name='DESIGN_SET']")
    inputs = design.find("action[@name='design']/inputs")
    for item in list(inputs):
        if item.get('type') == 'FILE':
            inputs.remove(item)
        elif item.get('file'):
            item.set('file', local(BASE/item.get('file')))
    stamp = datetime.now().isoformat(timespec='seconds')
    for path in paths:
        fmt = 'systemverilog' if path.suffix == '.sv' else 'verilog'
        item = ET.SubElement(inputs, 'item', type='FILE', file=local(path), format=fmt, timespec=stamp, library='work')
        options = ET.SubElement(item, 'options')
        for key, value in [('format', fmt), ('library', 'work')]:
            ET.SubElement(options, 'option', name=key, type='string', value=value)
    include = design.find("options/option[@name='include_path']")
    for item in list(include):
        include.remove(item)
    ET.SubElement(ET.SubElement(include, 'list'), 'item', type='string', value='../rtl/include')
    fdc_name = 'Demo_CornerReview_Advanced.fdc'
    fdc = (BASE/'DualView_OV5640.fdc').read_text(encoding='utf-8-sig')
    fdc += '\n# Demo advanced mailbox reset assertion only; release/data remain timed.\n'
    for name in ['csr', 'ccr']:
        fdc += 'set_false_path -through [get_pins {g_realtime.g_advanced.'+name+'/arst_n}]\n'
    (OUT/fdc_name).write_text(fdc, encoding='utf-8')
    for item in project.iter('item'):
        if item.get('file') == 'DualView_OV5640.fdc':
            item.set('file', fdc_name)
    for state in project.iter('state'):
        state.set('value', 'INVALID')
    ET.indent(tree, space='    ')
    target = OUT/'Demo_CornerReview_Advanced.pds'
    tree.write(target, encoding='utf-8', xml_declaration=True)
    rom = OUT/'data/rom'
    rom.mkdir(parents=True, exist_ok=True)
    for path in (ROOT/'data/rom').glob('*.mem'):
        shutil.copy2(path, rom/path.name)
    verified = ET.parse(target).find(".//task[@name='DESIGN_SET']/action[@name='design']/inputs")
    saved = [(OUT/x.get('file')).resolve() for x in verified if x.get('file')]
    assert all(x.is_file() and x.is_relative_to(ROOT) for x in saved)
    assert len(saved) == len(set(saved))
    assert ROOT/'rtl/top/calibrated_view_top.v' not in saved
    report = dict(project=str(target), generated_top=str(generated), hdl_sources=len(paths),
                  canonical_sha256=hashlib.sha256(canonical.encode()).hexdigest(),
                  features=dict(ENABLE_CORNER_REVIEW=1, ENABLE_ADVANCED=1, ENABLE_DISTORT_DEMO=0),
                  input_paths_local=True, eda_launched=False, hardware_accessed=False)
    (ROOT/'build/demo/project_configuration.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
