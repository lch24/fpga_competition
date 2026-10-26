"""Generate the isolated review PDS OFFLINE. No EDA, UART or board operations.

Keep one canonical top; derive a profile changing only parameter defaults, so
all existing PLL/PHY hierarchy constraints retain their original instance names.
Re-run whenever the top changes. Generated files are not committed.
"""
from pathlib import Path
import hashlib
import json
import shutil
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT/'OV5640_DualView_100H'


def main():
    source = ROOT/'rtl/top/calibrated_view_top.v'
    canonical = source.read_text(encoding='utf-8-sig')
    profile = canonical
    for old, new in [('parameter ENABLE_CORNER_REVIEW = 0,', 'parameter ENABLE_CORNER_REVIEW = 1,'),
                     ('parameter ENABLE_DISTORT_DEMO = 1,', 'parameter ENABLE_DISTORT_DEMO = 0,')]:
        if profile.count(old) != 1:
            raise RuntimeError('Top changed: review profile generator needs updating')
        profile = profile.replace(old, new)
    build = ROOT/'build/review'
    build.mkdir(parents=True, exist_ok=True)
    generated = build/'project_top.v'
    generated.write_text('// GENERATED; regenerate using prepare_corner_review.py\n'+profile, encoding='utf-8')
    tree = ET.parse(BOARD/'DualView_OV5640.pds')
    design = tree.find(".//task[@name='DESIGN_SET']")
    for item in design.findall("action[@name='design']/inputs/item[@type='FILE']"):
        path = (BOARD/item.get('file')).resolve()
        assert path.is_relative_to(ROOT) and path.is_file(), path
        if path == source:
            item.set('file', '../build/review/project_top.v')
    include = design.find("options/option[@name='include_path']/list")
    for item in list(include):
        include.remove(item)
    ET.SubElement(include, 'item', type='string', value='../rtl/include')
    for state in tree.iter('state'):
        state.set('value', 'INVALID')
    target = BOARD/'CornerReview.pds'
    ET.indent(tree, space='    ')
    tree.write(target, encoding='utf-8', xml_declaration=True)
    rom = BOARD/'data/rom'
    rom.mkdir(parents=True, exist_ok=True)
    for path in (ROOT/'data/rom').glob('*.mem'):
        shutil.copy2(path, rom/path.name)
    report = dict(project=str(target), canonical_sha256=hashlib.sha256(canonical.encode()).hexdigest(),
                  ENABLE_CORNER_REVIEW=1, ENABLE_DISTORT_DEMO=0, eda_launched=False, hardware_accessed=False)
    (build/'configuration.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
