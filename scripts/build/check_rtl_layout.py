"""Validate the single RTL tree, simulation manifest and saved PDS file paths.
Use --update to regenerate source manifests after adding/removing RTL files.
Does not run synthesis or modify the PDS project.
"""
from pathlib import Path
import argparse,collections,json,re,xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[2]
BOARD=ROOT/'OV5640_DualView_100H'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--update',action='store_true')
    args=parser.parse_args()
    sources=sorted([p for p in (ROOT/'rtl').rglob('*') if p.suffix in ('.v','.sv')],
                   key=lambda p:(p.name!='lround_pkg.sv',p.as_posix()))
    board=[p for p in sources if 'board' in p.parts or 'clock' in p.parts or p.name=='calibrated_view_top.v']
    common=[p for p in sources if p not in board]
    manifest=['+incdir+rtl/include']+[p.relative_to(ROOT).as_posix() for p in common]
    excluded={s.strip() for s in (ROOT/'scripts/build/pds_excluded_sources.txt').read_text().splitlines()
              if s.strip() and not s.startswith('#')}
    expected=[p for p in common if p.relative_to(ROOT).as_posix() not in excluded]+board
    if args.update:
        (ROOT/'rtl/files.f').write_text('\n'.join(manifest)+'\n')
    actual=(ROOT/'rtl/files.f').read_text().splitlines()
    assert len(actual)==len(set(actual)), 'Duplicate manifest entry'
    assert set(actual)==set(manifest), 'RTL manifest is stale: run with --update'
    names=collections.defaultdict(list)
    for p in sources:
        text=p.read_text(encoding='utf-8-sig')
        text=re.sub(r'/\*.*?\*/|//[^\n]*','',text,flags=re.S)
        for name in re.findall(r'\bmodule\s+(\w+)',text):names[name].append(str(p))
        for include in re.findall(r'`include\s+"([^"]+)"',text):
            assert (ROOT/'rtl/include'/include).is_file() or (p.parent/include).is_file(),(p,include)
    assert all(len(paths)==1 for paths in names.values()), 'Duplicate module definition'
    for old in ('parameter/rtl','undistort/rtl','integration/rtl','algorithom/closer2fpga/rtl','OV5640_DualView_100H/source/rtl'):
        assert not any(p.suffix in ('.v','.sv','.vh') for p in (ROOT/old).rglob('*')), 'Old RTL remains: '+old
    project=ET.parse(BOARD/'DualView_OV5640.pds')
    design=project.find(".//task[@name='DESIGN_SET']")
    assert design.find("options/option[@name='top_module']").get('value')=='calibrated_view_top'
    saved=[(BOARD/p.get('file')).resolve() for p in design.findall("action[@name='design']/inputs/item[@type='FILE']")]
    assert len(saved)==len(set(saved)), 'Duplicate PDS source'
    assert all(p.is_file() for p in saved), 'Missing PDS source'
    assert set(saved)==set(expected), {'missing':[str(p) for p in set(expected)-set(saved)],
                                      'extra':[str(p) for p in set(saved)-set(expected)]}
    for p in design.findall("options/option[@name='include_path']/list/item"):
        assert (BOARD/p.get('value')).is_dir(),p.attrib
    print(json.dumps(dict(status='PASS',rtl_files=len(sources),modules=len(names),
                          pds_sources=len(saved),missing=0,duplicates=0),indent=2))

if __name__=='__main__':main()
