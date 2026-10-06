"""Check maintained test layout and relocatable source/data references; no simulator needed."""
from pathlib import Path
import ast
import json
import re

ROOT = Path(__file__).resolve().parents[2]

def main():
    benches = list((ROOT / 'tb').rglob('tb_*.sv')) + list((ROOT / 'tb').rglob('tb_*.v'))
    assert benches, 'No testbenches'
    for old in ('parameter/sim', 'integration/tb', 'undistort/tb', 'algorithom/closer2fpga/sim'):
        assert not list((ROOT / old).glob('tb_*.*')), f'Testbenches remain in {old}'
    for relative in json.loads((ROOT / 'scripts/system/integration_tests.json').read_text()):
        assert (ROOT / relative).is_file(), relative
    for tree in ('scripts', 'data/generators'):
        for p in (ROOT / tree).rglob('*'):
            if not p.is_file() or p.suffix not in ('.ps1', '.py', '.js', '.cmd', '.cpp', '.tcl'):
                continue
            text = p.read_text(encoding='utf-8-sig')
            assert not re.search(r'(?i)[A-Z]:[/\\]fpga[/\\]', text), f'Hardcoded checkout: {p}'
            if p.suffix == '.py': ast.parse(text, filename=str(p))
            if p.suffix == '.cpp':
                for include in re.findall(r'#include "([^"]+)"', text):
                    assert (p.parent / include).is_file(), (p, include)
    # Calibration tests execute from build/<function>/build_<test>.
    for group in ('calibration', 'compute', 'memory'):
        for p in (ROOT / 'scripts' / group).glob('*.ps1'):
            for rel in re.findall(r'(?<![\w/])\.\./\.\./\.\./([\w/.-]+\.(?:sv|v|do|cpp|js|f))', p.read_text()):
                assert (ROOT / rel).is_file(), (p, rel)
    for p in (ROOT / 'tb').rglob('*.sv'):
        for rel in re.findall(r'"\.\./\.\./(?:\.\./)?(data/[^"$]+)"', p.read_text(encoding='utf-8')):
            # Historical image vectors are generated on demand and aren't required for the quick suite.
            if rel.startswith('data/image/') or '%' in rel: continue
            assert (ROOT / rel).is_file(), (p, rel)
    print(json.dumps({'status': 'PASS', 'testbench_files': len(benches),
                      'layout': ['tb', 'scripts', 'data'], 'hardcoded_checkout_paths': 0}, indent=2))

if __name__ == '__main__':
    main()
