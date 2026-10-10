"""Check RTL instruction counts against the interpreter and summarize cycles."""
import csv
import json
from assemble import ROOT

OUT=ROOT/'build/calibration_engine'
reference=json.loads((OUT/'operation_counts.json').read_text())
names={case['entry']:case['name'] for case in json.loads((OUT/'checks.json').read_text())}
with (OUT/'cycles.csv').open() as stream:
    rows=[row for row in csv.DictReader(stream) if row['entry']!='max_relative']
assert len(rows)==len(reference),(len(rows),len(reference))
report=[]
for row,expected in zip(rows,reference):
    entry=int(row['entry'])
    assert entry==expected['entry']
    assert int(row['status'],16)==expected['status']
    assert int(row['instructions'])==expected['counts']['instructions'],(entry,row,expected)
    report.append(dict(name=names[entry],cycles=int(row['cycles']),
                       milliseconds_at_40MHz=int(row['cycles'])/40000,
                       instructions=int(row['instructions']),fp_ops=expected['fp_ops']))
(OUT/'measured_cycles.json').write_text(json.dumps(report,indent=2))
print(f'ENGINE_CENSUS_PASS {len(rows)} RTL instruction counts match independent interpreter')
