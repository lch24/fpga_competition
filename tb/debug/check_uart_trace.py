"""Check the host decoder against bytes emitted by the real RTL UART TB."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts/debug'))
from uart_monitor import Framer, Monitor, decode, self_test
import binascii
import struct

self_test()
trace = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / 'build/system/uart_packets.hex'
rows = []
for line in trace.read_text().splitlines():
    frames = Framer().feed(bytes.fromhex(line))
    assert len(frames) == 1
    rows.append(decode(frames[0]))
assert any(r.get('params') and r['params']['fx'] for r in rows)
assert any(r.get('rms') == 1.0 for r in rows)
assert any(r.get('corner_xy') == (1.0, 2.0) for r in rows)
assert any(r['algorithm_stale'] for r in rows)
assert any(r['core_stale'] for r in rows)
assert any(r['camera_clock_stale'] for r in rows)
assert any(r['camera_frame_count_mod256'] == 1 for r in rows)
assert any(r['rx_errors'] >= 3 for r in rows)
assert any(r['rejected_queries'] > 0 for r in rows)
progress = next(r['progress'] for r in rows if 'progress' in r)
assert progress['last_accepted_steps'] == 7
assert progress['instructions'] == 0x98765432
assert progress['shared_program_pc'] == 0x3456
assert progress['last_detect_count'] == 40 and progress['last_detect_view'] == 2
assert any(r['uptime_ms'] >= 1000 for r in rows)

# No algorithm clock: a status reply with an unmatched old page must remain
# visible to the operator, rather than looking like the UART itself timed out.
old_page = bytearray.fromhex(trace.read_text().splitlines()[0])
old_page[3] = 0x53
old_page[12] |= 8
old_page[13] &= ~8
old_page[126:128] = b'\x00\x00'
old_page[128:] = struct.pack('<H', binascii.crc_hqx(old_page[2:128], 0xffff))


class FakePort:
    def write(self, data):
        pass

    def read(self, count):
        return old_page


answer = Monitor(FakePort()).query(0x53)
assert answer['algorithm_stale'] and not answer['query_page_valid']
print(f'PASS host decoder against {len(rows)} RTL UART packets')
