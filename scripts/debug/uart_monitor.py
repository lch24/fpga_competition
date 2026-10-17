"""Board diagnostics: binary UART framing, readable JSON, optional corner export.

python scripts/debug/uart_monitor.py COM5
python scripts/debug/uart_monitor.py COM5 --once progress
python scripts/debug/uart_monitor.py COM5 --corners data/debug/corners.json
Only the live port requires pyserial; --self-test uses the standard library.
"""
import argparse
import binascii
import json
from pathlib import Path
import struct
import time

UI = ['idle', 'drain_display', 'capture_command', 'capture', 'start_algorithm',
      'offer_frame', 'wait_frame_release', 'wait_result', 'live_video', 'drain_video', 'demo_mode_change', 'demo_photo_processing', 'camera_preview']
ALGORITHM = ['idle', 'collect', 'wait_frame', 'gray_command', 'gray_wait',
             'detect_start', 'detect_arm', 'detect_wait', 'view_response',
             'release_frame', 'calibration_command', 'calibration_wait',
             'map_command', 'map_wait', 'remap_command', 'remap_wait', 'response']
PARAMS = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3', 'p1', 'p2']


class Framer:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while len(self.buffer) >= 8:
            p = self.buffer.find(b'\xa5\x5a')
            if p < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            del self.buffer[:p]
            if len(self.buffer) < 8:
                break
            if self.buffer[2] != 1 or self.buffer[6:8] != b'\x78\x00':
                del self.buffer[0]
                continue
            if len(self.buffer) < 130:
                break
            frame = bytes(self.buffer[:130])
            if binascii.crc_hqx(frame[2:128], 0xffff) != int.from_bytes(frame[128:], 'little'):
                self.bad_crc += 1
                del self.buffer[0]
                continue
            del self.buffer[:130]
            frames.append(frame)
        return frames


def decode(frame):
    p = frame[8:128]
    u32 = lambda b, n: struct.unpack_from('<I', b, n)[0]
    core, alg = p[24:56], p[56:120]
    word = u32(core, 0)
    tag, = struct.unpack_from('<H', alg, 62)
    flag = alg[6]
    result = {
        'sequence': int.from_bytes(frame[4:6], 'little'), 'reply': chr(frame[3]),
        'uptime_ms': u32(p, 0), 'health_bits': p[6],
        'core_age_ms': u32(p, 8), 'algorithm_age_ms': u32(p, 12),
        'camera_clock_stale': bool(p[4] & 16), 'camera_seen': bool(p[4] & 32),
        'camera_frame_count_mod256': p[7],
        'core_stale': bool(p[4] & 4), 'algorithm_stale': bool(p[4] & 8),
        'core_seen': bool(p[5] & 2), 'algorithm_seen': bool(p[5] & 4),
        'page_matches': bool(p[5] & 8),
        'rx_errors': u32(p, 16), 'rejected_queries': u32(p, 20),
        'ui_phase': UI[(word >> 12) & 15] if ((word >> 12) & 15) < len(UI) else 'unknown',
        'camera_configured': bool(word & (1 << 24)),
        'display_corrected': bool(word & (1 << 26)),
        'video_active': bool(word & (1 << 27)), 'map_ready': bool(word & (1 << 28)),
        'distort_mode': bool(word & (1 << 29)), 'mode_pending': bool(word & (1 << 30)),
        'demo_map_ready': bool(word & (1 << 31)),
        'status': (word >> 16) & 255, 'selected': (word >> 10) & 3,
        'captured_mask': (word >> 7) & 7, 'calibrated': bool(word & 64),
        'corrected': bool(word & 32), 'busy': bool(word & 16),
        'display_enabled': bool(word & 8),
        'ui_events': u32(core, 4), 'ui_dwell_cycles': u32(core, 8),
        'ddr_read_requests': u32(core, 12), 'ddr_return_activity': u32(core, 16),
        'ddr_write_requests': u32(core, 20), 'last_hmic_word_address': u32(core, 24),
        'core_reset_released': bool(core[28] & 1),
        'sticky_display_error': bool(core[28] & 2), 'sticky_underflow': bool(core[28] & 4),
        'first_ui_error': core[29], 'ignored_busy_keys': int.from_bytes(core[30:32], 'little'),
        'calibration_job': u32(alg, 0),
        'algorithm_phase': ALGORITHM[alg[4]] if alg[4] < len(ALGORITHM) else 'unknown',
        'view': alg[5], 'algorithm_reset_released': bool(flag & 1),
        'params_valid': bool(flag & 2), 'metrics_valid': bool(flag & 4),
        'converged': bool(flag & 8), 'calibration_phase': alg[7],
        'result_events': u32(alg, 8), 'last_result_phase': alg[13],
        'last_detect_flags': alg[12],
        'last_status': alg[14], 'last_error': alg[15],
        'corner_count': int.from_bytes(alg[60:62], 'little'), 'page': tag,
    }
    if frame[3] in (0x56, 0x45) and result['page_matches']:
        for key in ('ui_dwell_cycles', 'ddr_read_requests', 'ddr_return_activity',
                    'ddr_write_requests', 'last_hmic_word_address', 'core_reset_released',
                    'sticky_display_error', 'sticky_underflow', 'first_ui_error', 'ignored_busy_keys'):
            result.pop(key, None)
        cycles = u32(core, 28)
        result['video'] = dict(captured_frames=u32(core, 8), corrected_frames=u32(core, 12),
            displayed_new_frames=u32(core, 16), failed_frames=u32(core, 20),
            skipped_camera_frames_since_reset=u32(core, 24), last_remap_cycles=cycles,
            last_remap_ms=cycles / 125000.0)
    if frame[3] == 0x45 and result['page_matches']:
        result.pop('video', None)
        stage = u32(core, 8)
        result['demo'] = dict(mode_state=(stage >> 4) & 7, capture_state=(stage >> 7) & 7,
            remap_state=(stage >> 10) & 3, first_pass=bool(stage & 4), photo_job=bool(stage & 8),
            error=(stage >> 12) & 255, first_pass_cycles=u32(core, 12), second_pass_cycles=u32(core, 16),
            total_cycles=u32(core, 20), failed_passes=u32(core, 24), build_elapsed_ms=u32(core, 28))
    if tag == 0xffff:
        result['params'] = dict(zip(PARAMS, struct.unpack_from('<9f', alg, 16))) if flag & 2 else None
        result['rms'] = struct.unpack_from('<d', alg, 52)[0] if flag & 4 else None
    elif tag == 0xfffe:
        result['progress'] = {
            'lm_state': alg[16], 'lm_completed_outer_iterations': alg[17],
            'shared_program_pc': int.from_bytes(alg[18:20], 'little'),
            'calibration_controller_pc': alg[20] & 15,
            'instructions': u32(alg, 21),
            'ddr_handshake_bits': int.from_bytes(alg[25:27], 'little'),
            'read_requests': u32(alg, 27), 'read_beats': u32(alg, 31),
            'write_requests': u32(alg, 35), 'last_read_address': u32(alg, 39),
            'last_detect_count': int.from_bytes(alg[43:45], 'little'),
            'last_detect_view': alg[45],
            'last_accepted_steps': int.from_bytes(alg[46:48], 'little'),
            'last_error_job': u32(alg, 48),
        }
    elif tag not in (0xfffd, 0xfffc):
        result['corner_valid'] = bool(flag & 16)
        result['corner_xy'] = struct.unpack_from('<2f', alg, 16) if flag & 16 else None
    return result


def command(op, index=0):
    lo, hi = index & 255, (index >> 8) & 255
    return bytes([0xa5, 0x5a, op, lo, hi, op ^ lo ^ hi])


class Monitor:
    def __init__(self, port):
        self.port, self.framer = port, Framer()

    def read(self):
        return [decode(f) for f in self.framer.feed(self.port.read(1024))]

    def query(self, op, index=0):
        tag = 0xffff if op == 0x53 else 0xfffe if op == 0x44 else 0xfffd if op == 0x56 else 0xfffc if op == 0x45 else index
        # Bounded retries also handle a busy notification/query slot.
        for _ in range(3):
            self.port.write(command(op, index))
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                for result in self.read():
                    if result['reply'] == chr(op):
                        # A dead source can still answer with an old page. Return
                        # its explicit validity instead of hiding the diagnosis
                        # behind a host timeout or interpreting it as the request.
                        result['requested_page'] = tag
                        result['query_page_valid'] = result['page_matches'] and result['page'] == tag
                        return result
        raise TimeoutError('No matching UART response; check port, clock, baud and snapshot validity')


def self_test():
    payload = bytearray(120)
    payload[118:120] = b'\xff\xff'
    header = b'\xa5\x5a\x01\x53\x34\x12\x78\x00'
    data = header + payload
    good = data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff))
    bad = bytearray(good); bad[90] ^= 1
    framer = Framer()
    assert not framer.feed(b'noise' + bad + good[:13])
    frames = framer.feed(good[13:] + good)
    assert len(frames) == 2 and framer.bad_crc == 1
    assert decode(frames[0])['sequence'] == 0x1234
    assert command(0x43, 0x1234) == bytes([0xa5, 0x5a, 0x43, 0x34, 0x12, 0x65])
    payload[5] = 8  # Both requested pages match.
    struct.pack_into('<8I', payload, 24, (1 << 27) | (1 << 28), 17, 90, 89, 88, 1, 2, 3000000)
    payload[118:120] = b'\xfd\xff'
    video_header = header[:3] + b'\x56' + header[4:]
    data = video_header + payload
    result = decode(data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff)))
    assert result['video_active'] and result['map_ready']
    assert result['video']['corrected_frames'] == 89 and result['video']['displayed_new_frames'] == 88
    assert result['video']['last_remap_ms'] == 24.0 and 'ddr_read_requests' not in result
    payload[5] = 0
    data = video_header + payload
    assert 'video' not in decode(data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff)))
    # Demo counters share the normal core state/event words, never the params page.
    payload[5] |= 8
    struct.pack_into('<H', payload, 118, 0xfffc)
    struct.pack_into('<I', payload, 24, (1 << 29) | (1 << 31))
    for offset, value in ((36, 100), (40, 110), (44, 210), (48, 3), (52, 900)):
        struct.pack_into('<I', payload, offset, value)
    data = header[:3] + b'\x45' + header[4:] + payload
    result = decode(data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff)))
    assert result['distort_mode'] and result['demo_map_ready']
    assert result['demo']['total_cycles'] == 210 and result['demo']['failed_passes'] == 3
    print('PASS UART decoder: fragmentation, noise, CRC rejection, resynchronization, video counters/page validity')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', nargs='?')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--once', choices=['status', 'progress', 'video', 'demo'])
    parser.add_argument('--video-seconds', type=float, help='Measure NEW-frame counter deltas over this interval')
    parser.add_argument('--corners', type=Path)
    parser.add_argument('--log', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test(); return
    if not args.port:
        parser.error('Specify a serial port, e.g. COM5')
    try:
        import serial
    except ImportError:
        parser.error('Install the host dependency: python -m pip install pyserial')
    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        monitor = Monitor(port)
        if args.corners:
            state = monitor.query(0x53)
            if (not state['query_page_valid'] or state['algorithm_stale'] or state['core_stale'] or not state['algorithm_seen']
                    or not state['core_seen'] or state['busy']):
                raise RuntimeError('Export corners only with a fresh, idle algorithm snapshot')
            points = []
            for index in range(state['corner_count']):
                point = monitor.query(0x43, index)
                if (not point['query_page_valid'] or point['algorithm_stale'] or point['core_stale']
                        or point['busy'] or not point.get('corner_valid', False)
                        or point['calibration_job'] != state['calibration_job']):
                    raise RuntimeError('Calibration changed or snapshot expired during export')
                points.append({'index': index, 'xy': point['corner_xy']})
            args.corners.parent.mkdir(parents=True, exist_ok=True)
            args.corners.write_text(json.dumps({'status': state, 'corners': points}, indent=2), encoding='utf-8')
            print(f'Saved {len(points)} corners to {args.corners}')
        elif args.video_seconds is not None:
            if args.video_seconds < 1:
                parser.error('--video-seconds must be at least 1')
            a = monitor.query(0x56)
            deadline = time.monotonic() + args.video_seconds
            while time.monotonic() < deadline:
                monitor.read()  # Drain heartbeats; do not overrun host serial buffer.
            b = monitor.query(0x56)
            if not all(s['query_page_valid'] and not s['core_stale'] and s['video_active'] and 'video' in s for s in (a, b)):
                raise RuntimeError('Need two fresh video snapshots while video remains active')
            seconds = ((b['uptime_ms'] - a['uptime_ms']) & 0xffffffff) / 1000.0
            if seconds <= 0 or a['ui_events'] != b['ui_events'] or a['calibration_job'] != b['calibration_job']:
                raise RuntimeError('UI/job changed during measurement; retry a steady interval')
            deltas = {k: (b['video'][k] - a['video'][k]) & 0xffffffff for k in
                ('captured_frames', 'corrected_frames', 'displayed_new_frames', 'failed_frames', 'skipped_camera_frames_since_reset')}
            print(json.dumps(dict(seconds=seconds, deltas=deltas,
                new_frame_fps={k: deltas[k]/seconds for k in ('captured_frames', 'corrected_frames', 'displayed_new_frames')},
                last=b), indent=2))
        elif args.once:
            print(json.dumps(monitor.query({'status': 0x53, 'progress': 0x44, 'video': 0x56, 'demo': 0x45}[args.once]), indent=2))
        else:
            if args.log:
                args.log.parent.mkdir(parents=True, exist_ok=True)
            last = time.monotonic()
            while True:
                for result in monitor.read():
                    last = time.monotonic()
                    line = json.dumps(result)
                    print(line, flush=True)
                    if args.log:
                        with args.log.open('a', encoding='utf-8') as out:
                            out.write(line + '\n')
                if time.monotonic() - last > 3:
                    print('No heartbeat for 3 s: check board power/configuration/sys_clk/serial link', flush=True)
                    last = time.monotonic()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
