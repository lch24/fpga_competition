"""Atomic host control for the ISOLATED advanced bitstream. Default: dry run.

No serial port is opened unless --port is explicitly supplied. Calibration is
still performed by the FPGA; this utility only supplies inverse affine motion
and enhancement settings. Positive angle rotates the displayed image clockwise.

Affine and Gamma are independent board features but the wire protocol always
carries both. This tool remembers the last applied pair in a local state file,
so --angle/--gamma change only that item, --affine-off keeps Gamma, and --reset
turns both off. The state file is rewritten only after the board confirms the
new applied_count.
"""
import argparse
import binascii
import json
import math
import struct
import sys
import time
from pathlib import Path

from uart_monitor import Framer, command

MODES = {'off': 0, 'auto': 0, 'identity': 1, 'brighten': 2,
         'brighten-strong': 3, 'darken': 4, 'darken-strong': 5, 'equalize': 6}
IDENTITY = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
DEFAULT_STATE = Path(__file__).resolve().parents[2]/'data/debug/advanced_control_state.json'


def load_state(path):
    try:
        saved = json.loads(path.read_text(encoding='utf-8'))
    except (FileNotFoundError, json.JSONDecodeError):
        return dict(matrix=list(IDENTITY), affine=False, gamma='off')
    matrix = saved.get('matrix')
    if (not isinstance(matrix, list) or len(matrix) != 6
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in matrix)
            or saved.get('gamma') not in MODES or not isinstance(saved.get('affine'), bool)):
        raise ValueError(f'invalid state file {path}; delete it or use --reset')
    return dict(matrix=[float(v) for v in matrix], affine=saved['affine'], gamma=saved['gamma'])


def save_state(path, matrix, affine, gamma):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(dict(matrix=[float(v) for v in matrix], affine=bool(affine), gamma=gamma),
                              indent=2), encoding='utf-8')
    tmp.replace(path)


def config_flags(affine, gamma):
    return int(affine) | (int(gamma != 'off') << 1) | (MODES[gamma] << 2)


def resolve(matrix_arg, angle, scale, tx, ty, force_affine, affine_off, gamma_arg, reset, state):
    """Merge one command with the remembered configuration; return (matrix, affine, gamma)."""
    matrix = list(state['matrix'])
    affine = state['affine']
    gamma = state['gamma']
    if reset:
        return list(IDENTITY), False, 'off'
    if affine_off:
        affine = False
    if matrix_arg is not None or any(v is not None for v in (angle, scale, tx, ty)) or force_affine:
        matrix = list(matrix_arg) if matrix_arg is not None else list(inverse_affine(
            angle or 0.0, 1.0 if scale is None else scale, tx or 0.0, ty or 0.0))
        affine = True
    if gamma_arg is not None:
        gamma = gamma_arg
    return matrix, affine, gamma


def inverse_affine(angle=0.0, scale=1.0, tx=0.0, ty=0.0, width=1280, height=720):
    """Inverse of centered forward rotation/zoom/translation; image y points down."""
    if not all(math.isfinite(v) for v in (angle, scale, tx, ty)) or scale <= 0:
        raise ValueError('finite angle/translation and positive scale required')
    c, s = math.cos(math.radians(angle))/scale, math.sin(math.radians(angle))/scale
    cx, cy = (width-1)/2, (height-1)/2
    return (c, s, cx-c*(cx+tx)-s*(cy+ty),
            -s, c, cy+s*(cx+tx)-c*(cy+ty))


def payload(matrix, affine=False, gamma='off'):
    if len(matrix) != 6 or not all(math.isfinite(v) for v in matrix):
        raise ValueError('six finite inverse affine coefficients required')
    q = [round(v*65536) for v in matrix]
    if any(v < -(1 << 31) or v >= 1 << 31 for v in q):
        raise ValueError('Q16.16 coefficient outside signed 32-bit range')
    flags = int(affine) | (int(gamma != 'off') << 1) | (MODES[gamma] << 2)
    return struct.pack('<6iH', *q, flags)


def packets(data):
    if len(data) != 26:
        raise ValueError('payload must contain 26 bytes')
    words = struct.unpack('<13H', data)
    return [command(0x60+i, word) for i, word in enumerate(words)] + [
        command(0x6d, binascii.crc_hqx(data, 0xffff))]


def decode_status(frame):
    if frame[3] != 0x47:
        return None
    p = frame[8:128]
    if not p[5] & 8 or struct.unpack_from('<H', p, 118)[0] != 0xfffb:
        return None
    if p[4] & 12 or struct.unpack_from('<I', p, 8)[0] >= 100:
        return None
    words = struct.unpack_from('<8I', p, 24)
    if words[7] >> 24 != 0xa1:
        raise RuntimeError('not an advanced-profile bitstream (missing A1 marker)')
    return dict(applied_count=words[2], total_cycles=words[3],
                first_pass_cycles=words[4], second_pass_cycles=words[5],
                config_errors=words[6], flags=words[7] & 0xffff,
                auto_curve=(words[7] >> 16) & 7,
                logical_remap_ms=words[3]/125000.0)


def query(port, framer, timeout=4.0):
    deadline = time.monotonic()+timeout
    next_send = 0
    while time.monotonic() < deadline:
        if time.monotonic() >= next_send:
            port.write(command(0x47))
            next_send = time.monotonic()+0.25
        for frame in framer.feed(port.read(1024)):
            result = decode_status(frame)
            if result is not None:
                return result
    raise TimeoutError('no fresh advanced status: check advanced bitstream and running clocks')


def apply(port, data, timeout=6.0, expect_flags=None):
    framer = Framer()
    before = query(port, framer)
    if expect_flags is not None and before['flags'] != expect_flags:
        print('warning: board flags 0x%04x differ from saved state 0x%04x; '
              'merged configuration is being applied' % (before['flags'], expect_flags), file=sys.stderr)
    for packet in packets(data):
        port.write(packet)
    port.flush()
    flags = struct.unpack_from('<H', data, 24)[0]
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        after = query(port, framer, timeout=min(2, max(0.1, deadline-time.monotonic())))
        if after['config_errors'] != before['config_errors']:
            raise RuntimeError('configuration rejected (busy, incomplete, reserved bits or CRC)')
        if after['applied_count'] != before['applied_count']:
            if after['flags'] != flags or after['applied_count'] != (before['applied_count']+1) & 0xffffffff:
                raise RuntimeError('unexpected concurrent controller or reset; query status before retrying')
            return after
    raise TimeoutError('config not applied: start live video/preview; do not blindly resend while mailbox is pending')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angle', type=float, default=None)
    parser.add_argument('--scale', type=float, default=None)
    parser.add_argument('--tx', type=float, default=None)
    parser.add_argument('--ty', type=float, default=None)
    parser.add_argument('--matrix', nargs=6, type=float, help='direct output-to-source a00 a01 a02 a10 a11 a12')
    parser.add_argument('--affine', action='store_true', help='enable affine, identity unless a transform is given')
    parser.add_argument('--affine-off', action='store_true', help='disable affine and keep gamma')
    parser.add_argument('--reset', action='store_true', help='disable affine and gamma')
    parser.add_argument('--gamma', choices=MODES, default=None)
    parser.add_argument('--state', type=Path, default=DEFAULT_STATE, help='host-side memory of the last applied config')
    parser.add_argument('--port', help='explicit opt-in to actual UART writes, e.g. COM11')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--dry-run', action='store_true', help='never open a port, even with --port')
    parser.add_argument('--status', action='store_true', help='query only; requires --port')
    args = parser.parse_args()
    transform = any(v is not None for v in (args.angle, args.scale, args.tx, args.ty))
    if args.matrix and transform:
        parser.error('--matrix cannot be combined with angle/scale/translation')
    if args.affine and args.affine_off:
        parser.error('--affine and --affine-off are mutually exclusive')
    if args.reset and (args.affine or args.affine_off or args.gamma is not None or args.matrix or transform):
        parser.error('--reset cannot be combined with other configuration options')
    if args.status and (args.affine or args.affine_off or args.reset or args.gamma is not None or args.matrix or transform):
        parser.error('--status does not accept configuration options')
    try:
        state = load_state(args.state)
        matrix, affine, gamma = resolve(args.matrix, args.angle, args.scale, args.tx, args.ty,
                                        args.affine, args.affine_off, args.gamma, args.reset, state)
        data = payload(matrix, affine, gamma)
        if not args.port or args.dry_run:
            if args.status:
                parser.error('--status needs --port without --dry-run')
            print(json.dumps(dict(dry_run=True, state_file=str(args.state), previous=state,
                                  inverse_matrix=matrix, affine=affine, gamma=gamma,
                                  flags=config_flags(affine, gamma), payload_hex=data.hex(),
                                  commands_hex=[p.hex() for p in packets(data)]), indent=2))
            return
        import serial
        with serial.Serial(args.port, args.baud, timeout=0.05, write_timeout=2) as port:
            result = query(port, Framer()) if args.status else apply(
                port, data, expect_flags=config_flags(state['affine'], state['gamma']))
        if not args.status:
            save_state(args.state, matrix, affine, gamma)
        print(json.dumps(result, indent=2))
    except (ValueError, RuntimeError, TimeoutError) as exc:
        parser.exit(1, str(exc)+'\n')


if __name__ == '__main__':
    main()
