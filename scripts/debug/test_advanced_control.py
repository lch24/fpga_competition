"""Pure software checks; no serial module or port access."""
import binascii
import math
import struct
import tempfile
import unittest
from pathlib import Path

from advanced_control import (config_flags, decode_status, inverse_affine, load_state, packets, payload,
                              resolve, save_state)
from uart_monitor import Framer


class AdvancedProtocolTests(unittest.TestCase):
    def test_geometry(self):
        self.assertEqual(inverse_affine(), (1, 0, 0, 0, 1, 0))
        for angle in (-180, -90, -10, 0, 10, 90, 180):
            a, b, c, d, e, f = inverse_affine(angle, 1.2, 14, -9)
            self.assertAlmostEqual(a*(639.5+14)+b*(359.5-9)+c, 639.5)
            self.assertAlmostEqual(d*(639.5+14)+e*(359.5-9)+f, 359.5)
            self.assertAlmostEqual(a*e-b*d, 1/(1.2**2))

    def test_commands(self):
        data = payload(inverse_affine(10), True, 'auto')
        self.assertEqual(data.hex(), '1cfc0000742c0000f749cbff8cd3ffff1cfc0000778274000300')
        out = packets(data)
        self.assertEqual(len(out), 14)
        restored = bytearray()
        for i, p in enumerate(out):
            self.assertEqual(p[:2], b'\xa5\x5a')
            self.assertEqual(p[2], 0x60+i)
            self.assertEqual(p[5], p[2]^p[3]^p[4])
            if i < 13:
                restored.extend(p[3:5])
        self.assertEqual(restored, data)
        self.assertEqual(int.from_bytes(out[-1][3:5], 'little'), binascii.crc_hqx(data, 0xffff))
        self.assertEqual(out[-1].hex(), 'a55a6d33ffa1')

    def test_bounds(self):
        for v in (0, -1, math.inf, math.nan):
            with self.assertRaises(ValueError):
                inverse_affine(scale=v)
        with self.assertRaises(ValueError):
            payload((40000, 0, 0, 0, 1, 0))

    def test_status(self):
        frame = bytearray(b'\xa5\x5a\x01\x47\x01\x00\x78\x00'+bytes(122))
        frame[13] = 8
        struct.pack_into('<8I', frame, 32, 0, 0, 42, 600, 123, 456, 7, 0xa1020003)
        struct.pack_into('<H', frame, 126, 0xfffb)
        struct.pack_into('<H', frame, 128, binascii.crc_hqx(frame[2:128], 0xffff))
        framer = Framer()
        self.assertEqual(framer.feed(frame[:49]), [])
        frames = framer.feed(frame[49:])
        status = decode_status(frames[0])
        self.assertEqual(status['applied_count'], 42)
        self.assertEqual(status['config_errors'], 7)
        self.assertEqual(status['flags'], 3)
        self.assertEqual(status['auto_curve'], 2)
        frame[13] = 0
        self.assertIsNone(decode_status(frame))


class HostStateTests(unittest.TestCase):
    def test_independent_controls(self):
        state = dict(matrix=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0], affine=False, gamma='off')
        matrix, affine, gamma = resolve(None, 180, None, None, None, False, False, None, False, state)
        self.assertTrue(affine)
        self.assertEqual(gamma, 'off')
        self.assertNotEqual(matrix, state['matrix'])
        rotated = dict(matrix=matrix, affine=affine, gamma=gamma)
        kept, affine2, gamma2 = resolve(None, None, None, None, None, False, False, 'auto', False, rotated)
        self.assertEqual(kept, matrix)
        self.assertTrue(affine2)
        self.assertEqual(gamma2, 'auto')
        off, affine3, gamma3 = resolve(None, None, None, None, None, False, True, None, False,
                                       dict(matrix=kept, affine=affine2, gamma=gamma2))
        self.assertFalse(affine3)
        self.assertEqual(gamma3, 'auto')
        self.assertEqual(off, matrix)

    def test_explicit_gamma_off_keeps_affine(self):
        matrix, affine, gamma = resolve(None, None, None, None, None, False, False, 'off', False,
                                        dict(matrix=[2.0, 0.0, 0.0, 0.0, 2.0, 0.0], affine=True, gamma='darken'))
        self.assertTrue(affine)
        self.assertEqual(gamma, 'off')
        self.assertEqual(matrix, [2.0, 0.0, 0.0, 0.0, 2.0, 0.0])

    def test_reset(self):
        matrix, affine, gamma = resolve(None, None, None, None, None, False, False, None, True,
                                        dict(matrix=[2.0, 0.0, 0.0, 0.0, 2.0, 0.0], affine=True, gamma='auto'))
        self.assertEqual(matrix, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        self.assertFalse(affine)
        self.assertEqual(gamma, 'off')
        self.assertEqual(config_flags(affine, gamma), 0)

    def test_flags(self):
        self.assertEqual(config_flags(True, 'off'), 1)
        self.assertEqual(config_flags(False, 'identity'), 6)
        self.assertEqual(config_flags(True, 'auto'), 3)
        self.assertEqual(config_flags(True, 'darken-strong'), 1 | 2 | (5 << 2))

    def test_state_roundtrip(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'state.json'
            self.assertEqual(load_state(path), dict(matrix=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                                                   affine=False, gamma='off'))
            save_state(path, [0.0, -1.0, 1.0, 1.0, 0.0, 0.0], True, 'darken')
            saved = load_state(path)
            self.assertEqual(saved['gamma'], 'darken')
            self.assertTrue(saved['affine'])
            self.assertEqual(saved['matrix'], [0.0, -1.0, 1.0, 1.0, 0.0, 0.0])
            path.write_text('{"matrix": [1, 0], "affine": 1, "gamma": "loud"}', encoding='utf-8')
            with self.assertRaises(ValueError):
                load_state(path)


if __name__ == '__main__':
    unittest.main()
