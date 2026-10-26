"""Board diagnostics: binary UART framing, readable JSON, optional corner export.

python scripts/debug/uart_monitor.py COM5
python scripts/debug/uart_monitor.py COM5 --once progress
python scripts/debug/uart_monitor.py COM5 --once capture
python scripts/debug/uart_monitor.py COM5 --once geometry
python scripts/debug/uart_monitor.py COM5 --once params --save data/debug/camera.json
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
# Fallback for old bitstreams without the geometry page. New boards report it.
IMAGE_WIDTH, IMAGE_HEIGHT = 1280, 720
CHECK_REASONS = {0: 'none', 1: 'nonfinite_or_numeric_failure',
                 2: 'invalid_configuration', 3: 'residual_evaluation_failed',
                 4: 'not_converged', 5: 'nonpositive_focal_length',
                 6: 'reprojection_rms_too_large', 7: 'mapping_fold_or_degenerate'}
CHECK_WARNINGS = ['unusual_focal_length', 'principal_point_outside_image',
                  'small_pose_variation', 'few_calibration_views']
REASON_TEXT = ['无', '非有限数值或运算异常', '配置无效', '重投影计算失败/读角点超时',
               '迭代未收敛', '焦距不是正数', '重投影 RMS 达到或超过 3 像素', '映射翻折或局部退化']
WARNING_TEXT = ['焦距超出经验范围', '主点位于画面外', '棋盘姿态变化较小', '标定图片少于五张']
STATUS_TEXT = {0: '正常', 1: '配置/接口格式错误', 2: '未找到有效棋盘', 3: '缓存或帧格式异常',
               4: '标定结果未通过检查', 5: '访存错误', 6: '超时',
               0x81: '采集像素不足', 0x82: '采集像素过多', 0x83: '采集像素打包未对齐',
               0x84: '采集 FIFO 丢失一个字', 0x85: '采集 FIFO 丢失多个字',
               0x86: '摄像头帧格式异常', 0x87: '采集 FIFO 帧结束数据校验失败', 0x8b: '1–3 行像素不足',
               0x8c: '4–15 行像素不足', 0x8d: '至少 16 行像素不足'}


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
        'status_text': STATUS_TEXT.get((word >> 16) & 255, '未知状态'),
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
        'converged': bool(flag & 8), 'calibration_phase': alg[7] & 15,
        'candidate_valid': bool(flag & 32),
        'reduced_model': bool(flag & 64),
        'low_coverage': bool(flag & 128),
        'check_reason_code': alg[7] >> 4,
        'check_reason': CHECK_REASONS.get(alg[7] >> 4, 'unknown'),
        'check_warnings': [name for i, name in enumerate(CHECK_WARNINGS) if alg[13] & (16 << i)],
        'result_events': u32(alg, 8), 'last_result_phase': alg[13] & 15,
        'last_detect_flags': alg[12],
        'last_status': alg[14], 'last_error': alg[15],
        'corner_count': int.from_bytes(alg[60:62], 'little'), 'page': tag,
    }
    if (frame[3] in (0x56, 0x45) or tag in (0xfffb, 0xfffa)) and result['page_matches']:
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
    if tag == 0xfffb and result['page_matches']:
        result.pop('video', None)
        meta = u32(core, 20)
        result['capture'] = dict(supported=u32(core, 8) == 0x43415031,
            source_pixel_count=u32(core, 12), ddr_completed_words=u32(core, 16),
            source_status=meta & 255, source_lane=(meta >> 8) & 15,
            short_lines_saturated=(meta >> 14) & 63,
            source_frame_error_bits=(meta >> 23) & 15,
            source_frame_errors=[name for i,name in enumerate(
                ('roi_short_line','odd_byte_count','long_line','wrong_row_count')) if meta & (1 << (23+i))],
            frame_open=bool(meta & (1 << 21)), token_integrity_ok=bool(meta & (1 << 22)),
            completed_tokens=u32(core, 24), corrupt_end_tokens=u32(core, 28))
    if tag == 0xfffa and result['page_matches']:
        result.pop('video', None)
        pair = lambda offset: [u32(core, offset) & 65535, u32(core, offset) >> 16]
        flags = u32(core, 28)
        result['geometry'] = dict(supported=u32(core, 8) == 0x43525031,
            sensor_size=pair(12), processing_size=pair(16), crop_origin=pair(20), display_size=pair(24),
            scaler_error=bool(flags & 1), reader_error=bool(flags & 2), scaler_phase=(flags >> 2) & 3,
            scaler_reset_released=bool(flags & 16), display_enabled=bool(flags & 32),
            scaled_frames_since_display_enable=flags >> 8)
        if result['geometry']['supported']:
            w, h = result['geometry']['processing_size']
            result['geometry'].update(expected_pixels=w*h, expected_capture_words=w*h//16,
                rgb565_stride_bytes=w*2, rgb565_frame_bytes=w*h*2, packed_map_bytes=w*h*8,
                parameter_coordinates='cropped image pixels; display scaling excluded')
    if tag == 0xffff:
        result['params'] = dict(zip(PARAMS, struct.unpack_from('<9f', alg, 16))) if flag & 2 else None
        result['candidate_params'] = dict(zip(PARAMS, struct.unpack_from('<9f', alg, 16))) if flag & 32 else None
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
    elif tag not in (0xfffd, 0xfffc, 0xfffb, 0xfffa):
        result['corner_valid'] = bool(flag & 16)
        result['corner_xy'] = struct.unpack_from('<2f', alg, 16) if flag & 16 else None
    return result


def concise(result):
    """Operational output; full transport/DDR counters remain in the JSON log."""
    keys = ('uptime_ms', 'calibration_job', 'ui_phase', 'algorithm_phase', 'view',
            'captured_mask', 'distort_mode', 'busy', 'status', 'last_result_phase',
            'last_status', 'corner_count', 'params_valid', 'candidate_valid',
            'converged', 'rms', 'check_reason', 'check_warnings', 'reduced_model', 'low_coverage')
    out = {key: result[key] for key in keys if key in result}
    out['link_fresh'] = bool(result.get('page_matches') and not result.get('core_stale')
                            and not result.get('algorithm_stale'))
    if result.get('params_valid'):
        out['params'] = result.get('params')
    elif result.get('candidate_valid'):
        out['rejected_candidate_params'] = result.get('candidate_params')
    if result.get('last_status') == 4 and result.get('check_reason_code') == 0:
        out['diagnostic'] = 'No detailed reason reported (older bitstream or failure before validation).'
    for key in ('progress', 'video', 'demo', 'capture', 'geometry'):
        if key in result:
            out[key] = result[key]
    return out


def readable(result, show_result=True):
    reason = result.get('check_reason_code', 0)
    warning = result.get('check_warnings', [])
    lines = [f"[{result.get('uptime_ms', 0)/1000:.1f}s] 任务 {result.get('calibration_job')} "
             f"界面={result.get('ui_phase')} 算法={result.get('algorithm_phase')} "
             f"图={result.get('view', 0)+1} 角点={result.get('corner_count')} "
             f"人工畸变={'开' if result.get('distort_mode') else '关'} "
             f"状态={result.get('status')}({result.get('status_text')}) 参数可用={result.get('params_valid')}"]
    if result.get('core_stale') or result.get('algorithm_stale') or not result.get('page_matches'):
        lines.append('状态快照过期或页面不匹配：以下结果不能视为当前状态。')
    if show_result and result.get('low_coverage'):
        if result.get('reduced_model'):
            lines.append('旧固件降阶结果：覆盖门限使 k3 固定为 0。')
        else:
            lines.append('覆盖提示：边缘角点较少；仍使用含 k3 的完整模型，未因此禁用参数或拒绝标定。')
    elif show_result and result.get('reduced_model'):
        lines.append('旧固件降阶结果：完整模型曾映射翻折，重试时固定了 k3=0。')
    if show_result and result.get('metrics_valid'):
        lines.append(f"标定：收敛={result.get('converged')} RMS={result.get('rms')} 像素")
    if show_result and (reason or result.get('last_status')):
        why = REASON_TEXT[reason] if reason < len(REASON_TEXT) else f'未知原因 {reason}'
        if not reason:
            if result.get('last_status') == 2 and result.get('last_result_phase') == 0:
                why = '棋盘检测未组成完整网格；progress 页可查看失败图片编号及检测结束点数'
            else:
                why = '未提供详细原因（旧比特流或校验之前失败），请结合 last_result_phase/last_status 查询'
        lines.append(f"诊断：{why}；阶段={result.get('last_result_phase')} 错误码={result.get('last_status')}")
    if show_result and warning:
        lines.append('质量提示（不阻止使用）：' + '、'.join(WARNING_TEXT[CHECK_WARNINGS.index(x)] for x in warning))
    params = result.get('params') or result.get('candidate_params')
    if show_result and params:
        lines.append(('有效参数：' if result.get('params_valid') else '被拒绝的候选参数（仅供诊断）：') +
                     ' '.join(f'{key}={value:.9g}' for key, value in params.items()))
    return '\n'.join(lines)


def command(op, index=0):
    lo, hi = index & 255, (index >> 8) & 255
    return bytes([0xa5, 0x5a, op, lo, hi, op ^ lo ^ hi])


def intrinsics(params):
    """Nine FP32 slots -> 3x3 intrinsic matrix and OpenCV-ordered distortion.

    The board publishes `fx,fy,cx,cy,k1,k2,k3,p1,p2`, already decoded to physical
    pixels (the solver stores log focal length and normalised principal point).
    OpenCV's five-coefficient model is the same Brown model in the order
    (k1, k2, p1, p2, k3), so k3 moves to the last position here.
    """
    matrix = [[params['fx'], 0.0, params['cx']],
              [0.0, params['fy'], params['cy']],
              [0.0, 0.0, 1.0]]
    distortion = [params['k1'], params['k2'], params['p1'], params['p2'], params['k3']]
    return matrix, distortion


def camera_report(result, geometry=None):
    """Readable intrinsics/distortion report plus the OpenCV-compatible record."""
    if result.get('params_valid'):
        params, source = result['params'], 'validated'
    elif result.get('candidate_valid'):
        params, source = result['candidate_params'], 'candidate_rejected_by_validation'
    else:
        raise RuntimeError('No complete nine-parameter set in this snapshot; calibrate first')
    if not all(params.get(key) for key in ('fx', 'fy')):
        raise RuntimeError('Focal length is zero in this snapshot; parameter page is stale')
    matrix, distortion = intrinsics(params)
    geometry = geometry or result.get('geometry', {})
    width, height = geometry['processing_size'] if geometry.get('supported') else (IMAGE_WIDTH, IMAGE_HEIGHT)
    record = {'source': source, 'image_width': width, 'image_height': height,
              'geometry': geometry, 'geometry_source': 'board' if geometry.get('supported') else 'legacy_1280x720_assumption',
              'calibration_job': result.get('calibration_job'), 'rms_px': result.get('rms'),
              'focal_length_px': {'fx': params['fx'], 'fy': params['fy']},
              'principal_point_px': {'cx': params['cx'], 'cy': params['cy']},
              'camera_matrix': matrix, 'distortion_coefficients': distortion,
              'distortion_order': 'k1,k2,p1,p2,k3',
              'reduced_model': result.get('reduced_model'), 'low_coverage': result.get('low_coverage'),
              'check_warnings': result.get('check_warnings', [])}
    lines = [f"相机标定参数（{'已通过全部检查' if source == 'validated' else '仅为候选，未通过检查'}）"
             f"任务={record['calibration_job']}",
             f"焦距 fx={params['fx']:.6f} px  fy={params['fy']:.6f} px",
             f"主点 cx={params['cx']:.6f} px  cy={params['cy']:.6f} px",
             '内参矩阵 K =',
             f"    [{matrix[0][0]:.6f}, 0, {matrix[0][2]:.6f}]",
             f"    [0, {matrix[1][1]:.6f}, {matrix[1][2]:.6f}]",
             "    [0, 0, 1]",
             '畸变系数（OpenCV 顺序 k1,k2,p1,p2,k3）= ['
             + ', '.join(f'{value:.9g}' for value in distortion) + ']',
             f"重投影 RMS = {record['rms_px']} px（{width}x{height}，处理图像坐标）"]
    if record['reduced_model']:
        lines.append('模型：降阶拟合，k3 固定为 0（覆盖不足或完整模型曾发生映射翻折）')
    if record['check_warnings']:
        lines.append('质量提示（不阻止使用）：'
                     + '、'.join(WARNING_TEXT[CHECK_WARNINGS.index(name)] for name in record['check_warnings']))
    return '\n'.join(lines), record


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
    capture_payload = bytearray(payload)
    struct.pack_into('<H', capture_payload, 118, 0xfffb)
    struct.pack_into('<6I', capture_payload, 32, 0x43415031, 921600, 57600,
                     (1 << 22) | (1 << 21) | (17 << 14) | (11 << 23), 35, 0)
    capture_header = header[:3] + b'C' + header[4:]
    capture_data = capture_header + capture_payload
    capture_result = decode(capture_data + struct.pack('<H', binascii.crc_hqx(capture_data[2:], 0xffff)))
    assert capture_result['capture']['supported'] and capture_result['capture']['token_integrity_ok']
    assert capture_result['capture']['source_pixel_count'] == 921600
    assert capture_result['capture']['ddr_completed_words'] == 57600
    assert capture_result['capture']['short_lines_saturated'] == 17
    assert capture_result['capture']['source_frame_error_bits'] == 11
    assert capture_result['capture']['source_frame_errors'] == ['roi_short_line','odd_byte_count','wrong_row_count']
    assert 'video' not in capture_result and 'ddr_read_requests' not in capture_result
    geometry_payload = bytearray(capture_payload)
    struct.pack_into('<H', geometry_payload, 118, 0xfffa)
    struct.pack_into('<6I', geometry_payload, 32, 0x43525031, (720 << 16) | 1280,
                     (540 << 16) | 960, (90 << 16) | 160, (720 << 16) | 1280,
                     (123 << 8) | 0x35)
    geometry_data = capture_header + geometry_payload
    geometry_result = decode(geometry_data + struct.pack('<H', binascii.crc_hqx(geometry_data[2:], 0xffff)))
    geometry = geometry_result['geometry']
    assert geometry['supported'] and geometry['processing_size'] == [960, 540]
    assert geometry['crop_origin'] == [160, 90] and geometry['display_size'] == [1280, 720]
    assert geometry['expected_pixels'] == 518400 and geometry['expected_capture_words'] == 32400
    assert geometry['rgb565_frame_bytes'] == 1036800 and geometry['packed_map_bytes'] == 4147200
    assert geometry['scaled_frames_since_display_enable'] == 123 and geometry['scaler_error']
    assert 'corner_xy' not in geometry_result and 'ddr_read_requests' not in geometry_result
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
    # Parameter page: nine FP32 at payload[72], RMS at payload[108..116].
    # Values are exact binary fractions so the FP32 round trip is unambiguous.
    payload = bytearray(120)
    payload[118:120] = b'\xff\xff'
    struct.pack_into('<9f', payload, 72, 1171.5, 1170.0, 645.75, 351.25,
                     0.015625, 0.5, 0.0, 0.0009765625, -0.001953125)
    struct.pack_into('<d', payload, 108, 0.10747)
    payload[62] |= 4  # metrics_valid
    payload[62] |= 2  # params_valid
    data = header + payload
    result = decode(data + struct.pack('<H', binascii.crc_hqx(data[2:], 0xffff)))
    matrix, distortion = intrinsics(result['params'])
    assert matrix == [[1171.5, 0.0, 645.75], [0.0, 1170.0, 351.25], [0.0, 0.0, 1.0]], matrix
    assert distortion == [0.015625, 0.5, 0.0009765625, -0.001953125, 0.0], distortion
    text, record = camera_report(result)
    assert '内参矩阵' in text and '畸变系数' in text and record['source'] == 'validated'
    assert record['rms_px'] == 0.10747 and record['distortion_order'] == 'k1,k2,p1,p2,k3'
    _, cropped_record = camera_report(result, geometry)
    assert (cropped_record['image_width'], cropped_record['image_height']) == (960, 540)
    assert cropped_record['camera_matrix'] == matrix  # no silent display rescale
    empty = dict(result, params=None, params_valid=False, candidate_valid=False)
    try:
        camera_report(empty)
    except RuntimeError:
        pass
    else:
        raise AssertionError('camera_report accepted a snapshot without parameters')
    print('PASS UART decoder: fragmentation, noise, CRC rejection, resynchronization, video counters/'
          'page validity, intrinsics/distortion parsing')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port', nargs='?')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--once', choices=['status', 'progress', 'video', 'demo', 'params', 'capture', 'geometry'])
    parser.add_argument('--save', type=Path, help='With --once params: write the parsed record as JSON')
    parser.add_argument('--video-seconds', type=float, help='Measure NEW-frame counter deltas over this interval')
    parser.add_argument('--corners', type=Path)
    parser.add_argument('--log', type=Path)
    parser.add_argument('--json', action='store_true', help='Print every full JSON packet; default shows state changes and a 10 s heartbeat')
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
            geometry_page = monitor.query(0x43, 0xfffa)
            if not geometry_page['query_page_valid'] or geometry_page['core_stale']:
                raise RuntimeError('Cannot establish image geometry from a fresh snapshot')
            geometry = geometry_page.get('geometry', {})
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
            final = monitor.query(0x53)
            if (not final['query_page_valid'] or final['uptime_ms'] < geometry_page['uptime_ms'] or
                    final['calibration_job'] != state['calibration_job'] or final['busy']):
                raise RuntimeError('Board reset/job changed during corner export')
            args.corners.write_text(json.dumps({'geometry': geometry, 'status': state, 'corners': points}, indent=2), encoding='utf-8')
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
        elif args.once == 'params':
            geometry_page = monitor.query(0x43, 0xfffa)
            state = monitor.query(0x53)
            if (not geometry_page['query_page_valid'] or geometry_page['core_stale'] or
                    state['uptime_ms'] < geometry_page['uptime_ms']):
                raise RuntimeError('Geometry stale or board reset between queries')
            text, record = camera_report(state, geometry_page.get('geometry'))
            print(text)
            if args.save:
                args.save.parent.mkdir(parents=True, exist_ok=True)
                args.save.write_text(json.dumps(record, indent=2), encoding='utf-8')
                print(f'Saved {args.save}')
        elif args.once == 'capture':
            print(json.dumps(monitor.query(0x43, 0xfffb), indent=2))
        elif args.once == 'geometry':
            print(json.dumps(monitor.query(0x43, 0xfffa), indent=2))
        elif args.once:
            print(json.dumps(monitor.query({'status': 0x53, 'progress': 0x44, 'video': 0x56, 'demo': 0x45}[args.once]), indent=2))
        else:
            if args.log:
                args.log.parent.mkdir(parents=True, exist_ok=True)
            last = time.monotonic()
            printed = 0
            previous = None
            while True:
                for result in monitor.read():
                    last = time.monotonic()
                    line = json.dumps(result)
                    short = concise(result)
                    signature = {k: v for k, v in short.items() if k != 'uptime_ms'}
                    if args.json or signature != previous or last - printed >= 10:
                        print(line if args.json else readable(result, signature != previous), flush=True)
                        printed, previous = last, signature
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
