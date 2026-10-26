"""Sample capture metadata and video counters without changing board state.

python scripts/debug/watch_capture.py COM7 --seconds 20
Samples are NOT every frame; counter deltas describe all frames in the interval.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

from uart_monitor import Monitor


def summarize(records):
    if not records:
        raise ValueError('No samples')
    first, last = records[0], records[-1]
    # Refuse to join counters across a reset, mode change, or calibration job.
    for a, b in zip(records, records[1:]):
        if (b['uptime_ms'] < a['uptime_ms'] or
                b['ui_phase'] != a['ui_phase'] or
                b['calibration_job'] != a['calibration_job']):
            raise ValueError('Board reset or UI/job changed; repeat a steady interval')
    unique = {r['capture']['completed_tokens']: r['capture'] for r in records}
    delta = lambda key: (last['video'][key]-first['video'][key]) & 0xffffffff
    return dict(seconds=(last['uptime_ms']-first['uptime_ms'])/1000,
                sampled_distinct_frames=len(unique),
                sampled_source_status_counts=dict(Counter(c['source_status'] for c in unique.values())),
                sampled_frames_with_short_lines=sum(c['short_lines_saturated'] > 0 for c in unique.values()),
                sampled_max_short_lines=max(c['short_lines_saturated'] for c in unique.values()),
                sampled_pixel_count_range=[min(c['source_pixel_count'] for c in unique.values()),
                                           max(c['source_pixel_count'] for c in unique.values())],
                sampled_written_words_range=[min(c['ddr_completed_words'] for c in unique.values()),
                                             max(c['ddr_completed_words'] for c in unique.values())],
                corrupt_end_tokens_delta=(last['capture']['corrupt_end_tokens']-
                                          first['capture']['corrupt_end_tokens']) & 0xffffffff,
                counters={key: delta(key) for key in ('captured_frames', 'failed_frames',
                          'displayed_new_frames', 'skipped_camera_frames_since_reset')},
                notes=['Short-line statistics are sampled, not an all-frame error rate.',
                       'END integrity does not validate pixel payload contents.',
                       'failed_frames includes both capture and remap failures.'])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--seconds', type=float, default=20)
    p.add_argument('--out', type=Path, default=Path(__file__).resolve().parents[2]/'build/debug/capture_watch.json')
    args = p.parse_args()
    if not 1 <= args.seconds <= 60:
        p.error('--seconds must be in 1..60')
    import serial
    records = []
    with serial.Serial(args.port, 115200, timeout=.1) as port:
        monitor = Monitor(port)
        deadline = time.monotonic()+args.seconds
        while True:
            capture = monitor.query(0x43, 0xfffb)
            video = monitor.query(0x56)
            for r in (capture, video):
                if not r['query_page_valid'] or r['core_stale'] or not r['core_seen']:
                    raise RuntimeError('Stale or mismatched snapshot')
            if not capture.get('capture', {}).get('supported'):
                raise RuntimeError('Bitstream does not support capture diagnostics')
            # ui_events also counts error/status transitions. Those are the
            # events we are measuring, not evidence that the mode changed.
            if (capture['ui_phase'] != video['ui_phase'] or
                    capture['calibration_job'] != video['calibration_job'] or
                    video['uptime_ms'] < capture['uptime_ms']):
                raise RuntimeError('Board changed between diagnostic pages')
            if video['ui_phase'] not in ('camera_preview', 'live_video'):
                raise RuntimeError('Keep preview or live video active during measurement')
            records.append(dict(uptime_ms=video['uptime_ms'], ui_events=video['ui_events'], ui_phase=video['ui_phase'],
                                calibration_job=video['calibration_job'],
                                capture=capture['capture'], video=video['video']))
            if time.monotonic() >= deadline:
                break
            time.sleep(.2)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    # Preserve the evidence even if the interval is invalidated by a reset.
    report = dict(samples=records)
    try:
        report['summary'] = summarize(records)
    except ValueError as error:
        report['error'] = str(error)
    args.out.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report.get('summary', report.get('error')), indent=2))
    print(f'Saved {args.out}')
    if 'error' in report:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
