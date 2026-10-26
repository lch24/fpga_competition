"""Host-only checks: do not turn missing edge evidence or stale samples into success."""
import copy
import math
from inspect_calibration import spatial_metrics, image_size
from watch_capture import summarize


def main():
    assert image_size({'geometry': {'supported': True, 'processing_size': [960, 540]}}) == (960, 540)
    assert image_size({}) == (1280, 720)
    p = [(50, 50), (1, 50), (1, 1)]
    s = spatial_metrics(p, 100, 100, [3, 4, 0, 2, 6, 8])
    for region, expected in zip(('center', 'outer_band', 'outer_corners'), (5, 2, 10)):
        assert s['regions'][region]['count'] == 1
        assert s['regions'][region]['training_rms_px'] == expected
    s = spatial_metrics([(50, 50)], 100, 100, [0, 0])
    assert s['regions']['outer_corners']['training_rms_px'] is None
    assert s['empty_grid_cells'] == 11
    s = spatial_metrics([(math.nan, 1), (-1, 0), (99, 99)], 100, 100)
    assert s['regions']['outer_corners']['count'] == 1
    base = dict(uptime_ms=100, ui_events=3, ui_phase='camera_preview', calibration_job=1,
                capture=dict(completed_tokens=4, source_status=0, short_lines_saturated=2,
                             source_pixel_count=921600, ddr_completed_words=57600, corrupt_end_tokens=0),
                video={k: 0 for k in ('captured_frames', 'failed_frames', 'displayed_new_frames',
                                      'skipped_camera_frames_since_reset')})
    last = copy.deepcopy(base)
    last['uptime_ms'] = 1100
    last['ui_events'] = 8  # Error changes during preview must remain measurable.
    last['video']['captured_frames'] = 28
    last['video']['failed_frames'] = 2
    s = summarize([base, last])
    assert s['seconds'] == 1 and s['sampled_distinct_frames'] == 1
    assert s['sampled_frames_with_short_lines'] == 1 and s['counters']['failed_frames'] == 2
    for key, value in (('uptime_ms', 0), ('ui_phase', 'live_video'), ('calibration_job', 2)):
        altered = copy.deepcopy(last); altered[key] = value
        try:
            summarize([base, altered])
        except ValueError:
            pass
        else:
            raise AssertionError(f'Accepted invalid interval: {key}')
    print('PASS capture interval/reset detection, sparse edge coverage, regional RMS')


if __name__ == '__main__':
    main()
