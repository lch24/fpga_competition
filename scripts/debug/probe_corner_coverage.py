"""Read existing immutable corner snapshots; no board state-changing commands.

Works during idle, video, or a review wait, provided the algorithm is parked.
Abort if the job changes. Does not save files or replace calibration parameters.
"""
import argparse
import json
from uart_monitor import Monitor
from inspect_calibration import audit


def main():
    import serial
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('port')
    args = parser.parse_args()
    with serial.Serial(args.port, 115200, timeout=0.05) as port:
        monitor = Monitor(port)
        state = monitor.query(0x53)
        def valid(s):
            return (s['query_page_valid'] and not s['algorithm_stale'] and not s['core_stale']
                    and s['algorithm_phase'] in ('idle', 'corner_review')
                    and s['calibration_job'] == state['calibration_job'])
        if not valid(state) or state['corner_count'] != 120:
            raise RuntimeError('Need a parked algorithm and all 120 recorded corners')
        entries = []
        for i in range(120):
            point = monitor.query(0x43, i)
            if not valid(point) or not point.get('corner_valid'):
                raise RuntimeError('Snapshot changed or expired during read')
            entries.append(dict(index=i, xy=point['corner_xy']))
            if i%40 == 39:
                print(f'Read view {i//40+1}: 40 points', flush=True)
        report = audit(dict(corners=entries))
        waiting = state['algorithm_phase'] == 'corner_review'
        print(json.dumps(dict(job=state['calibration_job'], before_parameter_solve=waiting,
                              board_low_coverage=None if waiting else state['low_coverage'],
                              board_reduced_model=None if waiting else state['reduced_model'], audit=report), indent=2))


if __name__ == '__main__':
    main()
