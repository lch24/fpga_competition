"""Run shared-engine LM RTL on nonzero-k3 truth and historical folding data.

Prepare build/debug/calibration_replay.json using replay_calibration.py on
data/calibration/uart_fold_corners.json (1280x720), then run this script.
"""
from sim_runner import run

if __name__=='__main__':run('lm_lens')
