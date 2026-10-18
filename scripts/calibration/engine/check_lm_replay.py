"""Run real UART corners through RTL's fixed-k3 retry, after replay_calibration.py."""
from sim_runner import run

if __name__ == '__main__':
    run('lm_replay')
