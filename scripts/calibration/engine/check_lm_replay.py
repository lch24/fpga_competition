"""Run real UART corners through RTL's full lens model, after replay_calibration.py."""
from sim_runner import run

if __name__ == '__main__':
    run('lm_replay')
