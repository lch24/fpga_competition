"""External tool discovery shared by the numerical regression scripts."""
from pathlib import Path
import os
import shutil

def modelsim_bin():
    configured = os.environ.get('MODELSIM_BIN')
    executable = shutil.which('vsim.exe') or shutil.which('vsim')
    if not configured and not executable:
        raise RuntimeError('Set MODELSIM_BIN to the ModelSim executable directory, or add it to PATH')
    directory = Path(configured).resolve() if configured else Path(executable).parent
    if not any((directory / name).is_file() for name in ('vsim.exe', 'vsim')):
        raise RuntimeError(f'ModelSim executable missing in {directory}')
    return directory
