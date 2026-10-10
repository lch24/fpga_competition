"""Compatibility entry: initialization now always tests the board shared service."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent/'engine'))
from check_init import main
if __name__=='__main__':main()
