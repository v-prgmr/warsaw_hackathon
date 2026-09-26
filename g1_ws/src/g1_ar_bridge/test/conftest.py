"""Run the tests straight from the source tree (``pytest g1_ws/src/g1_ar_bridge``), no colcon."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))   # the package root (contains g1_ar_bridge/)
sys.path.insert(0, HERE)                    # synthetic.py
