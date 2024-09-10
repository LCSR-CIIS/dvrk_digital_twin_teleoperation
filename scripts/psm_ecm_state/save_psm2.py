import numpy as np
import subprocess
import yaml
import os
from save_ecm import dump_state

if __name__ == "__main__":
    dump_state("/PSM2/measured_js", "PSM2_state.yaml")
    dump_state("/PSM2/jaw/measured_js", "JAW2_state.yaml")
