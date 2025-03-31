#!/bin/bash

# Change the working directory to the build directory from the directory of the script
cd "$(dirname "$0")/.." || exit 1

echo "---Handling Grid Results---"
python3 analysis/grid_analysis.py
echo "---Handling STP Results---"
python3 analysis/stp_analysis.py
echo "---Handling Pancake Results---"
python3 analysis/pancake_analysis.py
