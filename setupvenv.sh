#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

echo "Creating/updating Vanguard Python virtual environment..."

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip setuptools wheel

python -m pip install \
  numpy \
  scipy \
  matplotlib \
  pyqtgraph \
  PyQt5 \
  numba \
  h5py \
  pyserial

echo
echo "Checking packages..."

python - <<'EOF'
import numpy
import scipy
import matplotlib
import pyqtgraph
import PyQt5
import numba
import h5py
import serial

print("Python package check OK")
EOF

echo
echo "Virtual environment ready."
