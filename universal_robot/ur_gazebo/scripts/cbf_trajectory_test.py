#!/usr/bin/env python3
"""Launcher wrapper for cbf_trajectory_test.

Roslaunch finds executables listed in a package's 'scripts' area. The actual
implementation lives in custom_script/cbf_trajectory_test.py. This wrapper
executes that file with the same Python interpreter and forwards argv.
"""
import os
import sys

THIS_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT = os.path.normpath(os.path.join(THIS_DIR, '..'))
TARGET = os.path.join(ROOT, 'custom_script', 'cbf_trajectory_test.py')

if not os.path.isfile(TARGET):
    sys.stderr.write(f"Error: target script not found: {TARGET}\n")
    sys.exit(1)

os.execv(sys.executable, [sys.executable, TARGET] + sys.argv[1:])
