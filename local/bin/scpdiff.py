#!/usr/bin/env python3

import argparse
import os
import os.path as osp
import shlex
import subprocess
import sys
import tempfile


def run_command(command, verbose=False):
    if verbose:
        print(f"+ {command}", file=sys.stderr)
    subprocess.call(shlex.split(command))


parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("file1", help="First file to compare")
parser.add_argument("file2", help="Second file to compare")
parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
parser.add_argument("--reverse", action="store_true", help="Reverse the order of the files")
args = parser.parse_args()

if args.reverse:
    file1 = args.file2
    file2 = args.file1
else:
    file1 = args.file1
    file2 = args.file2

tmp_dir = tempfile.mkdtemp()
tmp_file1 = osp.join(tmp_dir, file1.replace(':', '_'))
tmp_file2 = osp.join(tmp_dir, file2.replace(':', '_'))

os.makedirs(osp.dirname(tmp_file1), exist_ok=True)
os.makedirs(osp.dirname(tmp_file2), exist_ok=True)

run_command(f"scp -q {file1} {tmp_file1}", verbose=args.verbose)
run_command(f"scp -q {file2} {tmp_file2}", verbose=args.verbose)

run_command(f"git diff {tmp_file1} {tmp_file2}", verbose=args.verbose)
