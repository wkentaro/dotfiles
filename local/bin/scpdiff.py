#!/usr/bin/env python3

import argparse
import os
import os.path as osp
import shlex
import subprocess
import sys
import tempfile


def run_command(command, verbose=False, ask=False):
    if ask:
        answer = input(f"Execute?: {command} [y/N] ")
        if answer.lower() != "y":
            print("Aborted", file=sys.stderr)
            return

    if verbose:
        print(f"+ {command}", file=sys.stderr)
    subprocess.call(shlex.split(command))

    if ask:
        print(f"Executed: {command}", file=sys.stderr)


parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("file1", help="First file to compare")
parser.add_argument("file2", help="Second file to compare")
parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
parser.add_argument("--execute", "-e", action="store_true", help="Execute scp command")
args = parser.parse_args()

file1 = args.file1
file2 = args.file2

tmp_dir = tempfile.mkdtemp()
tmp_file1 = osp.join(tmp_dir, file1.replace(':', '_'))
tmp_file2 = osp.join(tmp_dir, file2.replace(':', '_'))

os.makedirs(osp.dirname(tmp_file1), exist_ok=True)
os.makedirs(osp.dirname(tmp_file2), exist_ok=True)

run_command(f"scp -q {file1} {tmp_file1}", verbose=args.verbose)
run_command(f"scp -q {file2} {tmp_file2}", verbose=args.verbose)

run_command(f"git --no-pager diff {tmp_file1} {tmp_file2}", verbose=args.verbose)

if args.execute:
    run_command(f"scp {file1} {file2}", verbose=args.verbose, ask=True)
