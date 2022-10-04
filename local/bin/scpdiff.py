#!/usr/bin/env python

import argparse
import os.path as osp
import subprocess
import tempfile


parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("file1", help="First file to compare")
parser.add_argument("file2", help="Second file to compare")
parser.add_argument("--reverse", action="store_true", help="Reverse the order of the files")
args = parser.parse_args()

if args.reverse:
    file1 = args.file2
    file2 = args.file1
else:
    file1 = args.file1
    file2 = args.file2

tmp_dir = tempfile.mkdtemp()
tmp_file1 = osp.join(tmp_dir, file1)
tmp_file2 = osp.join(tmp_dir, file2)

subprocess.call(["scp", file1, tmp_file1])
subprocess.call(["scp", file2, tmp_file2])

subprocess.call(["diff", tmp_file1, tmp_file2])
