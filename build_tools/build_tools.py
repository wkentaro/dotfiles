#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import sys
import argparse
import subprocess
import imp
import urllib
import tarfile


# args
parser = argparse.ArgumentParser()
parser.add_argument('formula', help='formula to install')
parser.add_argument('--prefix', default='~/local',
                    help='ex.) /usr/local, ~/local')
parser.add_argument('-v', '--verbose', action='store_true')
args = parser.parse_args(sys.argv[1:])

# sudo or not
sudo = False
if args.prefix == '/usr/local':
    sudo = True

# formula config
this_dir = os.path.join(os.path.dirname(__file__))
formula_path = os.path.join(this_dir, 'formula', args.formula + '.py')
formula_config = imp.load_source(args.formula, formula_path)

# download
def download(url, filename=None):
    if filename is None:
        filename = os.path.basename(url)
    urllib.urlretrieve(url, filename=filename)
    return filename

filename = download(formula_config.url)

# extract
with tarfile.open(filename) as f:
    f.extractall()
    extracted_dir = f.getnames()[0]

# build and install
os.chdir(extracted_dir)
commands = []
commands.append(['./configure', '--prefix', os.path.expanduser(args.prefix)])
commands.append(['make'])
make_install = ['make', 'install']
if sudo:
    make_install = ['sudo'] + make_install
commands.append(make_install)
for cmd in commands:
    stdout = subprocess.check_output(cmd)
    if args.verbose:
        print(stdout)
os.chdir('..')
