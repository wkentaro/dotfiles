#!/bin/zsh
#

function _get_dist_path()
{
python -c """
import sys
import argparse
import imp
import pkgutil

def get_distribution_paths():
    pkg_paths = {}
    for pkg in pkgutil.iter_modules():
        pkg_name = pkg[1]
        file_, pkg_path = imp.find_module(pkg_name)[:2]
        if file_ is not None:
            continue
        pkg_paths[pkg_name] = pkg_path
    return pkg_paths

def main():
    if len(sys.argv) < 2:
        print('usage: pycd DIST_NAME')
        return
    dist_paths = get_distribution_paths()
    try:
        dist_path = dist_paths[sys.argv[1]]
    except KeyError:
        print('package not found: %s' % sys.argv[1])
        return
    print(dist_path)

main()
""" $1
}

function pycd ()
{
  DIST_PATH=`_get_dist_path $1`
  cd ${DIST_PATH}
}