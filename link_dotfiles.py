#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import os
import sys
import argparse
import fnmatch


def drop_ignore_files(filenames, ignore_config='.linkignore'):
    with open(ignore_config) as f:
        ignore_files = map(lambda x: x.strip(), f.readlines())
    being_ignored = []
    for ignore in ignore_files:
        being_ignored.extend(fnmatch.filter(filenames, ignore))
    filtered = filter(lambda x: x not in being_ignored, filenames)
    return filtered


def link_dotfiles(dry_run=False):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    home_dir = os.path.expanduser('~')
    files = os.listdir(this_dir)

    for f in drop_ignore_files(filenames=files):
        file1 = os.path.join(this_dir, f)
        file2 = os.path.join(home_dir, '.{}'.format(f))
        print('ln -s {} {}'.format(file1, file2))
        # if not os.path.exists(file2):
        #     os.system('ln -s {0} {1}'.format(file1, file2))


def main():
    parser = argparse.ArgumentParser(description='options to link dotfiles')
    parser.add_argument('-n', '--dry-run', action='store_true',
                        help='output the commands which will be executed')
    args = parser.parse_args(sys.argv[1:])

    link_dotfiles(dry_run=args.dry_run)


if __name__ == '__main__':
    main()

