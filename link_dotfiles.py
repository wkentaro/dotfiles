#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import os
import sys
import argparse
import yaml



def link_dotfiles(force, dry_run):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    home_dir = os.path.expanduser('~')

    with open(os.path.join(this_dir, 'link_config.yml')) as f:
        link_config = yaml.load(f)

    for from_, to in link_config.items():
        from_ = os.path.join(this_dir, from_)
        to = os.path.join(home_dir, to)
        if dry_run:
            print('{0} -> {1}'.format(from_, to))
        else:
            if os.path.exists(to):
                if force and os.path.islink(to) and not os.path.isdir(to):
                    os.system('ln -fs {0} {1}'.format(from_, to))
                elif force and not os.path.islink(to):
                    print('the destination file is not symlink')
                else:
                    print('skipping: {0}'.format(from_))
            else:
                os.system('ln -s {0} {1}'.format(from_, to))


def main():
    parser = argparse.ArgumentParser(description='options to link dotfiles')
    parser.add_argument('-f', '--force', action='store_true',
                        help='force to link dotfiles')
    parser.add_argument('-n', '--dry-run', action='store_true',
                        help='output the commands which will be executed')
    args = parser.parse_args(sys.argv[1:])

    link_dotfiles(force=args.force, dry_run=args.dry_run)


if __name__ == '__main__':
    main()

