#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import os
import subprocess
import yaml


def install_dotfiles(force, dry_run):
    this_dir = os.path.dirname(os.path.abspath(__file__))
    home_dir = os.path.expanduser('~')

    with open(os.path.join(this_dir, 'dotfiles.yaml')) as f:
        link_config = yaml.load(f)

    for from_, to in link_config.items():
        from_ = os.path.join(this_dir, from_)
        to = os.path.join(home_dir, to)
        if dry_run:
            print('{0} -> {1}'.format(from_, to))
        else:
            if not force and os.path.exists(to):
                continue
            if os.path.islink(to):
                if force and not os.path.isdir(to):
                    os.system('ln -fs {0} {1}'.format(from_, to))
                else:
                    print('skipping: {0}'.format(from_))
            else:
                os.system('ln -s {0} {1}'.format(from_, to))


def install_commands():
    bin_path = os.path.expanduser('~/.local/bin')
    if not os.path.exists(bin_path):
        os.makedirs(bin_path)

    this_dir = os.path.dirname(os.path.abspath(__file__))
    scripts_dir = os.path.join(this_dir, 'install_scripts')
    for script in os.listdir(scripts_dir):
        script = os.path.join(scripts_dir, script)
        if not os.access(script, os.X_OK):
            continue
        subprocess.call(script, shell=True)


def main():
    parser = argparse.ArgumentParser(description='options to link dotfiles')
    parser.add_argument('-f', '--force', action='store_true',
                        help='force to link dotfiles')
    parser.add_argument('-n', '--dry-run', action='store_true',
                        help='output the commands which will be executed')
    args = parser.parse_args()

    force = args.force
    dry_run = args.dry_run

    install_dotfiles(force=force, dry_run=dry_run)
    install_commands()


if __name__ == '__main__':
    main()
