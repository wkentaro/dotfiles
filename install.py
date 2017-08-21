#!/usr/bin/env python

import argparse
import glob
import os
import os.path as osp
import shutil
import subprocess
import yaml


def run_command(cmd, cwd=None):
    cwd = os.getcwd() if cwd is None else cwd
    print('+ cd %s && %s' % (cwd, cmd))
    subprocess.call(cmd, shell=True, cwd=cwd)


def link_file(from_, to, dry_run=False):
    if not osp.exists(osp.dirname(to)):
        os.makedirs(osp.dirname(to))
    if osp.exists(to):
        return
    if osp.islink(to):
        if not osp.isdir(to):
            if dry_run:
                print('{0} -> {1}'.format(from_, to))
                return
            run_command('ln -fs {0} {1}'.format(from_, to))
        else:
            print('skipping: {0}'.format(from_))
    else:
        if dry_run:
            print('{0} -> {1}'.format(from_, to))
            return
        run_command('ln -s {0} {1}'.format(from_, to))


def copy_file(from_, to, dry_run=False):
    if not osp.exists(osp.dirname(to)):
        os.makedirs(osp.dirname(to))
    if osp.exists(to):
        return
    if dry_run:
        print('{0} -> {1}'.format(from_, to))
        return
    if osp.isdir(from_):
        shutil.copytree(from_, to)
    else:
        shutil.copy(from_, to)


def install_private():
    path = osp.expanduser('~/.dotfiles/private')
    if osp.exists(path):
        run_command('git pull origin master', cwd=path)
    else:
        url = 'https://github.com/wkentaro/private.git'
        run_command('git clone {} {}'.format(url, path))


def install_dotfiles(dry_run):
    this_dir = osp.dirname(osp.abspath(__file__))
    home_dir = osp.expanduser('~')

    with open(osp.join(this_dir, 'dotfiles.yaml')) as f:
        link_config = yaml.load(f)

    for from_, to in link_config.items():
        if isinstance(to, basestring):
            type_ = 'symlink'
        elif isinstance(to, dict):
            type_ = to['type']
            to = to['name']
        from_ = osp.join(this_dir, from_)
        to = osp.join(home_dir, to)
        for from_file in glob.glob(from_):
            if from_file == from_:
                to_file = to
            else:
                to_file = osp.join(to, osp.basename(from_file))
            if type_ == 'symlink':
                link_file(from_file, to_file, dry_run)
            elif type_ == 'copy':
                copy_file(from_file, to_file, dry_run)


def install_commands():
    bin_path = osp.expanduser('~/.local/bin')
    if not osp.exists(bin_path):
        os.makedirs(bin_path)

    this_dir = osp.dirname(osp.abspath(__file__))
    scripts_dir = osp.join(this_dir, 'install_scripts')
    for script in os.listdir(scripts_dir):
        script = osp.join(scripts_dir, script)
        if not os.access(script, os.X_OK):
            continue
        run_command(script)


here = osp.dirname(osp.abspath(__file__))


def main():
    parser = argparse.ArgumentParser(description='options to link dotfiles')
    parser.add_argument('-n', '--dry-run', action='store_true',
                        help='output the commands which will be executed')
    parser.add_argument('-p', '--private', action='store_true',
                        help='install private')
    args = parser.parse_args()

    run_command('git submodule update --init --recursive', cwd=here)

    if args.private:
        install_private()

    install_dotfiles(dry_run=args.dry_run)
    install_commands()


if __name__ == '__main__':
    main()
