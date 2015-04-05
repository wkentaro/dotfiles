#!/usr/bin/env python
# -*- coding: utf-8 -*-
#

import os

home = os.path.expanduser('~')
dotfiles_dir = os.path.join(home, '.dotfiles')
files = os.listdir(dotfiles_dir)

link_ignores = ['LICENSE.txt',
                'install_vim.sh',
                'iterm2-memo.txt',
                'link_dotfiles.py']

for f in files:
    if f.startswith('.'):
        # ignore .git or .gitmodules
        continue
    file1 = os.path.join(dotfiles_dir, f)
    file2 = os.path.join(home, '.'+f)
    print('ln -s {0} {1}'.format(file1, file2))
    os.system('ln -s {0} {1}'.format(file1, file2))
