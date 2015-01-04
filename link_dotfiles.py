#!/usr/bin/env python
# -*- coding: utf-8 -*-
# link_dotfiles.py
# author: Kentaro Wada <www.kentaro.wada@gmail.com>

import os

home = os.path.expanduser('~')
dotfiles_dir = os.path.join(home, '.dotfiles')
files = os.listdir(dotfiles_dir)

for file in files:
    if file.startswith('.'):
        continue  # .git and .gitignore is passed
    elif file.endswith('.txt'):
        continue
    file1 = os.path.join(dotfiles_dir, file)
    file2 = os.path.join(home, '.'+file)
    if os.access(file1, os.X_OK):
        continue  # executable script is passed
    print('ln -s {0} {1}'.format(file1, file2))
    os.system('ln -s {0} {1}'.format(file1, file2))