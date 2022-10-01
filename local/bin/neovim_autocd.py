#!/usr/bin/env python3

import os

import neovim
import neovim.api


try:
    nvim = neovim.attach('socket', path=os.environ['NVIM'])
    nvim.vars['__autocd_cwd'] = os.getcwd()
    nvim.command('execute "lcd" fnameescape(g:__autocd_cwd)')
    del nvim.vars['__autocd_cwd']
except neovim.api.NvimError:
    pass
