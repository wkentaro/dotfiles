#!/usr/bin/env python3
# neovim-autocd.py
import neovim
import neovim.api
import os

nvim = neovim.attach('socket', path=os.environ['NVIM'])
try:
    nvim.vars['__autocd_cwd'] = os.getcwd()
    nvim.command('execute "lcd" fnameescape(g:__autocd_cwd)')
    del nvim.vars['__autocd_cwd']
except neovim.api.NvimError:
    pass
