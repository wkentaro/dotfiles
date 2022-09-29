#!/usr/bin/env python3
# neovim-autocd.py
import neovim
import os

nvim = neovim.attach('socket', path=os.environ['NVIM'])
nvim.vars['__autocd_cwd'] = os.getcwd()
nvim.command('execute "lcd" fnameescape(g:__autocd_cwd)')
del nvim.vars['__autocd_cwd']
