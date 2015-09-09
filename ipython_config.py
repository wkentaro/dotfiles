#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import sys

c = get_config()

c.TerminalIPythonApp.display_banner = True
c.InteractiveShellApp.log_level = 0
c.InteractiveShellApp.exec_lines = [
    #'import numpy as np',
]
c.InteractiveShell.autoindent = True
c.InteractiveShell.colors = 'LightBG'
c.InteractiveShell.confirm_exit = False
c.InteractiveShell.deep_reload = True
c.InteractiveShell.editor = 'vim'
c.InteractiveShell.xmode = 'Context'

c.PromptManager.in_template = r'{color.Blue}\Y1 [\#]:\n'
c.PromptManager.in2_template = r''
c.PromptManager.out_template = r'> '
c.PromptManager.justify = True

c.PrefilterManager.multi_line_specials = True

if sys.platform == 'linux2':
    c.AliasManager.user_aliases = [
            ('ls', 'ls --color=auto'),
            ('la', 'ls --color=auto -al'),
            ('lsa', 'ls --color=auto -lah'),
            ('..', 'cd ..'),
            ]
else:
    c.AliasManager.user_aliases = [
            ('ls', 'ls -G'),
            ('la', 'ls -G -al'),
            ('lsa', 'ls -G -lah'),
            ('..', 'cd ..'),
            ]
