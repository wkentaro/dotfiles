" ----------------------------------------------------------------
" Filetype
" ----------------------------------------------------------------
" Specific settings according to filetype
au BufNewFile,BufRead *.sh set ft=sh tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.zsh set ft=zsh tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.bash set ft=sh tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.py set tabstop=4 shiftwidth=4 colorcolumn=80
au BufNewFile,BufRead *.pyx set ft=python tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cfg set ft=python tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.c set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cpp set tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.launch set tabstop=2 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.php set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.md set tabstop=4 shiftwidth=4 ft=markdown
au BufNewFile,BufRead *.rst set tabstop=4 shiftwidth=4 ft=rst
au BufNewFile,BufRead *.tex set tabstop=2 shiftwidth=2 ft=tex