" ----------------------------------------------------------------
" Filetype
" ----------------------------------------------------------------
" Specific settings according to filetype
au BufNewFile,BufRead *.sh set ft=sh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.zsh set ft=zsh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.bash set ft=sh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.py set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.py set colorcolumn=80
au BufNewFile,BufRead *.pyx set ft=python tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.cfg set ft=python tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.c set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.cpp set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.launch set tabstop=8 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.php set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.md set tabstop=8 shiftwidth=4 ft=markdown
au BufNewFile,BufRead *.rst set tabstop=8 shiftwidth=2 ft=rst
au BufNewFile,BufRead *.tex set tabstop=8 shiftwidth=2 ft=tex
au BufRead,BufNewFile,BufReadPre *.coffee   set filetype=coffee
autocmd FileType coffee setlocal sw=2 sts=2 ts=2 et


" ----------------------------------------------------------
" Cpp setting
" ----------------------------------------------------------
autocmd FileType cpp setlocal path=.,./include,../include/,/usr/include,/usr/local/include,/usr/include/c++/4.8/,/opt/ros/indigo/include,/usr/include/pcl-1.7


" ----------------------------------------------------------
" Lisp setting
" ----------------------------------------------------------
au BufNewFile,BufRead *.l set wrap tabstop=8 shiftwidth=2 ft=lisp
let lisp_rainbow = 1
