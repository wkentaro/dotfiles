" ----------------------------------------------------------------
" Filetype
" ----------------------------------------------------------------
au BufNewFile * set endofline

" Specific settings according to filetype
au BufNewFile,BufRead *.sh set ft=sh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.zsh set ft=zsh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.bash set ft=sh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.go set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.py set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.py set colorcolumn=80
au BufNewFile,BufRead *.py set indentkeys-=:
au BufNewFile,BufRead *.pyx set ft=python tabstop=8 shiftwidth=4
autocmd FileType yaml setlocal indentkeys-=<:> tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.cfg set ft=python tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.c set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.cpp set tabstop=8 shiftwidth=2 colorcolumn=80
au BufNewFile,BufRead *.launch set tabstop=8 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.test set tabstop=8 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.php set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.md set tabstop=8 shiftwidth=2 ft=markdown colorcolumn=80
au BufNewFile,BufRead *.html set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.js set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.rst set tabstop=8 shiftwidth=2 ft=rst
au BufNewFile,BufRead *.tex set tabstop=8 shiftwidth=2 ft=tex colorcolumn=100
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
au BufNewFile,BufRead *.l set colorcolumn=100
au BufNewFile,BufRead *.l set isk+=-
let lisp_rainbow = 1


" ----------------------------------------------------------
" RestructuredText setting
" ----------------------------------------------------------
au BufRead,BufNewFile,BufReadPre *.rst nnoremap ,h1 VypVr=
au BufRead,BufNewFile,BufReadPre *.rst nnoremap ,h2 VypVr-
au BufRead,BufNewFile,BufReadPre *.rst nnoremap ,h3 VypVr+
