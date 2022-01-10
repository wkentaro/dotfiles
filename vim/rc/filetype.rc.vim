" ----------------------------------------------------------------
" Filetype
" ----------------------------------------------------------------
au BufNewFile * set endofline

au BufNewFile,BufRead *.sh set ft=sh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.zsh set ft=zsh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.bash set ft=sh tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.go set tabstop=8 shiftwidth=4
autocmd FileType yaml setlocal indentkeys-=<:> tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.cfg set ft=python tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.h set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.c set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.cpp set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.launch set tabstop=8 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.test set tabstop=8 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.md set tabstop=8 shiftwidth=2 ft=markdown
au BufNewFile,BufRead *.html set tabstop=8 shiftwidth=2
autocmd FileType,BufRead *.html let b:syntastic_skip_checks = 1
au BufNewFile,BufRead *.js set tabstop=8 shiftwidth=2
au BufNewFile,BufRead *.json set tabstop=8 shiftwidth=4

"--------------------------------------
" Python setting
"--------------------------------------
autocmd FileType python nnoremap ,b :!black --line-length 79 %<cr> :e<cr>
au BufNewFile,BufRead *.py set tabstop=8 shiftwidth=4
au BufNewFile,BufRead *.py set
au BufNewFile,BufRead *.py set indentkeys-=:
au BufNewFile,BufRead *.pyx set ft=python tabstop=8 shiftwidth=4

" ----------------------------------------------------------
" C++ setting
" ----------------------------------------------------------
autocmd FileType cpp setlocal path=.,./include,../include/,/usr/include,/usr/local/include,/usr/include/c++/4.8/,/opt/ros/indigo/include,/usr/include/pcl-1.7
autocmd FileType cpp nnoremap ,cl :!clang-format -i %<cr> :e<cr>

" ----------------------------------------------------------
" RestructuredText setting
" ----------------------------------------------------------
au BufNewFile,BufRead *.rst set tabstop=8 shiftwidth=2 ft=rst
au BufRead,BufNewFile,BufReadPre *.rst nnoremap ,h1 VypVr=
au BufRead,BufNewFile,BufReadPre *.rst nnoremap ,h2 VypVr-
au BufRead,BufNewFile,BufReadPre *.rst nnoremap ,h3 VypVr+

"--------------------------------------
" Tex setting
"--------------------------------------
au BufNewFile,BufRead *.tex set tabstop=8 shiftwidth=2 ft=tex wrap linebreak
autocmd FileType,BufRead *.tex let b:syntastic_skip_checks = 1
au BufNewFile,BufRead *.bib set iskeyword+=:
" au BufNewFile,BufRead *.tex set isk+=-
" autocmd FileType,BufRead *.tex nnoremap <leader>m :w<cr> :!make<cr> <cr>
