au BufNewFile * set endofline


" ----------------------------------------------------------------
" python
" ----------------------------------------------------------------
au BufNewFile,BufRead *.py set filetype=python tabstop=8 shiftwidth=4 indentkeys-=:
autocmd FileType python nnoremap ,b :w<cr> :!black --line-length 79 %<cr> :e<cr>
