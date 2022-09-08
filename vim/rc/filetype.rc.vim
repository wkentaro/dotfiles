au BufNewFile * set endofline


" ----------------------------------------------------------------
" python
" ----------------------------------------------------------------
au BufNewFile,BufRead *.py set filetype=python tabstop=8 shiftwidth=4 indentkeys-=:
autocmd FileType python inoremap ,k from IPython.core.debugger import Pdb; ipdb = Pdb(); ipdb.set_trace()<esc>
" autocmd FileType python nnoremap ,b :w<cr> :!black --line-length 79 %<cr> :e<cr>
" autocmd FileType python nnoremap ,b :w<cr> :!darker --line-length 110 --skip-string-normalization %<cr> :e<cr>
autocmd FileType python nnoremap ,f :w<cr> :!flake8 %<cr>

" Black(Python) format the visual selection
xnoremap ,b :!blacken 110<CR>
