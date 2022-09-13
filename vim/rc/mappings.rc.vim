" Mapleader "{{{
let mapleader="\<space>"
let maplocalleader=','
"}}}


" FIXME: C-h does not work...
imap <C-h> <BS>


" ----------------------------------------------------------
" Delete keymap
" ----------------------------------------------------------
" delete without yanking it
nnoremap <localleader>d "_d
vnoremap <localleader>d "_d
nnoremap <localleader>x "_x
"}}}


" ----------------------------------------------------------
" Paste keymap
" ----------------------------------------------------------
" without yanking it
vnoremap <localleader>p "_dP
"}}}


" ----------------------------------------------------------
" Move keymap
" ----------------------------------------------------------
" Move key "{{{
nnoremap j gj
nnoremap k gk
nnoremap gj j
nnoremap gk k
"}}}


" ----------------------------------------------------------
" Buffer keymap
" ----------------------------------------------------------
" Fast buffer alternation, next/prev, close "{{{
map ga <C-^>
nmap gn :bn<CR>
nmap gp :bp<CR>
nmap gk :bp<bar>bd #<CR>
"}}}


" ----------------------------------------------------------
" Tab keymap
" ----------------------------------------------------------
nmap gr gT

" ----------------------------------------------------------
" Window keymap
" ----------------------------------------------------------
" Quicker window movement
nnoremap <C-j> <C-w>j
nnoremap <C-k> <C-w>k
nnoremap <C-h> <C-w>h
nnoremap <C-l> <C-w>l

" Change window size
nnoremap <S-Left>  <C-w>><CR>
nnoremap <S-Right> <C-w><<CR>
nnoremap <S-Up>    <C-w>+<CR>
nnoremap <S-Down>  <C-w>-<CR>
"}}}


" ----------------------------------------------------------
" File quiting
" ----------------------------------------------------------
nmap <localleader>w :w<CR>
nmap <localleader>q :q<cr>


" ----------------------------------------------------------
" Spell checking
" ----------------------------------------------------------
map <localleader>ss :setlocal spell!<cr>
