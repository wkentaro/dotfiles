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
" nnoremap <localleader>x "_x
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
" nnoremap <C-j> <C-w>j
" nnoremap <C-k> <C-w>k
" nnoremap <C-h> <C-w>h
" nnoremap <C-l> <C-w>l

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
" map <localleader>ss :setlocal spell!<cr>


" ----------------------------------------------------------
" Terminal
" ----------------------------------------------------------
if has('nvim')
  nmap <localleader>x :terminal<cr>

  tnoremap <Esc> <C-\><C-n>
  tnoremap <M-[> <Esc>
  tnoremap <C-v><Esc> <Esc>

  augroup terminal_settings
    autocmd!

    autocmd TermOpen * startinsert
    autocmd TermOpen * setlocal nonumber norelativenumber
    autocmd BufWinEnter,WinEnter term://* startinsert
    autocmd BufLeave term://* stopinsert

    " Ignore various filetypes as those will close terminal automatically
    " Ignore fzf, ranger, coc
    autocmd TermClose term://*
      \ if (expand('<afile>') !~ "fzf") && (expand('<afile>') !~ "ranger") && (expand('<afile>') !~ "coc") |
      \   call nvim_input('<CR>')  |
      \ endif
  augroup END

  " Terminal mode:
  tnoremap <M-h> <C-\><C-n><C-w>h
  tnoremap <M-j> <C-\><C-n><C-w>j
  tnoremap <M-k> <C-\><C-n><C-w>k
  tnoremap <M-l> <C-\><C-n><C-w>l
  " Insert mode:
  inoremap <M-h> <Esc><C-w>h
  inoremap <M-j> <Esc><C-w>j
  inoremap <M-k> <Esc><C-w>k
  inoremap <M-l> <Esc><C-w>l
  " Visual mode:
  vnoremap <M-h> <Esc><C-w>h
  vnoremap <M-j> <Esc><C-w>j
  vnoremap <M-k> <Esc><C-w>k
  vnoremap <M-l> <Esc><C-w>l
  " Normal mode:
  nnoremap <M-h> <C-w>h
  nnoremap <M-j> <C-w>j
  nnoremap <M-k> <C-w>k
  nnoremap <M-l> <C-w>l
endif
