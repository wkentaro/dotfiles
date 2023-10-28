vim.cmd [[
function! s:reload_config()
  luafile ~/.config/nvim/init.lua
  PackerInstall
  PackerClean
  PackerCompile
endfunction

command! Reload call s:reload_config()
]]

-- navigation
vim.cmd [[set virtualedit=all]]
vim.cmd [[set ignorecase]]
vim.cmd [[set incsearch]]
vim.cmd [[set nowrapscan]]

-- yank
vim.cmd [[set clipboard=unnamedplus]]

-- appearence
vim.cmd [[set number]]
vim.cmd [[set list]]
vim.cmd [[set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲]]

-- edit
vim.cmd [[set noswapfile]]
vim.cmd [[set shiftwidth=2]]
vim.cmd [[
  set expandtab
  set autoindent
]]

-- window
vim.cmd [[set splitbelow]]
vim.cmd [[set splitright]]


-- mappings

vim.cmd [[
  let mapleader="\<space>"
  let maplocalleader=","
  set tm=500
]]

vim.cmd [[
  nnoremap <localleader>cd :lcd %:p:h<CR>:pwd<CR>
]]

vim.cmd [[
  nnoremap j gj
  nnoremap k gk
]]

vim.cmd [[
  " set grepprg=rg\ --vimgrep\ --no-heading\ --smart-case
  " set grepformat=%f:%l:%c:%m,%f:%l:%m
  " nnoremap <leader>e :silent grep!<Space>
  " nnoremap <expr> <leader>r ':silent grep! ' . expand('<cword>') . '<CR>'
  " nnoremap <expr> <leader>q ':cw \| wincmd L<CR>'
  nnoremap <silent> [c :cprevious<CR>
  nnoremap <silent> ]c :cnext<CR>
  nnoremap <silent> [l :lprevious<CR>
  nnoremap <silent> ]l :lnext<CR>
  " au QuickfixCmdPost make,grep,grepadd,vimgrep Telescope quickfix
  " au QuickfixCmdPost make,grep,grepadd,vimgrep copen | wincmd L
  " autocmd! FileType qf setl nonumber

  aug QFClose
    au!
    au WinEnter * if winnr('$') == 1 && &buftype == "quickfix"|q|endif
  aug END
]]

vim.cmd [[
  nnoremap <localleader>d "_d
  vnoremap <localleader>d "_d
  nnoremap <localleader>x "_dx
  vnoremap <localleader>p "_dP
]]

vim.cmd [[
  nnoremap <localleader>w :w<CR>
]]

vim.cmd [[
  nnoremap gn :bn<CR>
  nnoremap gp :bp<CR>
  nnoremap gk :bp<bar>bd! #<CR>
  nnoremap gr gT
]]

vim.cmd [[
  au BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$")
    \| exe "normal! g'\"" | endif
]]

vim.cmd [[
  tnoremap <Esc> <C-\><C-n>
  tnoremap <C-[> <C-\><C-n>

  function! IsOnThePrompt(prompt)
    let s:space_or_eol = '\( \|$\|\n\)'
    let l:line_number_end = line('$')
    let l:line_number_current = line('.')

    if abs(l:line_number_end - l:line_number_current) < 3
      return 1
    endif

    let l:found_prompt = 0
    let l:line_number_prompt = l:line_number_end
    while l:line_number_prompt > 0
      " found prompt
      if match(getline(l:line_number_prompt), a:prompt . s:space_or_eol) !=# -1
        let l:found_prompt = 1
        break
      endif
      " too long terminal
      if l:line_number_end - l:line_number_prompt > 10000
        break
      endif
      let l:line_number_prompt = l:line_number_prompt - 1
    endwhile

    if l:found_prompt == 0
      return 1  " in less
    endif

    return l:found_prompt && (l:line_number_current - l:line_number_prompt > -3)
  endfunction

  augroup terminal_settings
    autocmd TermOpen * startinsert
    autocmd TermOpen * setlocal nonumber norelativenumber
    autocmd BufWinEnter,WinEnter term://* if IsOnThePrompt('%') | startinsert | endif
    autocmd TermClose term://* call nvim_input('<CR>')
  augroup end
]]

vim.cmd [[
  autocmd BufWinEnter * if bufname() == "" | setlocal buftype=nofile | startinsert | endif
]]

vim.cmd [[
  imap <C-c> <Esc>
  nmap <C-c> <Esc>

  cnoremap <C-b> <Left>
  cnoremap <C-f> <Right>
  cnoremap <C-n> <Down>
  cnoremap <C-p> <Up>
  cnoremap <C-a> <Home>
  cnoremap <C-e> <End>
  cnoremap <C-d> <Del>
  cnoremap <C-x> <C-r>=expand('%:p')<CR>

  "tnoremap <C-Enter> <Enter>

  "tnoremap <M-n> <C-\><C-n>gt
  "inoremap <M-n> <Esc>gt
  "vnoremap <M-n> <Esc>gt
  "nnoremap <M-n> gt

  "tnoremap <M-p> <C-\><C-n>gT
  "inoremap <M-p> <Esc>gT
  "vnoremap <M-p> <Esc>gT
  "nnoremap <M-p> gT

  "tnoremap <M-h> <C-\><C-n><C-w>h
  "tnoremap <M-j> <C-\><C-n><C-w>j
  "tnoremap <M-k> <C-\><C-n><C-w>k
  "tnoremap <M-l> <C-\><C-n><C-w>l

  "inoremap <C-h> <Esc><C-w>h
  "inoremap <C-j> <Esc><C-w>j
  "inoremap <C-k> <Esc><C-w>k
  "inoremap <C-l> <Esc><C-w>l

  vnoremap <C-h> <Esc><C-w>h
  vnoremap <C-j> <Esc><C-w>j
  vnoremap <C-k> <Esc><C-w>k
  vnoremap <C-l> <Esc><C-w>l

  nnoremap <C-h> <C-w>h
  nnoremap <C-j> <C-w>j
  nnoremap <C-k> <C-w>k
  nnoremap <C-l> <C-w>l
]]

vim.cmd [[
  " ----------------------------------------------------------
  " Filetype
  " ----------------------------------------------------------
  autocmd FileType python setl tabstop=4 shiftwidth=4 indentkeys-=:
  autocmd FileType cpp setl tabstop=4 shiftwidth=4 indentkeys-=:
  autocmd FileType qf set nonumber

  if $USER == 'mujin'
    set colorcolumn=111
    autocmd FileType python noremap <localleader>b :!black --line-length 110 %<CR>
    autocmd FileType python noremap <localleader>i :!isort --force-single-line %<CR>
    autocmd FileType python xnoremap <localleader>b :!blacken 110<CR>
  else
    set colorcolumn=89
    autocmd FileType python noremap <localleader>b :!black %<CR>
  endif
]]
