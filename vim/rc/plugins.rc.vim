call plug#begin('~/.vim/plugged')
  Plug 'flazz/vim-colorschemes'

  Plug 'davidhalter/jedi-vim'

  Plug 'ycm-core/YouCompleteMe'

  if has('nvim')
    " Plug 'Shougo/deoplete.nvim', { 'do': ':UpdateRemotePlugins' }
    " Plug 'zchee/deoplete-jedi'
  else
    Plug 'Shougo/neocomplete.vim'
  endif

  Plug 'aperezdc/vim-template'

  Plug 'tomtom/tcomment_vim'

  Plug 'Shougo/unite.vim'
  Plug 'Shougo/neomru.vim'
  Plug 'Shougo/vimfiler.vim'
  Plug 'Shougo/vimproc.vim', {'do' : 'make'}

  " Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
  " Plug 'junegunn/fzf.vim'

  Plug 'nvim-lua/plenary.nvim'
  Plug 'nvim-telescope/telescope.nvim', { 'tag': '0.1.0' }

  Plug 'tami5/sql.nvim'  " sudo apt-get install sqlite3 libsqlite3-dev
  Plug 'nvim-telescope/telescope-frecency.nvim'

  Plug 'Shougo/neosnippet.vim'

  Plug 'rhysd/committia.vim'

  Plug 'hotwatermorning/auto-git-diff'

  Plug 'lervag/vimtex'

  Plug 'preservim/tagbar'

  Plug 'RRethy/vim-illuminate'

  " Plug 'scrooloose/syntastic'

  Plug 'tyru/open-browser.vim'

  Plug 'github/copilot.vim', { 'branch': 'release' }

  Plug 'tpope/vim-fugitive'

  Plug 'nvim-treesitter/nvim-treesitter'

  Plug 'folke/which-key.nvim'

  Plug 'kyazdani42/nvim-web-devicons' " Recommended (for coloured icons)
  " Plug 'ryanoasis/vim-devicons' Icons without colours
  Plug 'akinsho/bufferline.nvim', { 'tag': 'v2.*' }
call plug#end()


" ----------------------------------------------------------------
" flazz/vim-colorschemes
" ----------------------------------------------------------------
let g:solarized_termtrans = 1
if $VIM_COLORSCHEME == "solarized"
  colorscheme solarized
endif
hi Normal ctermfg=none
hi TabLineSel ctermfg=LightBlue ctermbg=Black
hi TelescopeNormal ctermbg=Black
hi WhichKeyFloat ctermbg=Black



" ----------------------------------------------------------------
" aperezdc/vim-template
" ----------------------------------------------------------------
let g:templates_directory = ['~/.vim/after/templates']
let g:templates_no_builtin_templates = 1


" ----------------------------------------------------------------
" Shougo/vimfiler.vim
" ----------------------------------------------------------------
let g:vimfiler_as_default_explorer = 1
let g:vimfiler_safe_mode_by_default = 0
noremap <silent> <localleader>f :VimFilerBuffer -buffer-name=explorer -split -simple -winwidth=35 -toggle -no-quit<cr>

autocmd FileType vimfiler call s:vimfiler_settings()
function! s:vimfiler_settings() abort
  silent! nunmap <buffer> <C-l>
endfunction

autocmd BufEnter * if winnr('$') == 1 && exists('b:vimfiler') && b:vimfiler['context']['explorer'] | quit | endif


" ----------------------------------------------------------------
" Shougo/neosnippet.vim
" ----------------------------------------------------------------
imap <C-k>     <Plug>(neosnippet_expand_or_jump)
smap <C-k>     <Plug>(neosnippet_expand_or_jump)
xmap <C-k>     <Plug>(neosnippet_expand_target)

imap <expr><TAB> neosnippet#expandable_or_jumpable() ?
\ "\<Plug>(neosnippet_expand_or_jump)"
\: pumvisible() ? "\<C-n>" : "\<TAB>"
smap <expr><TAB> neosnippet#expandable_or_jumpable() ?
\ "\<Plug>(neosnippet_expand_or_jump)"
\: "\<TAB>"
let g:neosnippet#disable_runtime_snippets = {
\   '_' : 1,
\ }
let g:neosnippet#enable_snipmate_compatibility = 1
let g:neosnippet#snippets_directory='~/.vim/after/snippets'


" ----------------------------------------------------------------
" davidhalter/jedi.vim
" ----------------------------------------------------------------
" autocmd FileType python setl omnifunc=jedi#completions
" autocmd FileType python setl completeopt-=preview
" let g:jedi#popup_select_first = 0
" let g:jedi#completions_enabled = 1  " use deoplete-jedi for async completion
" let g:jedi#auto_vim_configuration = 1
" let g:jedi#show_call_signatures = 0
" let g:jedi#rename_command = '<Leader>R'


" ----------------------------------------------------------------
" Shougo/deoplete.nvim
" ----------------------------------------------------------------
if has('nvim')
  let g:deoplete#enable_at_startup = 1
  autocmd InsertLeave,CompleteDone * if pumvisible() == 0 | pclose | endif
endif


" ----------------------------------------------------------------
" Shougo/neocomplete.vim
" ----------------------------------------------------------------
if !has('nvim') && has('lua')
  let g:acp_enableAtStartup = 0
  let g:neocomplete#enable_at_startup = 1

  let g:neocomplete#enable_smart_case = 1

  let g:neocomplete#sources#syntax#min_keyword_length = 3
  let g:neocomplete#lock_buffer_name_pattern = '\*ku\*'
  let g:neocomplete#enable_auto_select = 0
  let g:neocomplete#enable_auto_close_preview = 1
  let g:neocomplete#enable_ignore_case = 1

  if !exists('g:neocomplete#sources#omni#input_patterns')
    let g:neocomplete#sources#omni#input_patterns = {}
  endif
  let g:neocomplete#sources#omni#input_patterns.perl = '\h\w*->\h\w*\|\h\w*::'
  if !exists('g:neocomplete#force_omni_input_patterns')
    let g:neocomplete#force_omni_input_patterns = {}
  endif
  let g:neocomplete#force_omni_input_patterns.cpp =
    \ '[^.[:digit:] *\t]\%(\.\|->\)\w*\|\h\w*::\w*'
  let g:neocomplete#force_omni_input_patterns.python =
    \ '\%([^. \t]\.\|^\s*@\|^\s*from\s.\+import \|^\s*from \|^\s*import \)\w*'
endif


" ----------------------------------------------------------------
" lervag/vimtex
" ----------------------------------------------------------------
if has("mac")
  let g:vimtex_view_method = 'skim'
elseif has("unix")
  let g:vimtex_view_method = 'general'
  let g:vimtex_view_general_viewer = 'okular'
  let g:vimtex_view_general_options = '--unique file:@pdf\#src:@line@tex'
  let g:vimtex_view_general_options_latexmk = '--unique'
endif
let g:vimtex_quickfix_ignore_filters = [
      \ 'Package caption Warning: Unknown document class (or package)',
      \ 'Package subfig Warning: Your document class has a bad definition',
      \ 'Package hyperref Warning: Token not allowed in a PDF string (Unicode)',
      \ 'Overfull ',
      \ 'Underfull ',
      \ 'Package minitoc(hints) Warning: ',
      \ 'LaTeX Warning: Citation ',
      \ 'LaTeX Warning: No positions in optional float specifier.',
      \ 'LaTeX Warning: There were undefined references.',
      \ 'LaTeX Font Warning:',
      \]
let g:tex_flavor = 'latex'


" ----------------------------------------------------------------
" preservim/tagbar
" ----------------------------------------------------------------
let g:tagbar_sort = 0
noremap <silent> <localleader>t :TagbarToggle<CR>


" " ----------------------------------------------------------------
" " junegunn/fzf.vim
" " ----------------------------------------------------------------
" function! FZFOpen(cmd)
"     if winnr('$') > 1 && (!&modifiable || &ft == 'nerdtree' || &ft == 'qf')
"         wincmd l
"         wincmd k
"     endif
"     execute a:cmd
" endfunction
"
" command! -bang -nargs=* Rg call fzf#vim#grep(
"     \ 'rg --column --line-number --no-heading --color=always '.shellescape(<q-args>),
"     \ 1,
"     \ fzf#vim#with_preview({'dir': system('git rev-parse --show-toplevel 2> /dev/null')[:-2], 'options': '--delimiter : --nth 4..'}),
"     \ <bang>0)
"
" function! s:find_git_root()
"     return system('git rev-parse --show-toplevel 2> /dev/null')[:-2]
" endfunction
"
" command! ProjectFiles execute 'Files' s:find_git_root()
" nnoremap <silent> <C-p> :call FZFOpen(":ProjectFiles")<CR>
"
" nnoremap <silent> <C-n> :call FZFOpen(":Rg")<CR>
"
" nnoremap <silent> <C-s> :call FZFOpen(":GFiles?")<CR>
"
" let g:coderoot = system('realpath ~/coderoot')[:-2]
" nnoremap <silent> <C-]> :call FZFOpen(":Files " . g:coderoot)<CR>
"
" function! RgAt(directory)
"     let cwd = getcwd()
"     execute "cd " . a:directory
"     call FZFOpen(":Rg")
"     execute "cd " . cwd
" endfunction
" nnoremap <silent> <C-\> :call FZFOpen(":call RgAt(g:coderoot)")<CR>
"
" let g:fzf_buffers_jump = 1
" let g:fzf_action = {
"     \ 'ctrl-o': 'tab split',
"     \ 'ctrl-x': 'split',
"     \ 'ctrl-v': 'vsplit'}

" ----------------------------------------------------------------
" ycm-core/YouCompleteMe
" ----------------------------------------------------------------
let g:ycm_clangd_binary_path = trim(system('brew --prefix llvm')).'/bin/clangd'
nnoremap <expr> <S-k> &pvw == 1 ? ":pclose<CR>h" : ":YcmCompleter GetDoc<CR> <C-w>j"
nnoremap <silent> <localleader>d :YcmCompleter GoTo<CR>


" ----------------------------------------------------------------
" tyru/open-browser.vim
" ----------------------------------------------------------------
nmap <silent> <localleader>o <Plug>(openbrowser-open)


" scrooloose/syntastic
" let g:syntastic_mode_map = {'mode': 'passive', 'active_filetypes': [], 'passive_filetypes': []}
" let g:syntastic_python_checkers = ['flake8']
" let g:syntastic_check_on_open = 1
" let g:syntastic_cpp_compiler = 'clang++'
" let g:syntastic_cpp_compiler_options = ' -std=c++11 -stdlib=libc++'

" ----------------------------------------------------------------
" telescope
" ----------------------------------------------------------------
lua << EOF
require('telescope').setup{
  defaults = {
    mappings = {
      i = {
        ["<C-j>"] = require('telescope.actions').move_selection_next,
        ["<C-k>"] = require('telescope.actions').move_selection_previous,
        ["<C-u>"] = false,  -- clear the search field
        ["<C-f>"] = require('telescope.actions').cycle_history_next,
        ["<C-b>"] = require('telescope.actions').cycle_history_prev,
        ["<C-d>"] = require('telescope.actions').preview_scrolling_down,
        ["<C-e>"] = require('telescope.actions').preview_scrolling_up,
        ["<C-q>"] = require('telescope.actions').close,
      },
    }
  }
}
require('telescope').load_extension('frecency')
EOF

nnoremap <c-p> <cmd>Telescope find_files<cr>
nnoremap <c-g> <cmd>Telescope live_grep<cr>

nnoremap <leader>ff <cmd>Telescope frecency<cr>
nnoremap <leader>fc <cmd>Telescope commands<cr>
nnoremap <leader>fh <cmd>Telescope help_tags<cr>

nnoremap <c-s> <cmd>Telescope git_status<cr>
nnoremap <c-n> <cmd>Telescope buffers<cr>

nnoremap <leader>gf <cmd>Telescope git_files<cr>
nnoremap <leader>gs <cmd>Telescope git_status<cr>
nnoremap <leader>gl <cmd>Telescope git_commits<cr>
nnoremap <leader>gb <cmd>Telescope git_bcommits<cr>
nnoremap <leader>gt <cmd>Telescope git_stash<cr>

nnoremap <leader>tr <cmd>Telescope treesitter<cr>

nnoremap <leader>gd <cmd>Git diff %<cr>

autocmd FileType TelescopePrompt call deoplete#custom#buffer_option('auto_complete', v:false)

let g:copilot_filetypes = {
  \ 'TelescopePrompt': v:false,
  \ }


" ----------------------------------------------------------------
" which-key
" ----------------------------------------------------------------
lua << EOF
  require("which-key").setup {
    -- your configuration comes here
    -- or leave it empty to use the default settings
    -- refer to the configuration section below
  }
EOF

lua << EOF
require("bufferline").setup{}
EOF

" if has('nvim')
"   tnoremap <Esc> <C-\><C-n>
"   tnoremap <M-[> <Esc>
"   tnoremap <C-v><Esc> <Esc>
" endif
"
" augroup terminal_settings
"   autocmd!
"
"   autocmd TermOpen * startinsert
"   autocmd TermOpen * setlocal nonumber norelativenumber
"   autocmd BufWinEnter,WinEnter term://* startinsert
"   autocmd BufLeave term://* stopinsert
"
"   " Ignore various filetypes as those will close terminal automatically
"   " Ignore fzf, ranger, coc
"   autocmd TermClose term://*
"         \ if (expand('<afile>') !~ "fzf") && (expand('<afile>') !~ "ranger") && (expand('<afile>') !~ "coc") |
"         \   call nvim_input('<CR>')  |
"         \ endif
" augroup END
"
" " Terminal mode:
" tnoremap <M-h> <c-\><c-n><c-w>h
" tnoremap <M-j> <c-\><c-n><c-w>j
" tnoremap <M-k> <c-\><c-n><c-w>k
" tnoremap <M-l> <c-\><c-n><c-w>l
" " Insert mode:
" inoremap <M-h> <Esc><c-w>h
" inoremap <M-j> <Esc><c-w>j
" inoremap <M-k> <Esc><c-w>k
" inoremap <M-l> <Esc><c-w>l
" " Visual mode:
" vnoremap <M-h> <Esc><c-w>h
" vnoremap <M-j> <Esc><c-w>j
" vnoremap <M-k> <Esc><c-w>k
" vnoremap <M-l> <Esc><c-w>l
" " Normal mode:
" nnoremap <M-h> <c-w>h
" nnoremap <M-j> <c-w>j
" nnoremap <M-k> <c-w>k
" nnoremap <M-l> <c-w>l
