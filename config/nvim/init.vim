" ----------------------------------------------------------------
" vim useful options
" ----------------------------------------------------------------
if !has('nvim')
  " make vim more useful
  set nocompatible
endif

" colorscheme
set background=dark
colorscheme default

" Enable syntax highlighting
syntax on

" clipboard
" Use the OS clipboard by default (on versions compiled with `+clipboard`)
if has('unnamedplus')
  set clipboard=unnamed,unnamedplus
else
  set clipboard=unnamed
endif

" Enhance command-line completion
set wildmenu
set wildignore+=*.dll,*.o,*.pyc,*.bak,*.exe,*.jpg,*.jpeg,*.png,*.gif,*$py.class,*.class,*/*.dSYM/*,*.dylib,*.so,*.swp,*.zip,*.tgz,*.gz
set wildmode=list,full

" Ignore compiled files
set wildignore=*.o,*~,*.pyc
if has("win16") || has("win32")
  set wildignore+=*/.git/*,*/.hg/*,*/.svn/*,*/.DS_Store
else
  set wildignore+=.git\*,.hg\*,.svn\*
endif

" Allow cursor keys in insert mode
" set esckeys

" Optimize for fast terminal connections
if has('nvim')
  set ttyfast
endif

" Use UTF-8 without BOM
set encoding=utf-8 nobomb
set fileencodings=ucs-bom,utf-8,iso-2022-jp,sjis,euc-jp

" Don’t add empty newlines at the end of files
set binary
"set noeol

" filetype detection on
filetype plugin indent on

" use space aside from tab
set expandtab

" use 2 spaces for indentation
set tabstop=2
set shiftwidth=2
set shiftround
set iminsert=0

" wrap for long line
set wrap

" Allow backspace in insert mode
set backspace=indent,eol,start

" Respect modeline in files
set modeline
set modelines=4

" No annoying sound on errors
set noerrorbells
set novisualbell
if has('nvim')
  set t_vb=
endif
set tm=500

" Don’t reset cursor to start of line when moving around.
set nostartofline
set modifiable
if !has('nvim')
  set ttymouse=xterm2
endif

" Enable mouse in all modes
if has('mouse')
  set mouse=a
endif

" Ignore case of searches
set ignorecase

" Show the cursor position
set ruler

" Open new split panes to right and bottom, which feels more natural
set splitbelow
set splitright
set history=50
set commentstring=\ #\ %s
set autoindent
set browsedir=buffer
set expandtab
set hidden

" Make the command line two lines high and change the statusline display to
" something that looks useful.
set cmdheight=1
set laststatus=2
" set statusline=[%l,%v\ %P%M]\ %f\ %r%h%w\ (%{&ff})
set statusline=[%{getcwd()}]\ %f\ [%P%M]

set showcmd
set noshowmode
set number

" Highlight dynamically as pattern is typed
" set nohlsearch
set incsearch
set showmatch
set smartcase
set smartindent
set smarttab
set whichwrap=b,s,h,l,<,>,[,]
set nowrapscan
set shiftround
set infercase
set virtualedit=all

" A buffer becomes hidden when it is abandoned
set hidden

set switchbuf=useopen
set showmatch
set matchtime=3
set matchpairs& matchpairs+=<:>
set backspace=indent,eol,start
set nowritebackup


" ----------------------------------------------------------------
" Files, backups and undo
" ----------------------------------------------------------------
" Turn backup off, since most stuff is in SVN, git et.c anyway...
" set nobackup
" set nowb
set noswapfile

" set autochdir
" set undofile
set nofoldenable
set list
"set number
set wrap
set novisualbell
set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲
set cinoptions+=:0,g0

if has("autocmd")
  au BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$")
    \| exe "normal! g'\"" | endif
endif


" ----------------------------------------------------------
" Key mappings
" ----------------------------------------------------------
" Mapleader "{{{
let mapleader="\<space>"
let maplocalleader=','
"}}}


"nnoremap ; :

" FIXME: C-h does not work...
" imap <C-h> <BS>


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
" nmap gr gT

" ----------------------------------------------------------
" Window keymap
" ----------------------------------------------------------
" Quicker window movement
" nnoremap <C-j> <C-w>j
" nnoremap <C-k> <C-w>k
" nnoremap <C-h> <C-w>h
" nnoremap <C-l> <C-w>l

" Change window size
" nnoremap <S-Left>  <C-w>><CR>
" nnoremap <S-Right> <C-w><<CR>
" nnoremap <S-Up>    <C-w>+<CR>
" nnoremap <S-Down>  <C-w>-<CR>
"}}}


" ----------------------------------------------------------
" File quiting
" ----------------------------------------------------------
" nmap <localleader>w :w<CR>
" nmap <localleader>q :q<cr>


" ----------------------------------------------------------
" Spell checking
" ----------------------------------------------------------
" map <localleader>ss :setlocal spell!<cr>

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

" ----------------------------------------------------------
" Terminal
" ----------------------------------------------------------
if has('nvim')
  tnoremap <Esc> <C-\><C-n>
  tnoremap <C-[> <C-\><C-n>

  augroup terminal_settings
    autocmd!

    autocmd TermOpen * startinsert
    autocmd TermOpen * setlocal nonumber norelativenumber
    autocmd BufWinEnter,WinEnter term://* if IsOnThePrompt('%') | startinsert | endif

    " Ignore various filetypes as those will close terminal automatically
    " Ignore fzf, ranger, coc
    autocmd TermClose term://*
      \ if (expand('<afile>') !~ "fzf") && (expand('<afile>') !~ "ranger") && (expand('<afile>') !~ "coc") |
      \   call nvim_input('<CR>')  |
      \ endif
  augroup END

  tnoremap <C-Enter> <Enter>

  tnoremap <M-n> <C-\><C-n>gt
  inoremap <M-n> <Esc>gt
  vnoremap <M-n> <Esc>gt
  nnoremap <M-n> gt

  tnoremap <M-p> <C-\><C-n>gT
  inoremap <M-p> <Esc>gT
  vnoremap <M-p> <Esc>gT
  nnoremap <M-p> gT

  " tnoremap <M-n> <CMD>Telescope buffers<CR>
  " inoremap <M-n> <Esc><CMD>Telescope buffers<CR>
  " vnoremap <M-n> <Esc><CMD>Telescope buffers<CR>
  " nnoremap <M-n> <CMD>Telescope buffers<CR>

  " tnoremap <M-p> <CMD>Telescope find_files<CR>
  " inoremap <M-p> <Esc><CMD>Telescope find_files<CR>
  " vnoremap <M-p> <Esc><CMD>Telescope find_files<CR>
  " nnoremap <M-p> <CMD>Telescope find_files<CR>

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

" ----------------------------------------------------------
" Plugins
" ----------------------------------------------------------
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
  Plug 'nvim-telescope/telescope.nvim'

  Plug 'tami5/sql.nvim'  " sudo apt-get install sqlite3 libsqlite3-dev
  Plug 'nvim-telescope/telescope-frecency.nvim'

  Plug 'airblade/vim-rooter'

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

  Plug 'brettanomyces/nvim-editcommand'

  Plug 'romainl/vim-cool'

  Plug 'catppuccin/nvim', { 'branch': 'main', 'as': 'catppuccin'}
call plug#end()


" ----------------------------------------------------------------
" flazz/vim-colorschemes
" ----------------------------------------------------------------
let g:solarized_termtrans = 1
if $VIM_COLORSCHEME == "solarized"
  colorscheme solarized
endif
hi Normal ctermfg=none
hi PmenuSel ctermbg=none
"hi TabLineSel ctermfg=LightBlue ctermbg=Black
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
nnoremap <silent> <localleader>t :TagbarToggle<CR>


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
" nnoremap <silent> <localleader>d :YcmCompleter GoTo<CR>


" ----------------------------------------------------------------
" tyru/open-browser.vim
" ----------------------------------------------------------------
" nmap <silent> <localleader>o <Plug>(openbrowser-open)


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
local actions = require('telescope.actions')
require('telescope').setup{
  defaults = {
    mappings = {
      i = {
        ["<C-d>"] = false,  -- -> delete_buffer
        ["<C-j>"] = actions.move_selection_next,
        ["<C-k>"] = actions.move_selection_previous,
        ["<C-u>"] = false,  -- -> clear the search field
        ["<C-f>"] = actions.cycle_history_next,
        ["<C-b>"] = actions.cycle_history_prev,
        ["<C-n>"] = actions.preview_scrolling_down,
        ["<C-p>"] = actions.preview_scrolling_up,
        ["<C-q>"] = actions.close,
      },
    },
  },
  pickers = {
    buffers = {
      mappings = {
        i = {
          ["<c-d>"] = actions.delete_buffer,
        },
      },
    },
  },
}
require('telescope').load_extension('frecency')
EOF
let g:rooter_cd_cmd = 'lcd'

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


lua << EOF
-- External dependancies
local actions = require("telescope.actions")
local actions_set = require("telescope.actions.set")
local actions_state = require("telescope.actions.state")
local conf = require("telescope.config").values
local entry_display = require("telescope.pickers.entry_display")
local finders = require("telescope.finders")
local from_entry = require("telescope.from_entry")
local log = require("telescope.log")
local pickers = require("telescope.pickers")
local previewers = require("telescope.previewers")
local t_utils = require("telescope.utils")
local Path = require("plenary.path")

function telescope_find_dir(opts)
  pickers.new(opts, {
    prompt_title = "Find Directory",
    finder = finders.new_oneshot_job({ "fdfind", "^\\.git$", "--hidden", "--type", "d", "--absolute-path", vim.fn.expand("~/workspaces"), "--exec", "dirname" }),
    sorter = conf.generic_sorter(opts),
    attach_mappings = function(prompt_bufnr, map)
      actions_set.select:replace(function()
        local entry = actions_state.get_selected_entry()
        local dir = from_entry.path(entry)
        if entry ~= nil then
          actions.close(prompt_bufnr, false)
          vim.cmd("cd " .. dir)
          vim.cmd("echon ''")
          print("cwd: " .. vim.fn.getcwd())
        end
      end)
      return true
    end,
  }):find()
end
EOF

" ----------------------------------------------------------------
" which-key
" ----------------------------------------------------------------
lua << EOF
  local which_key = require("which-key")
  which_key.setup {}
  which_key.register({
    f = { "<cmd>Telescope find_files<cr>", "Find file" },
    t = { "<cmd>terminal<cr>", "Open terminal" },

    c = { "<cmd>lua telescope_find_dir()<cr>", "Change directory" },

    w = "Window commands",
    wv = { "<cmd>vsplit<cr>", "Vsplit window" },
    wh = { "<cmd>split<cr>", "Split" },

    b = "Buffer commands",
    bk = { "<cmd>bp<bar>bd#<cr>", "Kill buffer" },

    g = "Git commands",
    gs = { "<cmd>Git<cr>", "Git status" },
    gd = { "<cmd>Git diff<cr>", "Git diff" },
  }, { prefix = "<leader>" })
EOF

lua << EOF
require("bufferline").setup{
  highlights = {
    fill = {
      bg = 'none',
    }
  },
}
EOF

let g:editcommand_prompt = '%'

let g:catppuccin_flavour = "mocha" " latte, frappe, macchiato, mocha

lua << EOF
require("catppuccin").setup()
EOF

colorscheme catppuccin

autocmd TermOpen * set ft=terminal
let g:Illuminate_ftblacklist = ["terminal"]

" ----------------------------------------------------------
" Filetype
" ----------------------------------------------------------
autocmd FileType python set tabstop=4
autocmd FileType python set shiftwidth=4
autocmd FileType python set indentkeys-=:
autocmd FileType python inoremap <localleader>p from IPython.core.debugger import Pdb; ipdb = Pdb(); print("[ipdb] >>> "); ipdb.set_trace()<esc>
autocmd FileType python inoremap <localleader>i import IPython; print("[ipython] >>> "); IPython.embed()<esc>
" autocmd FileType python nnoremap <localleader>f :w<cr> :!flake8 %<cr>

if $USER == 'mujin'
  autocmd FileType python xnoremap <localleader>b :!blacken 110<CR>
else
  autocmd FileType python noremap <localleader>b :!black %<CR>
endif