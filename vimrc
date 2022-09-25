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
set noeol

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
"set number

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
execute 'source' '~/.vim/rc/mappings.rc.vim'

" ----------------------------------------------------------
" Plugins
" ----------------------------------------------------------
if &g:loadplugins
  if v:version >= 704
    execute 'source' '~/.vim/rc/plugins.rc.vim'
  endif
endif

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
