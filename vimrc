" ----------------------------------------------------------------
" source_rc function
" ----------------------------------------------------------------
function! s:source_rc(path)
  execute 'source' fnameescape(expand('~/.vim/rc/' . a:path))
endfunction


" ----------------------------------------------------------------
" vim useful options
" ----------------------------------------------------------------
" make vim more useful
set nocompatible

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
set wildignore+=*.dll,*.o,*.pyc,*.bak,*.exe,*.jpg,*.jpeg,*.png,*.gif,*$py.class,*.class,*/*.dSYM/*,*.dylib
set wildmode=list:full

" Ignore compiled files
set wildignore=*.o,*~,*.pyc
if has("win16") || has("win32")
  set wildignore+=*/.git/*,*/.hg/*,*/.svn/*,*/.DS_Store
else
  set wildignore+=.git\*,.hg\*,.svn\*
endif

" Allow cursor keys in insert mode
set esckeys

" Optimize for fast terminal connections
set ttyfast

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
set t_vb=
set tm=500

" Don’t reset cursor to start of line when moving around.
set nostartofline
set modifiable
set ttymouse=xterm2

" set hightlight search
" set hlsearch

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
set cmdheight=2
set laststatus=2
set statusline=[%l,%v\ %P%M]\ %f\ %r%h%w\ (%{&ff})\ %{fugitive#statusline()}
set showcmd
set number

" Highlight dynamically as pattern is typed
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
set nobackup
set nowb
set noswapfile

" set undofile
set nofoldenable
set list
set number
set wrap
set t_vb=
set novisualbell
set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲
set cinoptions+=:0,g0


" ----------------------------------------------------------------
" Filetype
" ----------------------------------------------------------------
" Specific settings according to filetype
au BufNewFile,BufRead *.sh set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.zsh set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.bash set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.py set tabstop=4 shiftwidth=4 colorcolumn=80
au BufNewFile,BufRead *.pyx set ft=python tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cfg set ft=python tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.c set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cpp set tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.launch set tabstop=2 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.php set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.bash set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.md set tabstop=4 shiftwidth=4 ft=markdown
au BufNewFile,BufRead *.tex set tabstop=2 shiftwidth=2 ft=tex


" ----------------------------------------------------------
" Cpp setting
" ----------------------------------------------------------
autocmd FileType cpp setlocal path=.,/usr/include,/usr/local/include,/usr/include/c++/4.8/,/opt/ros/indigo/include,/usr/include/pcl-1.7


" ----------------------------------------------------------
" Lisp setting
" ----------------------------------------------------------
au BufNewFile,BufRead *.l set wrap tabstop=8 shiftwidth=2 ft=lisp
let lisp_rainbow = 1


" ----------------------------------------------------------
" Highlight Zenkaku Space
" ----------------------------------------------------------
highlight ZenkakuSpace cterm=underline ctermfg=lightblue guibg=#666666
au BufNewFile,BufRead * match ZenkakuSpace /　/


" ----------------------------------------------------------
" Search
" ----------------------------------------------------------
" cnoremap <expr> / getcmdtype() == '/' ? '\/' : '/'
" cnoremap <expr> ? getcmdtype() == '?' ? '\?' : '?'


" ----------------------------------------------------------
" Key mappings
" ----------------------------------------------------------
call s:source_rc('mappings.rc.vim')


" ----------------------------------------------------------
" Plugins
" ----------------------------------------------------------
if &g:loadplugins
  call s:source_rc('plugins.rc.vim')
endif
