" ----------------------------------------------------------------
" source_rc function
" ----------------------------------------------------------------
function! s:source_rc(path)
  execute 'source' fnameescape(expand('~/.vim/rc/' . a:path))
endfunction


" ----------------------------------------------------------------
" vim useful options
" ----------------------------------------------------------------
if has('nvim')
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
set esckeys

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
set statusline=[%l,%v\ %P%M]\ %f\ %r%h%w\ (%{&ff})
set showcmd
set noshowmode
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
set novisualbell
set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲
set cinoptions+=:0,g0


" ----------------------------------------------------------
" Filetype
" ----------------------------------------------------------
call s:source_rc('filetype.rc.vim')


" ----------------------------------------------------------
" Highlight Zenkaku Space
" ----------------------------------------------------------
highlight ZenkakuSpace cterm=underline ctermfg=lightblue guibg=#666666
au BufNewFile,BufRead * match ZenkakuSpace /　/


" ----------------------------------------------------------
" Key mappings
" ----------------------------------------------------------
call s:source_rc('mappings.rc.vim')


let g:github_access_token = $GITHUB_TOKEN


" ----------------------------------------------------------
" Plugins
" ----------------------------------------------------------
if v:version >= 704
  if &g:loadplugins
    call s:source_rc('plugins.rc.vim')
  endif
endif
