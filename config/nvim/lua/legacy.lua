-- TODO(Kentaro):check if needed
vim.cmd [[
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

" Use UTF-8 without BOM
set encoding=utf-8 nobomb
set fileencodings=ucs-bom,utf-8,iso-2022-jp,sjis,euc-jp

" use 2 spaces for indentation
set tabstop=2
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

set history=50
set commentstring=\ #\ %s
set autoindent
"set browsedir=buffer

" Make the command line two lines high and change the statusline display to
" something that looks useful.
set cmdheight=1
set laststatus=2

set showcmd
set noshowmode

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

set nofoldenable
set cinoptions+=:0,g0
]]
