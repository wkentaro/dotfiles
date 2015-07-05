" Use Default Dark theme
set background=dark
colorscheme default

" Make vim more useful
set nocompatible

" Use the OS clipboard by default
" (on versions compiled with `+clipboard`)
if has('unnamedplus')
  set clipboard=unnamed,unnamedplus
else
  set clipboard=unnamed
endif

function! s:source_rc(path)
  execute 'source' fnameescape(expand('~/.vim/rc/' . a:path))
endfunction

call s:source_rc('mappings.rc.vim')

" Enhance command-line completion
set wildmenu

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

" Don’t add empty newlines at the end of files
set binary
set noeol

" Enable syntax highlighting
syntax on

" filetype detection on
filetype plugin indent on

" Softtabs, 2 spaces
set shiftround
set expandtab

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

" Highlight dynamically as pattern is typed
set incsearch
set number
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
au BufNewFile,BufRead * set iminsert=0 tabstop=2 shiftwidth=2 wrap
au BufNewFile,BufRead *.py set tabstop=4 shiftwidth=4 colorcolumn=80
au BufNewFile,BufRead *.pyx set ft=python tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cfg set ft=python tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.c set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cpp set tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.launch set tabstop=2 shiftwidth=2 ft=xml
au BufNewFile,BufRead *.php set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.sh set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.zsh set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.bash set tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.md set tabstop=4 shiftwidth=4 ft=markdown
au BufNewFile,BufRead *.tex set tabstop=2 shiftwidth=2 ft=tex


autocmd FileType cpp setlocal path=.,/usr/include,/usr/local/include,/usr/include/c++/4.8/,/opt/ros/indigo/include,/usr/include/pcl-1.7

" lisp
au BufNewFile,BufRead *.l set wrap tabstop=8 shiftwidth=2 ft=lisp
let lisp_rainbow = 1

" Highlight Zenkaku Space ------------------------------------
highlight ZenkakuSpace cterm=underline ctermfg=lightblue guibg=#666666
au BufNewFile,BufRead * match ZenkakuSpace /　/

" Plugin
" if &g:loadplugins
  call s:source_rc('plugins.rc.vim')
" endif


" ----------------------------------------------------------
" Spell checking
" ----------------------------------------------------------
" Pressing ,ss will toggle and untoggle spell checking
map <Leader>ss :setlocal spell!<cr>

" Shortcuts using <leader>
map <Leader>sn ]s
map <Leader>sp [s
map <Leader>sa zg
map <Leader>s? z=


" ----------------------------------------------------------
" Search
" ----------------------------------------------------------
" cnoremap <expr> / getcmdtype() == '/' ? '\/' : '/'
" cnoremap <expr> ? getcmdtype() == '?' ? '\?' : '?'


" jedi
autocmd FileType python setl omnifunc=jedi#completions
autocmd FileType python setl completeopt-=preview
let g:jedi#popup_on_dot = 0
let g:jedi#popup_select_first = 0
let g:jedi#completions_enabled = 1
let g:jedi#auto_vim_configuration = 1
let g:jedi#show_call_signatures = 0
let s:hooks = neobundle#get_hooks("jedi-vim")

" HTML 5 tags
syn keyword htmlTagName contained article aside audio bb canvas command
syn keyword htmlTagName contained datalist details dialog embed figure
syn keyword htmlTagName contained header hgroup keygen mark meter nav output
syn keyword htmlTagName contained progress time ruby rt rp section time
syn keyword htmlTagName contained source figcaption
syn keyword htmlArg contained autofocus autocomplete placeholder min max
syn keyword htmlArg contained contenteditable contextmenu draggable hidden
syn keyword htmlArg contained itemprop list sandbox subject spellcheck
syn keyword htmlArg contained novalidate seamless pattern formtarget
syn keyword htmlArg contained formaction formenctype formmethod
syn keyword htmlArg contained sizes scoped async reversed sandbox srcdoc
syn keyword htmlArg contained hidden role
syn match   htmlArg "\<\(aria-[\-a-zA-Z0-9_]\+\)=" contained
syn match   htmlArg contained "\s*data-[-a-zA-Z0-9_]\+"

" emmet-vim
let g:user_emmet_mode = 'iv'
let g:user_emmet_leader_key = '<C-Y>'
let g:use_emmet_complete_tag = 1
let g:user_emmet_settings = {
      \ 'lang' : 'ja',
      \ 'html' : {
      \   'extends' : 'html',
      \   'filters' : 'html',
      \ },
      \ 'css' : {
      \   'filters' : 'fc',
      \ },
      \ 'php' : {
      \   'extends' : 'html',
      \   'filters' : 'html',
      \ },
      \}
augroup EmmitVim
  autocmd!
  autocmd FileType * let g:user_emmet_settings.indentation = '  '[:&tabstop]
augroup END

" taglist.vim
noremap <silent> <Leader>t :TlistToggle<CR>

" syntastic
set statusline+=%#warningmsg#
" set statusline+=%{SyntasticStatuslineFlag()}
set statusline+=%*
let g:syntastic_mode_map = { 'mode': 'passive', 'active_filetypes': [], 'passive_filetypes': [] }
nnoremap <silent> <Leader>e :SyntasticCheck<CR>

let g:syntastic_always_populate_loc_list = 1
let g:syntastic_auto_loc_list = 1
let g:syntastic_check_on_open = 0
let g:syntastic_check_on_wq = 0
let g:syntastic_quiet_messages = {"level": "warning"}
" au BufNewFile,BufRead *.l let g:syntastic_quiet_messages = {"level": "error"}
let g:syntastic_python_checkers = ['pyflakes']
let g:syntastic_c_remove_include_errors = 1
let g:syntastic_cpp_remove_include_errors = 1
let g:syntastic_cpp_config_file = '~/.clang_complete'

" clang_complete
" cmd option
" let g:clang_use_library=1
" let g:clang_debug=1
if has('mac')
  let g:clang_library_path = '/Library/Developer/CommandLineTools/usr/lib/'
else
  " let g:clang_library_path = '/usr/lib'
  let g:clang_library_path = '/usr/lib/llvm-3.4/lib'
endif
let g:clang_user_options = '-std=c++11'
" use with neocomplete.vim
if !exists('g:neocomplete#force_omni_input_patterns')
  let g:neocomplete#force_omni_input_patterns = {}
endif
let g:neocomplete#force_overwrite_completefunc = 1
let g:neocomplete#force_omni_input_patterns.c =
      \ '[^.[:digit:] *\t]\%(\.\|->\)\w*'
let g:neocomplete#force_omni_input_patterns.cpp =
      \ '[^.[:digit:] *\t]\%(\.\|->\)\w*\|\h\w*::\w*'
let g:neocomplete#force_omni_input_patterns.objc =
      \ '[^.[:digit:] *\t]\%(\.\|->\)\w*'
let g:neocomplete#force_omni_input_patterns.objcpp =
      \ '[^.[:digit:] *\t]\%(\.\|->\)\w*\|\h\w*::\w*'
" don't auto complete with clang_complete
let g:clang_complete_auto = 0
let g:clang_auto_select = 0

" for InstantRst
let g:instant_rst_browser = 'chrome'
let g:instant_rst_forever = 1

highlight DiffAdd    cterm=bold ctermfg=10 ctermbg=22
highlight DiffDelete cterm=bold ctermfg=10 ctermbg=52
highlight DiffChange cterm=bold ctermfg=10 ctermbg=17
highlight DiffText   cterm=bold ctermfg=10 ctermbg=21

" for git-commit
autocmd FileType gitcommit :set dictionary=~/.vim/dict/github_users.dict
