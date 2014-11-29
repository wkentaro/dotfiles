" Use Default Dark theme
set background=dark
colorscheme default
" Make vim more useful
set nocompatible
" Use the OS clipboard by default
" (on versions compiled with `+clipboard`)
set clipboard=unnamed
" Enhance command-line completion
set wildmenu
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
" Softtabs, 2 spaces
set tabstop=2
set shiftwidth=2
set shiftround
set expandtab
" Allow backspace in insert mode
set backspace=indent,eol,start
" Respect modeline in files
set modeline
set modelines=4
" Disable error bells
set noerrorbells
" Don’t reset cursor to start of line when moving around.
set nostartofline
set modifiable
set ttymouse=xterm2
" Enable mouse in all modes
set mouse=a
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
set shiftwidth=2
set showmatch
set smartcase
set smartindent
set smarttab
set whichwrap=b,s,h,l,<,>,[,]
set nowrapscan
set shiftround
set infercase
set virtualedit=all
set hidden
set switchbuf=useopen
set showmatch
set matchtime=3
set matchpairs& matchpairs+=<:>
set backspace=indent,eol,start
set nowritebackup
set nobackup
set noswapfile
set nofoldenable
set list
set number
set wrap
set t_vb=
set novisualbell
set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲
set cinoptions+=:0,g0

" release autogroup in MyAutoCmd
augroup MyAutoCmd
  autocmd!
augroup END

" Specific settings according to filetype ------------------------------
au BufNewFile,BufRead * set iminsert=0
au BufNewFile,BufRead * set tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.l set wrap ft=lisp
au BufNewFile,BufRead *.py set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.c set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cpp set wrap tabstop=4 shiftwidth=4

augroup cpp-path
  autocmd!
  autocmd FileType cpp setlocal path=.,/usr/include,/usr/local/include
augroup END

" lisp setup
let lisp_rainbow=1

" lisp
au BufNewFile,BufRead *.l set wrap tabstop=2 shiftwidth=2 ft=lisp
let lisp_rainbow = 1

" Highlight Zenkaku Space ------------------------------------
highlight ZenkakuSpace cterm=underline ctermfg=lightblue guibg=#666666
au BufNewFile,BufRead * match ZenkakuSpace /　/

" NeoBundle -------------------------------------------------  
if has('vim_starting')
  set nocompatible               " Be iMproved

  " Required:
  set runtimepath+=~/.vim/bundle/neobundle.vim/
endif

" Required:
call neobundle#begin(expand('~/.vim/bundle/'))

" Let NeoBundle manage NeoBundle
" Required:
NeoBundleFetch 'Shougo/neobundle.vim'

NeoBundle 'wkentaro/conque.vim'
NeoBundle 'ompugao/ros.vim'
NeoBundle 'toyamarinyon/vim-swift'
NeoBundle 'tpope/vim-repeat'
NeoBundle 'chreekat/vim-instant-markdown'
NeoBundle 'nathanaelkane/vim-indent-guides'
NeoBundle 'tpope/vim-surround'
NeoBundle 'tomtom/tcomment_vim'
NeoBundle 'Shougo/vimfiler.vim'
NeoBundle 'itchyny/lightline.vim'
NeoBundle 'tpope/vim-fugitive'
NeoBundle 'tyru/open-browser.vim'
" NeoBundle 'Shougo/vimshell'
if has('lua')
  NeoBundle 'Shougo/neocomplete.vim'
endif
NeoBundle 'Shougo/neomru.vim'
NeoBundle 'Shougo/unite.vim'
NeoBundle 'Shougo/neosnippet.vim'
NeoBundle 'Shougo/neosnippet-snippets'
NeoBundle 'thinca/vim-ref'
NeoBundle 'thinca/vim-quickrun'
NeoBundle 'thinca/vim-template'
NeoBundle 'flazz/vim-colorschemes'
NeoBundle 'git://git.code.sf.net/p/vim-latex/vim-latex'
NeoBundle 'hattya/python_fold.vim'
NeoBundle 'davidhalter/jedi-vim'

NeoBundleLazy 'kana/vim-altr'

let vimproc_updcmd = has('win64') ?
      \ 'tools\\update-dll-mingw 64' : 'tools\\update-dll-mingw 32'
execute "NeoBundle 'Shougo/vimproc.vim'," . string({
      \ 'build' : {
      \     'windows' : vimproc_updcmd,
      \     'cygwin' : 'make -f make_cygwin.mak',
      \     'mac' : 'make -f make_mac.mak',
      \     'unix' : 'make -f make_unix.mak',
      \    },
      \ })

call neobundle#end()

" Required:
filetype plugin indent on

" If there are uninstalled bundles found on startup,
" this will conveniently prompt you to install them.
NeoBundleCheck

" NeoComplete -------------------------------------------------  
let g:neocomplete#enable_auto_select = 0
let g:neocomplete#enable_auto_close_preview = 1
let g:neocomplete#enable_ignore_case = 1
" Disable AutoComplPop.
let g:acp_enableAtStartup = 0
" Use neocomplete.
let g:neocomplete#enable_at_startup = 1
" Use smartcase.
let g:neocomplete#enable_smart_case = 1
" Set minimum syntax keyword length.
let g:neocomplete#sources#syntax#min_keyword_length = 3
let g:neocomplete#lock_buffer_name_pattern = '\*ku\*'
" Define dictionary.
let g:neocomplete#sources#dictionary#dictionaries = {
    \ 'default' : '',
    \ 'vimshell' : $HOME.'/.vimshell_hist',
    \ 'scheme' : $HOME.'/.gosh_completions'
        \ }
" Define keyword.
if !exists('g:neocomplete#keyword_patterns')
  let g:neocomplete#keyword_patterns = {}
endif
let g:neocomplete#keyword_patterns._ = '\h\w*'
" Plugin key-mappings.
inoremap <expr><C-g>     neocomplete#undo_completion()
inoremap <expr><C-l>     neocomplete#complete_common_string()
" Recommended key-mappings.
" inoremap <silent> <CR> <C-r>=<SID>my_cr_function()<CR>
" function! s:my_cr_function()
"   return neocomplete#close_popup() . "\<CR>"
" endfunction
" <TAB>: completion.
inoremap <expr><TAB>  pumvisible() ? "\<C-n>" : "\<TAB>"
" <C-h>, <BS>: close popup and delete backword char.
" inoremap <expr><C-h> neocomplete#smart_close_popup()."\<C-h>"
" inoremap <expr><BS> neocomplete#smart_close_popup()."\<C-h>"
inoremap <expr><C-y>  neocomplete#close_popup()
inoremap <expr><C-e>  neocomplete#cancel_popup()
" Enable omni completion.
autocmd FileType css setlocal omnifunc=csscomplete#CompleteCSS
autocmd FileType html,markdown setlocal omnifunc=htmlcomplete#CompleteTags
autocmd FileType javascript setlocal omnifunc=javascriptcomplete#CompleteJS
autocmd FileType python setlocal omnifunc=pythoncomplete#Complete
autocmd FileType xml setlocal omnifunc=xmlcomplete#CompleteTags
" Enable heavy omni completion.
if !exists('g:neocomplete#sources#omni#input_patterns')
  let g:neocomplete#sources#omni#input_patterns = {}
endif
let g:neocomplete#sources#omni#input_patterns.perl = '\h\w*->\h\w*\|\h\w*::'

" clang-format
augroup cpp-clangformat
  autocmd!
  autocmd FileType c,cpp,objc noremap <C-K> :pyf /path/to/clang-format.py<CR>
augroup END

" Move Keymapping -------------------------------------------------  
nnoremap 0 $
vnoremap 0 $
nnoremap 1 ^
vnoremap 1 ^
nnoremap n nzz
nnoremap N Nzz
nnoremap * *zz
nnoremap # #zz
nnoremap g* g*zz
nnoremap g# g#zz

" Quicker window movement
nnoremap <C-j> <C-w>j
nnoremap <C-k> <C-w>k
nnoremap <C-h> <C-w>h
nnoremap <C-l> <C-w>l

" Copy Keymap
vnoremap y "+y

" Change Window Size -------------------------------------------------  
nnoremap <S-Left>  <C-w><<CR>
nnoremap <S-Right> <C-w>><CR>
nnoremap <S-Up>    <C-w>-<CR>
nnoremap <S-Down>  <C-w>+<CR>

" Change tabs -------------------------------------------------  
map <Space> :tabn<CR>
map <S-Space> :tabp<CR>

" Search -------------------------------------------------  
cnoremap <expr> / getcmdtype() == '/' ? '\/' : '/'
cnoremap <expr> ? getcmdtype() == '?' ? '\?' : '?'

" vimshell ------------------------------------------------- 
" nnoremap <silent> ,is :vsp<CR>:VimShell<CR>
" nnoremap <silent> <C-B> :sp<CR>:VimShell<CR>
" nnoremap <silent> ,ls :VimShell<CR>
" nnoremap <silent> ,ipy :VimShellInteractive python<CR>
" vmap <silent> ,ss :VimShellSendString<CR>
" nnoremap <silent> ,ss <S-v>:VimShellSendString<CR>
nnoremap <C-B> :ConqueTermSplit zsh<CR>
nnoremap <silent> ,ls :ConqueTermVSplit zsh<CR>
nnoremap <silent> ,is :ConqueTermVSplit ipython<CR>

" vim-latex ------------------------------------------------- 
filetype plugin on
let tex_flavor = 'latex'
set grepprg=grep\ -nH\ $*
set shellslash
let g:Tex_DefaultTargetFormat = 'pdf'
let g:Tex_CompileRule_dvi = 'platex --interaction=nonstopmode $*'
let g:Tex_CompileRule_pdf = 'dvipdfmx $*.dvi;open $*.pdf'
let g:Tex_FormatDependency_pdf = 'dvi,pdf'
au BufNewFile,BufRead *.tex inoremap 、 , 
au BufNewFile,BufRead *.tex inoremap 。 . 
au BufNewFile,BufRead *.tex inoremap （ (
au BufNewFile,BufRead *.tex inoremap ） )
au BufNewFile,BufRead *.tex inoremap ITM \item[]<++><esc>4hi
au BufNewFile,BufRead *.tex inoremap MBX \mbox{}<++><esc>4hi

" quickrun ------------------------------------------------- 
let g:quickrun_config = {
\   "_" : {
\       "outputter/buffer/split" : ":botright",
\       "outputter/buffer/close_on_empty" : 1,
\       "runner" : "vimproc",
\       "runner/vimproc/updatetime" : 60
\   },
\}
nnoremap <expr><silent> <C-c> quickrun#is_running() ? quickrun#sweep_sessions() : "\<C-c>"
nnoremap <silent> ,r :QuickRun<CR>

" evervim ------------------------------------------------- 
let g:evervim_devtoken='S=s204:U=18304d3:E=14e09ed5f84:C=146b23c3260:P=1cd:A=en-devtoken:V=2:H=bd1352f1c31b347a383e2ad1909f5b56'
nnoremap <silent> ,el :<C-u>EvervimNotebookList<CR>/INBOX<CR>
nnoremap <silent> ,ew :<C-u>EvervimOpenBrowser<CR>
nnoremap <silent> ,en :<C-u>EvervimCreateNote<CR>
nnoremap <silent> ,es :<C-u>EvervimSearchByQuery<SPACE>
nnoremap <silent> ,ea :<C-u>EvervimSearchByQuery -<CR>
let g:evervim_splitoption=''

" open-browser ------------------------------------------------- 
nmap <silent> ,w <Plug>(openbrowser-open)

" unite ------------------------------------------------- 
let g:unite_enable_start_insert=1
noremap <C-p> :Unite buffer<CR>
noremap <C-n> :Unite -buffer-name=file file<CR>
noremap <C-z> :Unite file_mru<CR>
noremap :uff :<C-u>UniteWithBufferDir file -buffer-name=file<CR>
au FileType unite nnoremap <silent> <buffer> <expr> <C-J> unite#do_action('split')
au FileType unite inoremap <silent> <buffer> <expr> <C-J> unite#do_action('split')
au FileType unite nnoremap <silent> <buffer> <expr> <C-K> unite#do_action('vsplit')
au FileType unite inoremap <silent> <buffer> <expr> <C-K> unite#do_action('vsplit')
au FileType unite nnoremap <silent> <buffer> <ESC><ESC> :q<CR>
au FileType unite inoremap <silent> <buffer> <ESC><ESC> <ESC>:q<CR>

" fugitive ------------------------------------------------- 
autocmd QuickFixCmdPost *grep* cwindow

" lightline ------------------------------------------------- 
set laststatus=2
let g:lightline = {
      \ 'colorscheme': 'powerline',
      \ 'mode_map': {'c': 'NORMAL'},
      \ 'active': {
      \   'left': [ ['mode', 'paste'], ['fugitive', 'filename', 'cakephp', 'currenttag', 'anzu'] ]
      \ },
      \ 'component': {
      \   'lineinfo': ' %3l:%-2v',
      \ },
      \ 'component_function': {
      \   'modified': 'MyModified',
      \   'readonly': 'MyReadonly',
      \   'fugitive': 'MyFugitive',
      \   'filename': 'MyFilename',
      \   'fileformat': 'MyFileformat',
      \   'filetype': 'MyFiletype',
      \   'fileencoding': 'MyFileencoding',
      \   'mode': 'MyMode',
      \   'anzu': 'anzu#search_status',
      \ }
      \ }


function! MyModified()
  return &ft =~ 'help\|vimfiler\|gundo' ? '' : &modified ? '+' : &modifiable ? '' : '-'
endfunction

function! MyReadonly()
  return &ft !~? 'help\|vimfiler\|gundo' && &readonly ? ' ' : ''
endfunction

function! MyFilename()
  return ('' != MyReadonly() ? MyReadonly() . ' ' : '') .
        \ (&ft == 'vimfiler' ? vimfiler#get_status_string() :
        \  &ft == 'unite' ? unite#get_status_string() :
        \  &ft == 'vimshell' ? vimshell#get_status_string() :
        \ '' != expand('%:t') ? expand('%:t') : '[No Name]') .
        \ ('' != MyModified() ? ' ' . MyModified() : '')
endfunction

function! MyFugitive()
  try
    if &ft !~? 'vimfiler\|gundo' && exists('*fugitive#head') && strlen(fugitive#head())
      return ' ' . fugitive#head()
    endif
  catch
  endtry
  return ''
endfunction

function! MyFileformat()
  return winwidth(0) > 70 ? &fileformat : ''
endfunction

function! MyFiletype()
  return winwidth(0) > 70 ? (strlen(&filetype) ? &filetype : 'no ft') : ''
endfunction

function! MyFileencoding()
  return winwidth(0) > 70 ? (strlen(&fenc) ? &fenc : &enc) : ''
endfunction

function! MyMode()
  return winwidth(0) > 60 ? lightline#mode() : ''
endfunction

" vim-indent-guides ------------------------------------------------- 
colorscheme default
let g:indent_guides_enable_on_vim_startup = 1
let g:indent_guides_auto_colors = 0
let g:indent_guides_start_level = 2
let g:indent_guides_guide_size = 1
autocmd VimEnter,Colorscheme * :hi IndentGuidesOdd  guibg=darkgray ctermbg=8
autocmd VimEnter,Colorscheme * :hi IndentGuidesEven  guibg=darkgray ctermbg=8

" vim-repeat ------------------------------------------------- 
silent! call repeat#set("\<Plug>MyWonderfulMap", v:count)

" vim-filter ------------------------------------------------- 
let g:vimfiler_as_default_explorer = 1
let g:vimfiler_safe_mode_by_default = 0
noremap <C-t> :VimFiler -split -explorer<CR>
noremap <silent> ,fd :VimFiler -split -project<CR>
noremap <silent> ,fs :VimFiler<CR>

" vim-markdown -------------------------------------------------
let g:vim_markdown_codeblock_syntax=0
hi link htmlItalic LineNr
hi link htmlBold WarningMsg
hi link htmlBoldItalic ErrorMsg

" jedi
autocmd FileType python setlocal omnifunc=jedi#completions
" let g:jedi#completions_enabled = 0
" let g:jedi#auto_vim_configuration = 0
" let s:hooks = neobundle#get_hooks("jedi-vim")
let g:jedi#popup_on_dot = 0
" let g:jedi#popup_select_first = 1

if !exists('g:neocomplete#force_omni_input_patterns')
        let g:neocomplete#force_omni_input_patterns = {}
endif
let g:neocomplete#force_omni_input_patterns.cpp =
  \ '[^.[:digit:] *\t]\%(\.\|->\)\w*\|\h\w*::\w*'
let g:neocomplete#force_omni_input_patterns.python =
  \ '\%([^. \t]\.\|^\s*@\|^\s*from\s.\+import \|^\s*from \|^\s*import \)\w*'

" neocomplete
autocmd FileType python setlocal completeopt-=preview

" cursor
inoremap {<Enter> {}<Left><CR><ESC><S-o>
inoremap [<Enter> []<Left><CR><ESC><S-o>
inoremap (<Enter> ()<Left><CR><ESC><S-o>

" template
autocmd MyAutoCmd User plugin-template-loaded call s:template_keywords()
function! s:template_keywords()
    silent! %s/<+DATE+>/\=strftime('%Y-%m-%d')/g
    silent! %s/<+FILENAME+>/\=expand('%')/g
endfunction
autocmd MyAutoCmd User plugin-template-loaded
    \   if search('<+HERE+>')
    \ |   silent! execute 'normal! "_da>'
    \ | endif

" snippet
" Plugin key-mappings.
imap <C-k>     <Plug>(neosnippet_expand_or_jump)
smap <C-k>     <Plug>(neosnippet_expand_or_jump)
xmap <C-k>     <Plug>(neosnippet_expand_target)

" SuperTab like snippets behavior.
imap <expr><TAB> neosnippet#expandable_or_jumpable() ?
\ "\<Plug>(neosnippet_expand_or_jump)"
\: pumvisible() ? "\<C-n>" : "\<TAB>"
smap <expr><TAB> neosnippet#expandable_or_jumpable() ?
\ "\<Plug>(neosnippet_expand_or_jump)"
\: "\<TAB>"

" For snippet_complete marker.
if has('conceal')
  set conceallevel=2 concealcursor=i
endif
