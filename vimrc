" Vim Basic Setup
syntax on
set modifiable
set ttymouse=xterm2
set mouse=a
set encoding=utf-8
set textwidth=0
set softtabstop=2
set backspace=indent,eol,start
set ignorecase
set ruler
set wildmenu
set commentstring=\ #\ %s
set foldminlines=5
set foldlevel=5
set foldmethod=manual
set clipboard=unnamed,autoselect
set autoindent
set browsedir=buffer
set nocompatible
set expandtab
set hidden
set incsearch
"set list
"set listchars=eol:$,tab:>\ ,extends:<
set number
set shiftwidth=2
set showmatch
set smartcase
set smartindent
set smarttab
set tabstop=2
set whichwrap=b,s,h,l,<,>,[,]
set nowrapscan
set shiftround          " '<'や'>'でインデントする際に'shiftwidth'の倍数に丸める
set infercase           " 補完時に大文字小文字を区別しない
set virtualedit=all     " カーソルを文字が存在しない部分でも動けるようにする
set hidden              " バッファを閉じる代わりに隠す（Undo履歴を残すため）
set switchbuf=useopen   " 新しく開く代わりにすでに開いてあるバッファを開く
set showmatch           " 対応する括弧などをハイライト表示する
set matchtime=3
set matchpairs& matchpairs+=<:>
set backspace=indent,eol,start
set nowritebackup
set nobackup
set noswapfile
set list                " 不可視文字の可視化
set number              " 行番号の表示
set wrap                " 長いテキストの折り返し
set textwidth=0         " 自動的に改行が入るのを無効化
set t_vb=
set novisualbell
set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲

au BufNewFile,BufRead * set iminsert=0
au BufNewFile,BufRead * set tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.html set wrap tabstop=2 shiftwidth=2
au BufNewFile,BufRead *.py set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.php set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.sh set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.rb set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.cpp set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.c set wrap tabstop=4 shiftwidth=4
au BufNewFile,BufRead *.h set wrap tabstop=4 shiftwidth=4
highlight ZenkakuSpace cterm=underline ctermfg=lightblue guibg=#666666
au BufNewFile,BufRead * match ZenkakuSpace /　/


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

" My Bundles here:
NeoBundle 'tpope/vim-repeat'
NeoBundle 'rcmdnk/vim-markdown'
NeoBundle 'kannokanno/previm'
NeoBundle 'nathanaelkane/vim-indent-guides'
NeoBundle 'tpope/vim-surround'
NeoBundle 'tomtom/tcomment_vim'
"NeoBundle 'bronson/vim-trailing-whitespace'
NeoBundle 'Shougo/vimfiler.vim'
NeoBundle 'itchyny/lightline.vim'
NeoBundle 'tpope/vim-fugitive'
NeoBundle 'tyru/open-browser.vim'
NeoBundle 'kakkyz81/evervim'
"NeoBundle 'davidhalter/jedi-vim'
NeoBundle 'Shougo/vimshell'
NeoBundle 'Shougo/neocomplete.vim'
NeoBundle 'Shougo/neomru.vim'
NeoBundle 'Shougo/unite.vim'
NeoBundle 'Shougo/neosnippet.vim'
NeoBundle 'Shougo/neosnippet-snippets'
NeoBundle 'tpope/vim-fugitive'
NeoBundle 'thinca/vim-ref'
NeoBundle 'thinca/vim-quickrun'
NeoBundle 'flazz/vim-colorschemes'
NeoBundle 'git://git.code.sf.net/p/vim-latex/vim-latex'
NeoBundle 'thinca/vim-template'
NeoBundle 'hattya/python_fold.vim'

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

" neocompleteの設定
let g:neocomplete#enable_at_startup = 1
let g:neocomplete#enable_auto_select = 0
let g:neocomplete#enable_auto_close_preview = 1
let g:neocomplete#enable_ignore_case = 1
let s:hooks = neobundle#get_hooks("jedi-vim")
let g:jedi#popup_on_dot = 0
let g:jedi#popup_select_first = 1
"function! s:hooks.on_source(bundle)
"  " jediにvimの設定を任せると'completeopt+=preview'するので
"  " 自動設定機能をOFFにし手動で設定を行う
"  let g:jedi#auto_vim_configuration = 0
"  " 補完の最初の項目が選択された状態だと使いにくいためオフにする
"  let g:jedi#popup_select_first = 0
"  " quickrunと被るため大文字に変更
"  let g:jedi#rename_command = '<Leader>R'
"  " gundoと被るため大文字に変更 (2013-06-24 10:00 追記）
"  let g:jedi#goto_command = '<Leader>G'
"endfunction
let g:neocomplete#enable_smart_case = 1
"autocmd FileType python setlocal omnifunc=pythoncomplete#Complete
if !exists('g:neocomplete#keyword_patterns')
  let g:neocomplete#keyword_patterns = {}
endif
let g:neocomplete#keyword_patterns._ = '\h\w*'
"inoremap <expr><C-h> neocomplete#smart_close_popup()
inoremap <expr><C-h> neocomplete#close_popup()
inoremap <expr><C-e>  neocomplete#cancel_popup()
inoremap <expr><TAB> pumvisible() ? "\<C-n>" : "\<TAB>"
"inoremap <expr><S-TAB> pumvisible() ? "\<C-p>" : "\<S-TAB>"

" My Keymapping
imap <C-4> <Plug>IMAP_JumpForward
nmap <C-4> <Plug>IMAP_JumpForward
vmap <C-4> <Plug>IMAP_JumpForward
inoremap jj <Esc>
inoremap <silent> <C-j> j
inoremap <silent> kk <esc>
inoremap <silent> <C-k> k
inoremap <C-L> <Right>
inoremap <C-k> <Up>
inoremap <C-j> <Down>
inoremap <C-h> <Left>
inoremap <C-9> <esc>/<cr><esc>cf>
nnoremap <C-k> <Up>
nnoremap <C-l> <Right>
nnoremap <C-i> <Insert>
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
nnoremap j gj
nnoremap k gk

"" 画面の大きさ変更
nnoremap <S-Left>  <C-w><<CR>
nnoremap <S-Right> <C-w>><CR>
nnoremap <S-Up>    <C-w>-<CR>
nnoremap <S-Down>  <C-w>+<CR>
" map <Space><Left> <C-w><Left>
map <Space> :tabn<CR>
map <S-Space> :tabp<CR>

"" 検索
cnoremap <expr> / getcmdtype() == '/' ? '\/' : '/'
cnoremap <expr> ? getcmdtype() == '?' ? '\?' : '?'

"" VimShell
nnoremap <silent> ,is :vsp<CR>:VimShell<CR>
nnoremap <silent> ,ls :VimShell<CR>
nnoremap <silent> ,ipy :VimShellInteractive python<CR>
" ,ss: 非同期で開いたインタプリタに現在の行を評価させる
vmap <silent> ,ss :VimShellSendString<CR>
" 選択中に,ss: 非同期で開いたインタプリタに選択行を評価させる
nnoremap <silent> ,ss <S-v>:VimShellSendString<CR>

"" For LaTeX
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
"au BufNewFile,BufRead *.tex inoremap EIT
"au BufNewFile,BufRead *.tex inoremap EDO
"au BufNewFile,BufRead *.tex inoremap SSE


"" 折畳操作
" noremap [space] <nop>
" nmap <Space> [space]
" noremap [space]j zj
" noremap [space]k zk
" noremap [space]n ]z
" noremap [space]p [z
" noremap [space]h zc
" noremap [space]l zo
" noremap [space]a za
" noremap [space]m zM
" noremap [space]i zMzv
" noremap [space]r zR
" noremap [space]f zf

" vim-templates
augroup MyAutoCmd
  autocmd!
augroup END
autocmd MyAutoCmd User plugin-template-loaded call s:template_keywords()
function! s:template_keywords()
    silent! %s/<+DATE+>/\=strftime('%Y-%m-%d')/g
    "silent! %s/<+FILENAME+>/\=expand('%:r')/g
    silent! %s/<+FILENAME+>/\=expand('%r')/g
endfunction
" テンプレート中に含まれる'<+CURSOR+>'にカーソルを移動
autocmd MyAutoCmd User plugin-template-loaded
    \   if search('<+CURSOR+>')
    \ |   silent! execute 'normal! "_da>'
    \ | endif

" jedi-vim
command! -nargs=0 JediRename :call jedi#rename()
let g:jedi#rename_command = ""
let g:jedi#documentation_command = ""

" quickrun
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

" Evervim
let g:evervim_devtoken='S=s204:U=18304d3:E=14e09ed5f84:C=146b23c3260:P=1cd:A=en-devtoken:V=2:H=bd1352f1c31b347a383e2ad1909f5b56'
nnoremap <silent> ,el :<C-u>EvervimNotebookList<CR>/INBOX<CR>
nnoremap <silent> ,ew :<C-u>EvervimOpenBrowser<CR>
nnoremap <silent> ,en :<C-u>EvervimCreateNote<CR>
nnoremap <silent> ,es :<C-u>EvervimSearchByQuery<SPACE>
nnoremap <silent> ,ea :<C-u>EvervimSearchByQuery -<CR>
let g:evervim_splitoption=''

" Open-browser
nmap <silent> ,w <Plug>(openbrowser-open)

" unite
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

" fugitive
autocmd QuickFixCmdPost *grep* cwindow

" lightline
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

" vim-indent-guides
colorscheme default
let g:indent_guides_enable_on_vim_startup = 1
let g:indent_guides_auto_colors = 0
let g:indent_guides_start_level = 2
let g:indent_guides_guide_size = 1
autocmd VimEnter,Colorscheme * :hi IndentGuidesOdd  guibg=darkgray ctermbg=8
autocmd VimEnter,Colorscheme * :hi IndentGuidesEven  guibg=darkgray ctermbg=8

" vim-repeat
silent! call repeat#set("\<Plug>MyWonderfulMap", v:count)

" vim-filter
let g:vimfiler_as_default_explorer = 1
let g:vimfiler_safe_mode_by_default = 0
noremap <C-t> :VimFiler -split -explorer<CR>
noremap <silent> ,fd :VimFiler -split -project<CR>

" vim-markdown
autocmd MyAutoCmd BufNewFile,BufRead *.{md,mdwn,mkd,mkdn,mark*} set filetype=markdown
autocmd MyAutoCmd BufNewFile,BufRead *.{md,mdwn,mkd,mkdn,mark*} set foldlevel=3
autocmd MyAutoCmd FileType markdown hi! def link markdownItalic LineNr
let g:vim_markdown_codeblock_syntax=0
hi link htmlItalic LineNr
hi link htmlBold WarningMsg
hi link htmlBoldItalic ErrorMsg
"
" kobito
function! s:open_kobito(...)
  if a:0 == 0
    call system('open -a Kobito '.expand('%:p'))
  else
    call system('open -a Kobito '.join(a:000, ' '))
  endif
endfunction

" 引数のファイル(複数指定可)を Kobitoで開く
" （引数無しのときはカレントバッファを開く
command! -nargs=* Kobito call s:open_kobito(<f-args>)
" Kobito を閉じる
command! -nargs=0 KobitoClose call system("osascript -e 'tell application \"Kobito\" to quit'")
" Kobito にフォーカスを移す
command! -nargs=0 KobitoFocus call system("osascript -e 'tell application \"Kobito\" to activate'")

function! s:open_mou(...)
  if a:0 == 0
    call system('open -a Mou '.expand('%:p'))
  else
    call system('open -a Mou '.join(a:000, ' '))
  endif
endfunction
command! -nargs=* Mou call s:open_mou(<f-args>)
