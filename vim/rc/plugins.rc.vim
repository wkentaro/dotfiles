"--------------------------------------
" Plugins
"--------------------------------------


" --------------------------------------------------------
" NeoBundle
" --------------------------------------------------------
if has('vim_starting')
  set runtimepath+=~/.vim/bundle/neobundle.vim/
endif
call neobundle#begin(expand('~/.vim/bundle/'))
NeoBundleFetch 'Shougo/neobundle.vim'


" --------------------------------------------------------
" NeoComplete
" --------------------------------------------------------
if has('lua')
  NeoBundle 'Shougo/neocomplete.vim'

  " Disable AutoComplPop.
  let g:acp_enableAtStartup = 0

  " Use neocomplete.
  let g:neocomplete#enable_at_startup = 1

  " Use smartcase.
  let g:neocomplete#enable_smart_case = 1

  " Set minimum syntax keyword length.
  let g:neocomplete#sources#syntax#min_keyword_length = 3
  let g:neocomplete#lock_buffer_name_pattern = '\*ku\*'
  let g:neocomplete#enable_auto_select = 0
  let g:neocomplete#enable_auto_close_preview = 1
  let g:neocomplete#enable_ignore_case = 1

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
  " inoremap <expr><TAB>  pumvisible() ? "\<C-n>" : "\<TAB>"
  " <C-h>, <BS>: close popup and delete backword char.
  " inoremap <expr><C-h> neocomplete#smart_close_popup()."\<C-h>"
  " inoremap <expr><BS> neocomplete#smart_close_popup()."\<C-h>"
  " noremap <expr><C-y>  neocomplete#close_popup()
  inoremap <expr><C-e>  neocomplete#cancel_popup()

  " Enable omni completion.
  autocmd FileType css setl omnifunc=csscomplete#CompleteCSS
  autocmd FileType html,markdown setl omnifunc=htmlcomplete#CompleteTags
  autocmd FileType javascript setl omnifunc=javascriptcomplete#CompleteJS
  autocmd FileType python setl omnifunc=pythoncomplete#Complete
  autocmd filetype python setl smartindent cinwords=if,elif,else,for,while,try,except,finally,def,class
  autocmd FileType xml setl omnifunc=xmlcomplete#CompleteTags

  " Enable heavy omni completion.
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


" --------------------------------------------------------
" ConqueTerm
" --------------------------------------------------------
" Interactive terminal tool
NeoBundle 'wkentaro/conque.vim'
nnoremap <silent> <Leader>ls :ConqueTerm zsh <CR>
nnoremap <silent> <Leader>is :ConqueTerm ipython <CR>


" --------------------------------------------------------
" ros.vim
" --------------------------------------------------------
" roscd & rosed
NeoBundle 'ompugao/ros.vim'


" --------------------------------------------------------
" vim-swift
" --------------------------------------------------------
" Swift syntax
" https://github.com/toyamarinyon/vim-swift
" NeoBundle 'toyamarinyon/vim-swift'


" --------------------------------------------------------
" vim-repeat
" --------------------------------------------------------
" https://github.com/tpope/vim-repeat
NeoBundle 'tpope/vim-repeat'
silent! call repeat#set("\<Plug>MyWonderfulMap", v:count)


" --------------------------------------------------------
" vim-indent-guides
" --------------------------------------------------------
" https://github.com/nathanaelkane/vim-indent-guides
NeoBundle 'nathanaelkane/vim-indent-guides'
let g:indent_guides_enable_on_vim_startup = 0
let g:indent_guides_auto_colors = 0
let g:indent_guides_start_level = 2
let g:indent_guides_guide_size = 1
autocmd VimEnter,Colorscheme * :hi IndentGuidesOdd  guibg=darkgray ctermbg=8
autocmd VimEnter,Colorscheme * :hi IndentGuidesEven  guibg=darkgray ctermbg=8
noremap <silent> <Leader>g :IndentGuidesToggle<CR>


" --------------------------------------------------------
" vim-surround
" --------------------------------------------------------
" Surrounding editing
" https://github.com/tpope/vim-surround
NeoBundle 'tpope/vim-surround'


" --------------------------------------------------------
" tcomment_vim
" --------------------------------------------------------
" Easy commenting tool
" https://github.com/tomtom/tcomment_vim
NeoBundle 'tomtom/tcomment_vim'


" --------------------------------------------------------
" lightline.vim
" --------------------------------------------------------
" Colorful footer in vim
" https://github.com/itchyny/lightline.vim
NeoBundle 'itchyny/lightline.vim'
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


" --------------------------------------------------------
" vim-fugitive
" --------------------------------------------------------
" https://github.com/tpope/vim-fugitive
" git diff, log
NeoBundle 'tpope/vim-fugitive'
autocmd QuickFixCmdPost *grep* cwindow


" --------------------------------------------------------
" gitv
" --------------------------------------------------------
" https://github.com/gregsexton/gitv
NeoBundle 'gregsexton/gitv'


" --------------------------------------------------------
" open-browser.vim
" --------------------------------------------------------
" https://github.com/tyru/open-browser.vim
" Open URL
NeoBundle 'tyru/open-browser.vim'
nmap <silent> <Leader>o <Plug>(openbrowser-open)



" --------------------------------------------------------
" vimfiler.vim
" --------------------------------------------------------
if v:version > 703
  NeoBundle 'Shougo/vimfiler.vim'
  let g:vimfiler_as_default_explorer = 1
  let g:vimfiler_safe_mode_by_default = 0
  noremap <silent> <Leader>f :VimFiler -split -explorer<CR>
endif



" --------------------------------------------------------
" unite.vim
" --------------------------------------------------------
if v:version > 703
  NeoBundle 'Shougo/unite.vim'
endif
let g:unite_enable_start_insert=1
noremap <C-p> :Unite buffer -winheight=10<CR>
noremap <C-n> :Unite -buffer-name=file file<CR>
noremap <C-]> :Unite file_mru<CR>
noremap :uff :<C-u>UniteWithBufferDir file -buffer-name=file<CR>
au FileType unite nnoremap <silent> <buffer> <expr> <C-J> unite#do_action('split')
au FileType unite inoremap <silent> <buffer> <expr> <C-J> unite#do_action('split')
au FileType unite nnoremap <silent> <buffer> <expr> <C-K> unite#do_action('vsplit')
au FileType unite inoremap <silent> <buffer> <expr> <C-K> unite#do_action('vsplit')
au FileType unite nnoremap <silent> <buffer> <C-c> :q<CR>
au FileType unite inoremap <silent> <buffer> <C-c> <ESC>:q<CR>


NeoBundle 'Shougo/neomru.vim'


NeoBundle 'Shougo/neosnippet.vim'
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
let g:neosnippet#disable_runtime_snippets = {
\   '_' : 1,
\ }
" Enable snipMate compatibility feature.
let g:neosnippet#enable_snipmate_compatibility = 1
" Tell Neosnippet about the other snippets
let g:neosnippet#snippets_directory='~/.vim/bundle/vim-snippets/snippets,~/.vim/after/snippets'

" For snippet_complete marker.
" if has('conceal')
"   set conceallevel=2 concealcursor=i
" endif

" Change Mode ---
" nmap <C-3> <Plug>IMAP_JumpForward
" vmap <C-3> <Plug>IMAP_JumpForward


NeoBundle 'honza/vim-snippets'


NeoBundle 'thinca/vim-ref'


" --------------------------------------------------------
" unite.vim
" --------------------------------------------------------
NeoBundle 'thinca/vim-quickrun'
let g:quickrun_config = {
\   "_" : {
\       "outputter/buffer/split" : ":botright 8sp",
\       "outputter/buffer/close_on_empty" : 1,
\       "outputter" : "error",
\       "outputter/error/success" : "buffer",
\       "outputter/error" : "quickfix",
\       "runner" : "vimproc",
\       "runner/vimproc/updatetime" : 60
\   },
\}
nnoremap <expr><silent> <C-c> quickrun#is_running() ? quickrun#sweep_sessions() : "\<C-c>"
nnoremap <silent> <Leader>r :QuickRun<CR>


" --------------------------------------------------------
" vim-template
" --------------------------------------------------------
NeoBundle 'thinca/vim-template'
augroup MyAutoCmd
  autocmd!
augroup END
autocmd MyAutoCmd User plugin-template-loaded call s:template_keywords()
function! s:template_keywords()
    silent! %s/<+DATE+>/\=strftime('%Y-%m-%d')/g
    silent! %s/<+FILENAME+>/\=expand('%')/g
endfunction
autocmd MyAutoCmd User plugin-template-loaded
    \   if search('<+HERE+>')
    \ |   silent! execute 'normal! "_da>'
    \ | endif


" Add colorschemes
NeoBundle 'flazz/vim-colorschemes'


" python folding setting
" NeoBundle 'hattya/python_fold.vim'


" python completion
NeoBundle 'davidhalter/jedi-vim'


" For HTML and XML
NeoBundle 'mattn/emmet-vim'   " C-y
NeoBundle 'othree/html5.vim'  " indentation


" For CSS
NeoBundle 'hail2u/vim-css3-syntax'


" For JavaScript
NeoBundle 'pangloss/vim-javascript'


" NeoBundle 'kana/vim-altr'


" Show variables and functions
NeoBundle 'vim-scripts/taglist.vim'


" NeoBundle 'vim-scripts/L9'


" For Scala
" NeoBundle 'derekwyatt/vim-scala'


" Syntastic Check
NeoBundle 'scrooloose/syntastic'


" For C, C++
NeoBundle 'Rip-Rip/clang_complete'
NeoBundle 'greyblake/vim-preview'


" --------------------------------------------------------
" vim-latex-suite
" --------------------------------------------------------
" https://github.com/gerw/vim-latex-suite
NeoBundle 'gerw/vim-latex-suite'
set grepprg=grep\ -nH\ $*
set shellslash
let g:tex_conceal=''
let tex_flavor = 'latex'
let g:Tex_DefaultTargetFormat = 'pdf'
let g:Tex_CompileRule_dvi = 'platex --interaction=nonstopmode $*'
let g:Tex_BibtexFlavor = 'jbibtex'
let g:Tex_CompileRule_pdf = 'dvipdfmx $*.dvi;open $*.pdf'
let g:Tex_FormatDependency_pdf = 'dvi,pdf'
let g:Tex_ViewRule_dvi = 'xdvi'
let g:Tex_ViewRule_pdf = 'evince'
let Tex_FoldedSections=''
let Tex_FoldedEnvironments=''
let Tex_FoldedMisc=''
au BufNewFile,BufRead *.tex inoremap 、 , 
au BufNewFile,BufRead *.tex inoremap 。 . 
au BufNewFile,BufRead *.tex inoremap （ (
au BufNewFile,BufRead *.tex inoremap ） )
nnoremap <SID>I_won’t_ever_type_this <Plug>IMAP_JumpForward


" Python syntax
" NeoBundle 'klen/python-mode'


" vim-tmux seamless move
NeoBundle 'christoomey/vim-tmux-navigator'


" ignore git ignored files
" https://github.com/vim-scripts/gitignore
NeoBundle 'vim-scripts/gitignore'


" vimproc
" https://github.com/Shougo/vimproc.vim
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


" NeoBundle end
call neobundle#end()

" If there are uninstalled bundles found on startup,
" this will conveniently prompt you to install them.
NeoBundleCheck
