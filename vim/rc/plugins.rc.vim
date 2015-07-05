"--------------------------------------
" Plugins
"--------------------------------------


" NeoBundle
if has('vim_starting')
  set runtimepath+=~/.vim/bundle/neobundle.vim/
endif
call neobundle#begin(expand('~/.vim/bundle/'))
NeoBundleFetch 'Shougo/neobundle.vim'


" Load NeoComplete
if has('lua')
  NeoBundle 'Shougo/neocomplete.vim'
endif
" Setup NeoComplete
if !exists('g:loaded_neocomplete')
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
endif


" Interactive terminal tool
NeoBundle 'wkentaro/conque.vim'


" roscd & rosed
NeoBundle 'ompugao/ros.vim'


" Swift syntax
" https://github.com/toyamarinyon/vim-swift
" NeoBundle 'toyamarinyon/vim-swift'


" https://github.com/tpope/vim-repeat
NeoBundle 'tpope/vim-repeat'


" indent guide
" https://github.com/nathanaelkane/vim-indent-guides
NeoBundle 'nathanaelkane/vim-indent-guides'


" Surrounding editing
" https://github.com/tpope/vim-surround
NeoBundle 'tpope/vim-surround'


" Easy commenting tool
" https://github.com/tomtom/tcomment_vim
NeoBundle 'tomtom/tcomment_vim'


" Colorful footer in vim
" https://github.com/itchyny/lightline.vim
NeoBundle 'itchyny/lightline.vim'


" git diff, log
" https://github.com/tpope/vim-fugitive
NeoBundle 'tpope/vim-fugitive'


" https://github.com/gregsexton/gitv
NeoBundle 'gregsexton/gitv'


" Open URL
" https://github.com/tyru/open-browser.vim
NeoBundle 'tyru/open-browser.vim'


if v:version > 703

  NeoBundle 'Shougo/vimfiler.vim'

  NeoBundle 'Shougo/unite.vim'

endif


NeoBundle 'Shougo/neomru.vim'


NeoBundle 'Shougo/neosnippet.vim'


NeoBundle 'honza/vim-snippets'


NeoBundle 'thinca/vim-ref'


NeoBundle 'thinca/vim-quickrun'


" Use template
NeoBundle 'thinca/vim-template'


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


" For LaTeX
NeoBundle 'gerw/vim-latex-suite'


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

call neobundle#end()


" If there are uninstalled bundles found on startup,
" this will conveniently prompt you to install them.
NeoBundleCheck
