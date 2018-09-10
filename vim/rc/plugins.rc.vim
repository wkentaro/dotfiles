" ========================================================
" Plugins
" ========================================================


" --------------------------------------------------------
" dein.vim
" --------------------------------------------------------
if &compatible
  set nocompatible
endif

let s:dein_dir = expand('~/.cache/dein')
let s:dein_repo_dir = s:dein_dir . '/repos/github.com/Shougo/dein.vim'

if &runtimepath !~# '/dein.vim'
  if !isdirectory(s:dein_repo_dir)
    execute '!git clone https://github.com/Shougo/dein.vim' s:dein_repo_dir
  endif
  execute 'set runtimepath^=' . fnamemodify(s:dein_repo_dir, ':p')
endif

if dein#load_state(s:dein_dir)
  call dein#begin(s:dein_dir)

  let s:toml = expand('~/.vim/rc/dein.toml')
  let s:lazy_toml = expand('~/.vim/rc/dein_lazy.toml')

  call dein#load_toml(s:toml, {'lazy': 0})
  call dein#load_toml(s:lazy_toml, {'lazy': 1})

  call dein#end()
  call dein#save_state()
endif

if dein#check_install()
  call dein#install()
endif

" --------------------------------------------------------
" NeoComplete
" --------------------------------------------------------
if has('lua')
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
" vim-airline
" --------------------------------------------------------
if has("gui_running")
  "let g:airline_theme='solarized'
  let g:airline_theme='tomorrow'
  let g:airline_powerline_fonts=1
else
  let g:airline_powerline_fonts=0
endif
let g:airline_mode_map = {
  \ '__' : '-',
  \ 'n' : 'N',
  \ 'i' : 'I',
  \ 'R' : 'R',
  \ 'c' : 'C',
  \ 'v' : 'V',
  \ 'V' : 'V',
  \ '' : 'V',
  \ 's' : 'S',
  \ 'S' : 'S',
  \ '' : 'S',
  \ }
let g:airline#extensions#whitespace#checks = []
let g:airline#extensions#hunks#non_zero_only = 1
" let g:airline_section_y = airline#section#create_right(['%{printf("%s%s",&fenc,&ff!="unix"?":".&ff:"")}'])
" let g:airline_section_z = airline#section#create_right(['%3l:%2c'])
let g:airline#extensions#ctrlp#color_template = 'replace'

" --------------------------------------------------------
" taglist.vim
" --------------------------------------------------------
noremap <silent> <Leader>t :TlistToggle<CR>

" --------------------------------------------------------
" riv
" --------------------------------------------------------
let g:riv_disable_folding = 1

" --------------------------------------------------------
" syntastic
" --------------------------------------------------------
set statusline+=%#warningmsg#
" set statusline+=%{SyntasticStatuslineFlag()}
set statusline+=%*
let g:syntastic_mode_map = {
\     'mode': 'passive',
\     'active_filetypes': [],
\     'passive_filetypes': [],
\}
nnoremap <silent> <Leader>e :SyntasticCheck<CR>
let g:syntastic_always_populate_loc_list = 1
let g:syntastic_auto_loc_list = 1
let g:syntastic_check_on_open = 0
let g:syntastic_check_on_wq = 0
" let g:syntastic_quiet_messages = {"level": "warning"}
" au BufNewFile,BufRead *.l let g:syntastic_quiet_messages = {"level": "error"}
let g:syntastic_python_checkers = ['flake8']
let g:syntastic_cpp_checkers = ['cpplint']
" let g:syntastic_c_remove_include_errors = 1
" let g:syntastic_cpp_remove_include_errors = 1

" --------------------------------------------------------
" clang_complete
" --------------------------------------------------------
" let g:clang_use_library=1
" let g:clang_debug=1
if has('mac')
  let g:clang_library_path = '/Library/Developer/CommandLineTools/usr/lib/'
else
  let s:codename = system("echo -n \"$(lsb_release -sc)\"")
  if s:codename == 'trusty'
    let g:clang_library_path = '/usr/lib/llvm-3.4/lib'
  elseif s:codename == 'xenial'
    let g:clang_library_path = '/usr/lib/llvm-3.8/lib'
  else
    echo "Unsupported linux version"
  endif
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

" --------------------------------------------------------
" vim-latex-suite
" --------------------------------------------------------
set grepprg=grep\ -nH\ $*
set shellslash
let g:tex_conceal=''
let tex_flavor = 'latex'
let g:Tex_DefaultTargetFormat = 'pdf'
let g:Tex_CompileRule_dvi = 'platex --interaction=nonstopmode $*'
let g:Tex_BibtexFlavor = 'pbibtex -kanji=euc'
let g:Tex_CompileRule_pdf = 'dvipdfmx $*.dvi;open $*.pdf'
let g:Tex_FormatDependency_pdf = 'dvi,pdf'
let g:Tex_ViewRule_dvi = 'xdvi'
let g:Tex_ViewRule_pdf = 'evince'
let Tex_FoldedSections=''
let Tex_FoldedEnvironments=''
let Tex_FoldedMisc=''
au BufNewFile,BufRead *.tex inoremap 、 ，
au BufNewFile,BufRead *.tex inoremap 。 ．
au BufNewFile,BufRead *.tex inoremap （ (
au BufNewFile,BufRead *.tex inoremap ） )
nnoremap <SID>I_won’t_ever_type_this <Plug>IMAP_JumpForward

" --------------------------------------------------------
" flazz/vim-colorschemes
" --------------------------------------------------------
if v:version > 703
  try
    set background=dark
    if $VIM_COLORSCHEME == ""
      colorscheme default
    else
      " colorscheme solarized
      colorscheme $VIM_COLORSCHEME
    endif
  catch /^Vim\%((\a\+)\)\=:E185/
      "colorscheme default
  endtry
  hi Normal ctermbg=none
  hi NonText ctermbg=none
  hi Normal ctermfg=none
  " hi ColorColumn ctermbg=8
  hi Visual ctermbg=0
endif

" --------------------------------------------------------
" vimfiler.vim
" --------------------------------------------------------
let g:vimfiler_as_default_explorer = 1
let g:vimfiler_safe_mode_by_default = 0
let g:vimfiler_ignore_pattern = '\%(.pyc\)$'
noremap <silent> <Leader>f :VimFiler -split -explorer<CR>

" --------------------------------------------------------
" html5.vim
" --------------------------------------------------------
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
let g:html_indent_inctags = "html,body,head,tbody"

" --------------------------------------------------------
" emmet-vim
" --------------------------------------------------------
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

" --------------------------------------------------------
" vim-template
" --------------------------------------------------------
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

" --------------------------------------------------------
" jedi-vim
" --------------------------------------------------------
autocmd FileType python setl omnifunc=jedi#completions
autocmd FileType python setl completeopt-=preview
let g:jedi#popup_on_dot = 0
let g:jedi#popup_select_first = 0
let g:jedi#completions_enabled = 1
let g:jedi#auto_vim_configuration = 1
let g:jedi#show_call_signatures = 0
let g:jedi#rename_command = '<Leader>R'

" --------------------------------------------------------
" quickrun
" --------------------------------------------------------
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
" vim-snippets
" --------------------------------------------------------
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
let g:neosnippet#snippets_directory='~/.vim/bundle/neosnippet-snippets/neosnippets,~/.vim/after/snippets'
"
" For snippet_complete marker.
" if has('conceal')
"   set conceallevel=2 concealcursor=i
" endif
"
" Change Mode ---
" nmap <C-3> <Plug>IMAP_JumpForward
" vmap <C-3> <Plug>IMAP_JumpForward

" --------------------------------------------------------
" unite.vim
" --------------------------------------------------------
let g:unite_enable_start_insert=1
nmap <silent> <C-n> :Unite -winheight=10 -direction=botright buffer<CR>
noremap <C-]> :Unite file_mru -winheight=10 -direction=botright<CR>
autocmd FileType unite call s:unite_my_settings()
function! s:unite_my_settings()
  " Overwrite settings.
  imap <buffer> <C-k>   <Plug>(unite_select_previous_line)
  imap <buffer> <C-j>   <Plug>(unite_select_next_line)
  imap <buffer> <C-c>   <Plug>(unite_exit)

  imap <silent><buffer><expr> <C-x>     unite#do_action('split')
  imap <silent><buffer><expr> <C-v>     unite#do_action('vsplit')
endfunction

" --------------------------------------------------------
" neoyank.vim
" --------------------------------------------------------
nmap <silent> <Leader>y :Unite history/yank -direction=botright<CR>

" --------------------------------------------------------
" open-browser.vim
" --------------------------------------------------------
nmap <silent> <Leader>o <Plug>(openbrowser-open)

" --------------------------------------------------------
" vim-fugitive
" --------------------------------------------------------
autocmd QuickFixCmdPost *grep* cwindow
" color settings for git diff
highlight DiffAdd    cterm=bold ctermfg=10 ctermbg=22
highlight DiffDelete cterm=bold ctermfg=10 ctermbg=52
highlight DiffChange cterm=bold ctermfg=10 ctermbg=17
highlight DiffText   cterm=bold ctermfg=10 ctermbg=21
if exists('g:loaded_fugitive')
  set statusline+=%{fugitive#statusline()}
endif
nnoremap :ga :Gwrite
nnoremap :gc :Gcommit --verbose
nnoremap :gst :Gstatus
nnoremap :gd :Gdiff

" --------------------------------------------------------
" ConqueTerm
" --------------------------------------------------------
nnoremap <silent> <Leader>ls :ConqueTerm zsh <CR>
nnoremap <silent> <Leader>is :ConqueTerm ipython <CR>

" --------------------------------------------------------
" vim-repeat
" --------------------------------------------------------
silent! call repeat#set("\<Plug>MyWonderfulMap", v:count)

" --------------------------------------------------------
" vim-indent-guides
" --------------------------------------------------------
let g:indent_guides_enable_on_vim_startup = 0
let g:indent_guides_auto_colors = 0
let g:indent_guides_start_level = 2
let g:indent_guides_guide_size = 1
autocmd VimEnter,Colorscheme * :hi IndentGuidesOdd  guibg=darkgray ctermbg=8
autocmd VimEnter,Colorscheme * :hi IndentGuidesEven  guibg=darkgray ctermbg=8
noremap <silent> <Leader>d :IndentGuidesToggle<CR>

" --------------------------------------------------------
" calendar.vim
" --------------------------------------------------------
let g:calendar_google_calendar = 1
let g:calendar_google_task = 1
let g:calendar_view = 'week'
let g:calendar_views = ['week', 'month', 'day', 'clock']


" --------------------------------------------------------
" ctrlp.vim
" --------------------------------------------------------
let g:ctrlp_map = '<c-p>'
let g:ctrlp_cmd = 'CtrlP'
let g:ctrlp_working_path_mode = 'ra'
let g:ctrlp_custom_ignore = '\v[\/]\.(git|hg|svn)$'
let g:ctrlp_user_command = ['.git', 'cd %s && git ls-files -co --exclude-standard']


" --------------------------------------------------------
" geeknote
" --------------------------------------------------------
let g:GeeknoteFormat="markdown"
