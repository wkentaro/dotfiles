call plug#begin('~/.vim/plugged')
  Plug 'flazz/vim-colorschemes'

  Plug 'davidhalter/jedi-vim'

  if has('nvim')
    Plug 'Shougo/deoplete.nvim', { 'do': ':UpdateRemotePlugins' }

    Plug 'zchee/deoplete-jedi'
  else
    Plug 'Shougo/neocomplete.vim'
  endif

  Plug 'scrooloose/syntastic'

  Plug 'aperezdc/vim-template'

  Plug 'tomtom/tcomment_vim'

  Plug 'tyru/open-browser.vim'

  Plug 'Shougo/unite.vim'
  Plug 'Shougo/neomru.vim'

  Plug 'Shougo/vimfiler.vim'

  Plug 'ctrlpvim/ctrlp.vim'

  Plug 'Shougo/neoyank.vim'

  Plug 'Shougo/neosnippet.vim'

  Plug 'rhysd/committia.vim'

  Plug 'hotwatermorning/auto-git-diff'

  Plug 'tpope/vim-fugitive'

  Plug 'lervag/vimtex'
call plug#end()

" flazz/vim-colorschemes
syntax on
set background=dark
colorscheme solarized
hi Normal ctermbg=none
hi NonText ctermbg=none
hi Normal ctermfg=none
hi ColorColumn ctermbg=8
hi Visual ctermbg=0

" scrooloose/syntastic
let g:syntastic_python_checkers = ['flake8']
let g:syntastic_check_on_open = 1
let g:syntastic_cpp_compiler = 'clang++'
let g:syntastic_cpp_compiler_options = ' -std=c++11 -stdlib=libc++'

" aperezdc/vim-template
let g:templates_directory = ['~/.vim/after/templates']
let g:templates_no_builtin_templates = 1

" tyru/open-browser.vim
nmap <silent> <Leader>o <Plug>(openbrowser-open)

" Shougo/unite.vim
let g:unite_enable_start_insert=1
nmap <silent> <C-n> :Unite -winheight=10 -direction=botright buffer<CR>
noremap <C-]> :Unite file_mru -winheight=10 -direction=botright<CR>
autocmd FileType unite call s:unite_my_settings()
function! s:unite_my_settings()
  imap <buffer> <C-k>   <Plug>(unite_select_previous_line)
  imap <buffer> <C-j>   <Plug>(unite_select_next_line)
  imap <buffer> <C-c>   <Plug>(unite_exit)

  imap <silent><buffer><expr> <C-x>     unite#do_action('split')
  imap <silent><buffer><expr> <C-v>     unite#do_action('vsplit')
endfunction

" Shougo/vimfiler.vim
let g:vimfiler_as_default_explorer = 1
let g:vimfiler_safe_mode_by_default = 0
let g:vimfiler_ignore_pattern = '\%(.pyc\)$'
noremap <silent> <Leader>f :VimFiler -split -explorer -winwidth=50<CR>

" ctrlpvim/ctrlp.vim
let g:ctrlp_map = '<c-p>'
let g:ctrlp_cmd = 'CtrlP'
let g:ctrlp_working_path_mode = 'ra'
let g:ctrlp_custom_ignore = '\v[\/]\.(git|hg|svn)$'
let g:ctrlp_user_command = ['.git', 'cd %s && git ls-files -co --exclude-standard']

" Shougo/neoyank.vim
nmap <silent> <Leader>y :Unite history/yank -direction=botright<CR>

" Shougo/neosnippet.vim
imap <C-k>     <Plug>(neosnippet_expand_or_jump)
smap <C-k>     <Plug>(neosnippet_expand_or_jump)
xmap <C-k>     <Plug>(neosnippet_expand_target)

imap <expr><TAB> neosnippet#expandable_or_jumpable() ?
\ "\<Plug>(neosnippet_expand_or_jump)"
\: pumvisible() ? "\<C-n>" : "\<TAB>"
smap <expr><TAB> neosnippet#expandable_or_jumpable() ?
\ "\<Plug>(neosnippet_expand_or_jump)"
\: "\<TAB>"
let g:neosnippet#disable_runtime_snippets = {
\   '_' : 1,
\ }
let g:neosnippet#enable_snipmate_compatibility = 1
let g:neosnippet#snippets_directory='~/.vim/bundle/neosnippet-snippets/neosnippets,~/.vim/after/snippets'


  " davidhalter/jedi.vim
  autocmd FileType python setl omnifunc=jedi#completions
  autocmd FileType python setl completeopt-=preview
  let g:jedi#popup_select_first = 0
  let g:jedi#completions_enabled = 1
  let g:jedi#auto_vim_configuration = 1
  let g:jedi#show_call_signatures = 0
  let g:jedi#rename_command = '<Leader>R'

if has('nvim')
  " Shougo/deoplete.nvim
  let g:deoplete#enable_at_startup = 1
  autocmd InsertLeave,CompleteDone * if pumvisible() == 0 | pclose | endif
else
  " Shougo/neocomplete.vim
  if has('lua')
    let g:acp_enableAtStartup = 0
    let g:neocomplete#enable_at_startup = 1

    let g:neocomplete#enable_smart_case = 1

    let g:neocomplete#sources#syntax#min_keyword_length = 3
    let g:neocomplete#lock_buffer_name_pattern = '\*ku\*'
    let g:neocomplete#enable_auto_select = 0
    let g:neocomplete#enable_auto_close_preview = 1
    let g:neocomplete#enable_ignore_case = 1

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
endif

" tpope/vim-fugitive
command Gd Git diff
command Gdca Git diff --cached

" lervag/vimtex
let g:vimtex_view_method = 'skim'
let g:vimtex_quickfix_ignore_filters = [
      \ 'Package caption Warning: Unknown document class (or package)',
      \ 'Package subfig Warning: Your document class has a bad definition',
      \]
