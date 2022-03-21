call plug#begin('~/.vim/plugged')
  Plug 'flazz/vim-colorschemes'

  Plug 'davidhalter/jedi-vim'

  if has('nvim')
    Plug 'Shougo/deoplete.nvim', { 'do': ':UpdateRemotePlugins' }

    Plug 'zchee/deoplete-jedi'
  else
    Plug 'Shougo/neocomplete.vim'
  endif

  Plug 'aperezdc/vim-template'

  Plug 'tomtom/tcomment_vim'

  Plug 'Shougo/unite.vim'
  Plug 'Shougo/neomru.vim'

  Plug 'Shougo/vimfiler.vim'
  Plug 'Shougo/vimproc.vim', {'do' : 'make'}

  Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }
  Plug 'junegunn/fzf.vim'

  Plug 'Shougo/neosnippet.vim'

  Plug 'rhysd/committia.vim'

  Plug 'hotwatermorning/auto-git-diff'

  Plug 'tpope/vim-fugitive'

  Plug 'lervag/vimtex'

  Plug 'preservim/tagbar'

  Plug 'RRethy/vim-illuminate'
call plug#end()


" --------------------------------------------------------------------
" flazz/vim-colorschemes
" --------------------------------------------------------------------
syntax on
let g:solarized_termtrans = 1
set background=dark
colorscheme solarized
hi Normal ctermfg=none
hi TabLineSel ctermfg=LightBlue ctermbg=Black


" --------------------------------------------------------------------
" aperezdc/vim-template
" --------------------------------------------------------------------
let g:templates_directory = ['~/.vim/after/templates']
let g:templates_no_builtin_templates = 1


" --------------------------------------------------------------------
" Shougo/vimfiler.vim
" --------------------------------------------------------------------
let g:vimfiler_as_default_explorer = 1
let g:vimfiler_safe_mode_by_default = 0
noremap <silent> <leader>f :VimFilerBuffer -buffer-name=explorer -split -simple -winwidth=35 -toggle -no-quit<cr>

autocmd FileType vimfiler call s:vimfiler_settings()
function! s:vimfiler_settings() abort
  silent! nunmap <buffer> <C-l>
endfunction


function! s:find_git_root()
    return system('git rev-parse --show-toplevel 2> /dev/null')[:-2]
endfunction
command! ProjectFiles execute 'Files' s:find_git_root()
" nnoremap <C-p> :ProjectFiles<CR>
" nnoremap <C-n> :Buffer<CR>

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
if has("mac")
  let g:vimtex_view_method = 'skim'
elseif has("unix")
  let g:vimtex_view_method = 'general'
  let g:vimtex_view_general_viewer = 'okular'
  let g:vimtex_view_general_options = '--unique file:@pdf\#src:@line@tex'
  let g:vimtex_view_general_options_latexmk = '--unique'
endif
let g:vimtex_quickfix_ignore_filters = [
      \ 'Package caption Warning: Unknown document class (or package)',
      \ 'Package subfig Warning: Your document class has a bad definition',
      \ 'Package hyperref Warning: Token not allowed in a PDF string (Unicode)',
      \ 'Overfull ',
      \ 'Underfull ',
      \ 'Package minitoc(hints) Warning: ',
      \ 'LaTeX Warning: Citation ',
      \ 'LaTeX Warning: No positions in optional float specifier.',
      \ 'LaTeX Warning: There were undefined references.',
      \ 'LaTeX Font Warning:',
      \]
let g:tex_flavor = 'latex'

" preservim/tagbar
let g:tagbar_sort = 0
noremap <silent> <Leader>t :TagbarToggle<CR>

autocmd BufEnter * if winnr('$') == 1 && exists('b:vimfiler') && b:vimfiler['context']['explorer'] | quit | endif


" Prevent FZF commands from opening in none modifiable buffers
function! FZFOpen(cmd)
    " If more than 1 window, and buffer is not modifiable or file type is
    " NERD tree or Quickfix type
    if winnr('$') > 1 && (!&modifiable || &ft == 'nerdtree' || &ft == 'qf')
        " Move one window to the right, then up
        wincmd l
        wincmd k
    endif
    exe a:cmd
endfunction

" FZF in Open buffers
" nnoremap <silent> <C-n> :call FZFOpen(":Buffers")<CR>
nnoremap <silent> <C-n> :call FZFOpen(":Rg")<CR>

" FZF Search for Files
nnoremap <silent> <C-p> :call FZFOpen(":ProjectFiles")<CR>

nnoremap <silent> <C-]> :call FZFOpen(":Files ~")<CR>

let g:fzf_buffers_jump = 1
let g:fzf_action = {
    \ 'ctrl-o': 'tab split',
    \ 'ctrl-x': 'split',
    \ 'ctrl-v': 'vsplit'}

command! -bang -nargs=* Rg call fzf#vim#grep(
    \ 'rg --column --line-number --no-heading --color=always '.shellescape(<q-args>),
    \ 1,
    \ fzf#vim#with_preview({'dir': system('git rev-parse --show-toplevel 2> /dev/null')[:-2], 'options': '--delimiter : --nth 4..'}),
    \ <bang>0)
