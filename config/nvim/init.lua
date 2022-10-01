-- options

-- navigation
vim.cmd [[set virtualedit=all]]
vim.cmd [[set incsearch]]
vim.cmd [[set nowrapscan]]

-- yank
vim.cmd [[set clipboard=unnamedplus]]

-- appearence
vim.cmd [[set number]]
vim.cmd [[set list]]
vim.cmd [[set listchars=tab:»-,trail:-,extends:»,precedes:«,nbsp:%,eol:↲]]

-- edit
vim.cmd [[set noswapfile]]
vim.cmd [[set shiftwidth=2]]
vim.cmd [[set expandtab]]

-- window
vim.cmd [[set splitbelow]]
vim.cmd [[set splitright]]


-- mappings

vim.cmd [[
  let mapleader="\<space>"
  let maplocalleader=","
]]

vim.cmd [[
  nnoremap <localleader>d "_d
  vnoremap <localleader>d "_d
  nnoremap <localleader>x "_x
]]

vim.cmd [[
  nnoremap gn :bn<CR>
  nnoremap gp :bp<CR>
  nnoremap gk :bp<bar>bd #<CR>
]]

vim.cmd [[
  au BufReadPost * if line("'\"") > 0 && line("'\"") <= line("$")
    \| exe "normal! g'\"" | endif
]]

vim.cmd [[
  tnoremap <Esc> <C-\><C-n>
  tnoremap <C-[> <C-\><C-n>

  function! IsOnThePrompt(prompt)
    let s:space_or_eol = '\( \|$\|\n\)'
    let l:line_number_end = line('$')
    let l:line_number_current = line('.')

    if abs(l:line_number_end - l:line_number_current) < 3
      return 1
    endif

    let l:found_prompt = 0
    let l:line_number_prompt = l:line_number_end
    while l:line_number_prompt > 0
      " found prompt
      if match(getline(l:line_number_prompt), a:prompt . s:space_or_eol) !=# -1
        let l:found_prompt = 1
        break
      endif
      " too long terminal
      if l:line_number_end - l:line_number_prompt > 10000
        break
      endif
      let l:line_number_prompt = l:line_number_prompt - 1
    endwhile

    if l:found_prompt == 0
      return 1  " in less
    endif

    return l:found_prompt && (l:line_number_current - l:line_number_prompt > -3)
  endfunction

  augroup terminal_settings
    autocmd TermOpen * startinsert
    autocmd TermOpen * setlocal nonumber norelativenumber
    autocmd BufWinEnter,WinEnter term://* if IsOnThePrompt('%') | startinsert | endif
    autocmd TermClose term://* call nvim_input('<CR>')
  augroup end
]]

vim.cmd [[
  tnoremap <C-Enter> <Enter>

  tnoremap <M-n> <C-\><C-n>gt
  inoremap <M-n> <Esc>gt
  vnoremap <M-n> <Esc>gt
  nnoremap <M-n> gt

  tnoremap <M-p> <C-\><C-n>gT
  inoremap <M-p> <Esc>gT
  vnoremap <M-p> <Esc>gT
  nnoremap <M-p> gT

  tnoremap <M-h> <C-\><C-n><C-w>h
  tnoremap <M-j> <C-\><C-n><C-w>j
  tnoremap <M-k> <C-\><C-n><C-w>k
  tnoremap <M-l> <C-\><C-n><C-w>l

  inoremap <M-h> <Esc><C-w>h
  inoremap <M-j> <Esc><C-w>j
  inoremap <M-k> <Esc><C-w>k
  inoremap <M-l> <Esc><C-w>l

  vnoremap <M-h> <Esc><C-w>h
  vnoremap <M-j> <Esc><C-w>j
  vnoremap <M-k> <Esc><C-w>k
  vnoremap <M-l> <Esc><C-w>l

  nnoremap <M-h> <C-w>h
  nnoremap <M-j> <C-w>j
  nnoremap <M-k> <C-w>k
  nnoremap <M-l> <C-w>l
]]


-- plugins

vim.cmd [[packadd packer.nvim]]

require("packer").startup(function()
  use {"wbthomason/packer.nvim"}

  use {"rhysd/committia.vim"}

  use {"hotwatermorning/auto-git-diff"}

  use {"tomtom/tcomment_vim"}

  use {
    "Shougo/defx.nvim",
    requires = {"kristijanhusak/defx-git"},
    config = function()
      vim.cmd [[
        call defx#custom#column('git', 'column_length', 2)
        call defx#custom#option('_', {
          \ 'columns': 'mark:indent:git:filename',
          \ })
        function! s:defx_my_settings() abort
          setlocal nonumber
          nnoremap <silent><buffer><expr> l defx#do_action('open_directory')
          nnoremap <silent><buffer><expr> h defx#do_action('cd', ['..'])
          nnoremap <silent><buffer><expr> q defx#do_action('quit')
          nnoremap <silent><buffer><expr> e defx#is_directory() ? '' : defx#do_action('open')
          nnoremap <silent><buffer><expr> . defx#do_action('toggle_ignored_files')
          nnoremap <silent><buffer><expr> cd defx#do_action('change_vim_cwd')
          nnoremap <silent><buffer><expr> o defx#do_action('toggle_select') . 'j'
          vnoremap <silent><buffer><expr> o defx#do_action('toggle_select_visual') . 'j'
          nnoremap <silent><buffer><expr> <c-l> defx#do_action('redraw')
          nnoremap <silent><buffer><expr> <s-k> defx#do_action('')
          nnoremap <silent><buffer><expr> * defx#do_action('toggle_select_all')
          nnoremap <silent><buffer><expr> ! defx#do_action('execute_command')
          nnoremap <silent><buffer><expr> r defx#do_action('rename')
          nnoremap <silent><buffer><expr> d defx#do_action('remove')
          nnoremap <silent><buffer><expr> y defx#do_action('yank_path')
        endfunction
        autocmd FileType defx call s:defx_my_settings()
      ]]
    end,
  }

  use {
    "RRethy/vim-illuminate",
    config = function()
      vim.cmd [[
        autocmd TermOpen * setlocal ft=terminal
        let g:Illuminate_ftblacklist = ["terminal"]
      ]]
    end,
  }

  use {
    "github/copilot.vim",
    config = function()
      vim.cmd [[
        let g:copilot_filetypes = {
          \ 'TelescopePrompt': v:false,
          \ }
      ]]
    end,
  }

  use {"romainl/vim-cool"}

  use {"tpope/vim-fugitive"}

  use {
    "brettanomyces/nvim-editcommand",
    config = function()
      vim.cmd [[let g:editcommand_prompt = '%']]
    end,
  }

  use {
    "preservim/tagbar",
    config = function()
      vim.cmd [[
        let g:tagbar_sort = 0
        nnoremap <localleader>t :Tagbar<CR>
      ]]
    end,
  }

  use {
    "airblade/vim-rooter",
    config = function()
      vim.cmd [[let g:rooter_cd_cmd = 'lcd']]
    end,
  }

  use {
    "catppuccin/nvim",
    as = "catppuccin",
    setup = function()
      vim.cmd [[let g:catppuccin_flavour = "mocha"]]
    end,
    config = function()
      require("catppuccin").setup()
      vim.cmd [[colorscheme catppuccin]]
    end,
  }

  use {
    "nvim-telescope/telescope.nvim",
    requires = {"nvim-lua/plenary.nvim"},
    config = function()
      local actions = require("telescope.actions")
      require("telescope").setup({
        defaults = {
          mappings = {
            i = {
              ["<C-d>"] = false,  -- -> delete_buffer
              ["<C-j>"] = actions.move_selection_next,
              ["<C-k>"] = actions.move_selection_previous,
              ["<C-u>"] = false,  -- -> clear the search field
              ["<C-f>"] = actions.cycle_history_next,
              ["<C-b>"] = actions.cycle_history_prev,
              ["<C-n>"] = actions.preview_scrolling_down,
              ["<C-p>"] = actions.preview_scrolling_up,
              ["<C-q>"] = actions.close,
            },
          },
        },
        pickers = {
          buffers = {
            mappings = {
              i = {
                ["<c-d>"] = actions.delete_buffer,
              },
            },
          },
        },
      })

      vim.cmd [[
        autocmd FileType TelescopePrompt call deoplete#custom#buffer_option('auto_complete', v:false)
        nnoremap <C-p> :Telescope find_files<CR>
        nnoremap <C-n> :Telescope buffers<CR>
        nnoremap <C-s> :Telescope git_status<CR>
      ]]
    end,
  }

  use {
    "ycm-core/YouCompleteMe",
    run = "python3 install.py --all",
    config = function()
      vim.cmd [[
        nnoremap <expr> <S-k> &pvw == 1 ? ":pclose<CR>" : ":YcmCompleter GetDoc<CR> <C-w>j"
        nnoremap <leader>d :YcmCompleter GoToDefinition<CR>
      ]]
    end
  }

  -- use {
  --   "lervag/vimtex",
  --   config = function()
  --     vim.cmd [[
  --       if has("mac")
  --         let g:vimtex_view_method = 'skim'
  --       elseif has("unix")
  --         let g:vimtex_view_method = 'general'
  --         let g:vimtex_view_general_viewer = 'okular'
  --         let g:vimtex_view_general_options = '--unique file:@pdf\#src:@line@tex'
  --         let g:vimtex_view_general_options_latexmk = '--unique'
  --       endif
  --       let g:tex_flavor = 'latex'
  --     ]]
  --   end,
  -- }
end)
