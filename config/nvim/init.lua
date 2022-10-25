-- options

vim.cmd [[
function! s:reload_config()
  luafile ~/.config/nvim/init.lua
  PackerInstall
  PackerClean
  PackerCompile
endfunction

command! Reload call s:reload_config()
]]

-- navigation
vim.cmd [[set virtualedit=all]]
vim.cmd [[set ignorecase]]
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
vim.cmd [[
  set expandtab
  set autoindent
]]

-- window
vim.cmd [[set splitbelow]]
vim.cmd [[set splitright]]


-- mappings

vim.cmd [[
  let mapleader="\<space>"
  let maplocalleader=","
  set tm=500
]]

vim.cmd [[
  nnoremap j gj
  nnoremap k gk
]]

vim.cmd [[
  set grepprg=rg\ --vimgrep\ --no-heading\ --smart-case
  nnoremap <Leader>r :silent grep<Space>
  nnoremap <silent> [c :cprevious<CR>
  nnoremap <silent> ]c :cnext<CR>
  au QuickfixCmdPost make,grep,grepadd,vimgrep copen
]]

vim.cmd [[
  nnoremap <localleader>d "_d
  vnoremap <localleader>d "_d
  nnoremap <localleader>x "_x
  nnoremap <localleader>p "_p
]]

vim.cmd [[
  nnoremap <localleader>w :w<CR>
]]

vim.cmd [[
  nnoremap gn :bn<CR>
  nnoremap gp :bp<CR>
  nnoremap gk :bp<bar>bd! #<CR>
  nnoremap gr gT
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
  imap <C-c> <Esc>
  nmap <C-c> <Esc>

  cnoremap <C-b> <Left>
  cnoremap <C-f> <Right>
  cnoremap <C-n> <Down>
  cnoremap <C-p> <Up>
  cnoremap <C-a> <Home>
  cnoremap <C-e> <End>
  cnoremap <C-d> <Del>
  cnoremap <C-x> <C-r>=expand('%:p')<CR>

  "tnoremap <C-Enter> <Enter>

  "tnoremap <M-n> <C-\><C-n>gt
  "inoremap <M-n> <Esc>gt
  "vnoremap <M-n> <Esc>gt
  "nnoremap <M-n> gt

  "tnoremap <M-p> <C-\><C-n>gT
  "inoremap <M-p> <Esc>gT
  "vnoremap <M-p> <Esc>gT
  "nnoremap <M-p> gT

  "tnoremap <M-h> <C-\><C-n><C-w>h
  "tnoremap <M-j> <C-\><C-n><C-w>j
  "tnoremap <M-k> <C-\><C-n><C-w>k
  "tnoremap <M-l> <C-\><C-n><C-w>l

  "inoremap <C-h> <Esc><C-w>h
  "inoremap <C-j> <Esc><C-w>j
  "inoremap <C-k> <Esc><C-w>k
  "inoremap <C-l> <Esc><C-w>l

  vnoremap <C-h> <Esc><C-w>h
  vnoremap <C-j> <Esc><C-w>j
  vnoremap <C-k> <Esc><C-w>k
  vnoremap <C-l> <Esc><C-w>l

  nnoremap <C-h> <C-w>h
  nnoremap <C-j> <C-w>j
  nnoremap <C-k> <C-w>k
  nnoremap <C-l> <C-w>l
]]

vim.cmd [[
  " ----------------------------------------------------------
  " Filetype
  " ----------------------------------------------------------
  autocmd FileType python set tabstop=4
  autocmd FileType python set shiftwidth=4
  autocmd FileType python set indentkeys-=:
  " autocmd FileType python inoremap <localleader>p from IPython.core.debugger import Pdb; ipdb = Pdb(); print("[ipdb] >>> "); ipdb.set_trace()<esc>
  " autocmd FileType python inoremap <localleader>i import IPython; print("[ipython] >>> "); IPython.embed()<esc>
  " autocmd FileType python nnoremap <localleader>f :w<cr> :!flake8 %<cr>

  if $USER == 'mujin'
    autocmd FileType python xnoremap <localleader>b :!blacken 110<CR>
  else
    autocmd FileType python noremap <localleader>b :!black %<CR>
  endif
]]


-- plugins

vim.cmd [[packadd packer.nvim]]

require("packer").startup(function()
  use {"wbthomason/packer.nvim"}

  use {"rhysd/committia.vim"}

  use {"hotwatermorning/auto-git-diff"}

  use {"tomtom/tcomment_vim"}

  use {
    "nvie/vim-flake8",
    config = function()
      vim.cmd [[
        autocmd FileType python nnoremap <silent> <localleader>f :call Flake8()<CR>
      ]]
    end,
  }

  use {
    "simeji/winresizer",
    config = function()
      vim.cmd [[ let g:winresizer_start_key = '<C-\>' ]]
    end,
  }

  use {
    "akinsho/bufferline.nvim",
    tag = "v2.*",
    config = function()
      require("bufferline").setup{
        highlights = {
          fill = {
            bg = 'none',
          }
        },
      }
    end
  }

  use {
    "folke/which-key.nvim",
    config = function()
      require("which-key").setup()
      require("which-key").register({
        z = { "<Cmd>lua telescope_find_dirs()<CR>", "Telescope find_dirs" },
      }, { prefix = "<leader>" })
    end,
  }

  use {
    "mattn/vim-molder",
    requires = {"mattn/vim-molder-operations"},
    config = function()
      vim.cmd [[
        function! MolderOpenDir() abort
          let l:path = b:molder_dir .. substitute(getline('.'), '/$', '', '')
          echomsg l:path
          if isdirectory(l:path)
            execute 'edit' fnameescape(l:path)
            execute 'lcd' fnameescape(l:path)
          endif
        endfunction

        function! MolderOpenFile() abort
          let l:path = b:molder_dir .. substitute(getline('.'), '/$', '', '')
          if !isdirectory(l:path)
            execute 'edit' fnameescape(l:path)
          endif
        endfunction

        nnoremap <silent> <plug>(molder-open-dir) :<c-u>call MolderOpenDir()<cr>
        nnoremap <silent> <plug>(molder-open-file) :<c-u>call MolderOpenFile()<cr>

        augroup vim-molder
          autocmd!
          autocmd FileType molder lcd %
          autocmd FileType molder setlocal nonumber
          autocmd FileType molder nmap <buffer> <Leader>c <Plug>(molder-operations-command)
          autocmd FileType molder nmap <buffer> h <Plug>(molder-up)
          autocmd FileType molder nmap <buffer> l <Plug>(molder-open-dir)
          autocmd FileType molder nmap <buffer> e <Plug>(molder-open-file)
          autocmd FileType molder nmap <buffer> . <Plug>(molder-toggle-hidden)
          "autocmd FileType molder nmap <buffer> <C-l> <Plug>(molder-reload)
        augroup end

        " disable netrw
        autocmd BufEnter * silent! autocmd! FileExplorer *
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
      if vim.fn.has("macunix") == 1 then
        vim.g.copilot_node_command = "~/.nodeenv/versions/16.5.0/bin/node"
      end
    end,
  }

  use {
    "akinsho/toggleterm.nvim",
    config = function()
      require("toggleterm").setup({
        shade_terminals = false,
        size = function(term)
          if term.direction == "horizontal" then
            return vim.o.lines * 0.5
          elseif term.direction == "vertical" then
            return vim.o.columns * 0.5
          end
        end,
        direction = 'vertical',
      })
      vim.cmd [[
        inoremap <silent> <M-e> <Esc>:ToggleTerm<CR>
        nnoremap <silent> <M-e> :ToggleTerm<CR>
        tnoremap <silent> <M-e> <C-\><C-n>:ToggleTerm<CR>
      ]]
    end
  }

  use {"romainl/vim-cool"}

  use {"tpope/vim-fugitive"}

  use {
    "wkentaro/nvim-editcommand",
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
      vim.cmd [[
        let g:rooter_cd_cmd = 'lcd'
        let g:rooter_change_directory_for_non_project_files = 'current'
        let g:rooter_targets = '*'
      ]]
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
      vim.cmd [[
        colorscheme catppuccin

        " Status Line Custom
        let g:currentmode={
            \ 'n'  : 'Normal',
            \ 'no' : 'Normal·Operator Pending',
            \ 'v'  : 'Visual',
            \ 'V'  : 'V·Line',
            \ '^V' : 'V·Block',
            \ 's'  : 'Select',
            \ 'S'  : 'S·Line',
            \ '^S' : 'S·Block',
            \ 'i'  : 'Insert',
            \ 'R'  : 'Replace',
            \ 'Rv' : 'V·Replace',
            \ 'c'  : 'Command',
            \ 'cv' : 'Vim Ex',
            \ 'ce' : 'Ex',
            \ 'r'  : 'Prompt',
            \ 'rm' : 'More',
            \ 'r?' : 'Confirm',
            \ '!'  : 'Shell',
            \ 't'  : 'Terminal'
            \}

        "au InsertEnter * hi statusline guifg=black guibg=#d7afff
        "au InsertLeave * hi statusline guifg=black guibg=#black

        hi VertSplit guifg=#45475a

        hi User1 guifg=#cdd6f4 guibg=#11111b  " black
        hi User2 guifg=#cdd6f4 guibg=#45475a  " gray

        set statusline=%1*\ %{toupper(mode())}\ %*%2*\ %{getcwd()}\ %*%1*\ %f\ [%P%M]
      ]]
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
            },
          },
        },
        pickers = {
          buffers = {
            mappings = {
              i = {
                ["<C-d>"] = actions.delete_buffer,
              },
            },
          },
        },
      })

      vim.cmd [[
        autocmd FileType TelescopePrompt call deoplete#custom#buffer_option('auto_complete', v:false)
        nnoremap <silent> <C-q> :Telescope quickfix<CR>
        nnoremap <silent> <C-p> :Telescope find_files<CR>
        nnoremap <silent> <C-n> :Telescope buffers<CR>
        nnoremap <silent> <C-s> :Telescope git_status<CR>
        nnoremap <silent> <leader>g :Telescope grep_string<CR>
        nnoremap <silent> <leader>f :Telescope live_grep<CR>
        nnoremap <silent> <leader>e :Telescope current_buffer_fuzzy_find<CR>
        nnoremap <silent> <leader>j :Telescope jumplist<CR>
        nnoremap <silent> <leader>c :Telescope neoclip<CR>
      ]]

      local actions = require("telescope.actions")
      local actions_set = require("telescope.actions.set")
      local actions_state = require("telescope.actions.state")
      local conf = require("telescope.config").values
      local finders = require("telescope.finders")
      local from_entry = require("telescope.from_entry")
      local pickers = require("telescope.pickers")

      -- mujin
      local fd_command
      local search_dir
      if vim.fn.has("macunix") == 1 then
        fd_command = "fd"
        search_dir = "~"
      else
        fd_command = "fdfind"
        search_dir = "~/workspaces"
      end

      function telescope_find_dirs(opts)
        pickers.new(opts, {
          prompt_title = "Change Directory",
          finder = finders.new_oneshot_job({
            fd_command,
            "--full-path",
            "/\\.git$",
            vim.fn.expand(search_dir),
            "--maxdepth=4",
            "--hidden",
            "--type=d",
            "--exec=dirname",
            "--exclude=\\.cache",
            "--exclude=\\.local",
          }),
          previewer = require("telescope.previewers").vim_buffer_cat.new({}),
          sorter = conf.generic_sorter(opts),
          attach_mappings = function(prompt_bufnr, map)
            actions_set.select:replace(function()
              local entry = actions_state.get_selected_entry()
              local dir = from_entry.path(entry)
              if entry ~= nil then
                actions.close(prompt_bufnr, false)
                vim.cmd("lcd " .. dir)
                vim.cmd("echon ''")
                print("cwd: " .. vim.fn.getcwd())
              end
            end)
            return true
          end,
        }):find()
      end
    end,
  }

  use {
    "AckslD/nvim-neoclip.lua",
    requires = {
      {"kkharji/sqlite.lua", module = "sqlite"},
      {"nvim-telescope/telescope.nvim"},
    },
    config = function()
      require("neoclip").setup({
        enable_persistent_history = true,
        default_register = "+",
        keys = {
          telescope = {
            i = {
              select = "<CR>",
              paste = {},
              paste_behind = {},
              replay = {},
              delete = "<C-d>",
              custom = {},
            },
            n = {
              select = "<CR>",
              paste = {},
              paste_behind = {},
              replay = {},
              delete = "d",
              custom = {},
            },
          },
        },
      })
      require("telescope").load_extension("neoclip")
    end,
  }

  -- use {
  --   "williamboman/mason.nvim",
  --   requires = {
  --     {"williamboman/mason-lspconfig.nvim"},
  --     {"neovim/nvim-lspconfig"},
  --   },
  --   config = function()
  --     require('mason').setup()
  --     require('mason-lspconfig').setup_handlers({ function(server)
  --       local opt = {
  --         capabilities = require('cmp_nvim_lsp').default_capabilities(
  --           vim.lsp.protocol.make_client_capabilities()
  --         )
  --       }
  --       if (server == "jedi_language_server") then
  --         opt.init_options = {
  --           diagnostics = {
  --             enable = false,
  --           },
  --         }
  --       end
  --       require('lspconfig')[server].setup(opt)
  --     end })
  --
  --     vim.keymap.set('n', 'K',  '<cmd>lua vim.lsp.buf.hover()<CR>')
  --     vim.keymap.set('n', '<leader>f', '<cmd>lua vim.lsp.buf.format()<CR>')
  --     vim.keymap.set('n', '<leader>x', '<cmd>lua vim.lsp.buf.references()<CR>')
  --     vim.keymap.set('n', '<leader>d', '<cmd>lua vim.lsp.buf.definition()<CR>')
  --     -- vim.keymap.set('n', 'gD', '<cmd>lua vim.lsp.buf.declaration()<CR>')
  --     -- vim.keymap.set('n', 'gi', '<cmd>lua vim.lsp.buf.implementation()<CR>')
  --     -- vim.keymap.set('n', '<leader>t', '<cmd>lua vim.lsp.buf.type_definition()<CR>')
  --     vim.keymap.set('n', '<leader>r', '<cmd>lua vim.lsp.buf.rename()<CR>')
  --     -- vim.keymap.set('n', 'ga', '<cmd>lua vim.lsp.buf.code_action()<CR>')
  --     -- vim.keymap.set('n', 'ge', '<cmd>lua vim.diagnostic.open_float()<CR>')
  --     -- vim.keymap.set('n', 'g]', '<cmd>lua vim.diagnostic.goto_next()<CR>')
  --     -- vim.keymap.set('n', 'g[', '<cmd>lua vim.diagnostic.goto_prev()<CR>')
  --   end,
  -- }
  --
  -- use {
  --   "jose-elias-alvarez/null-ls.nvim",
  --   requires = { "nvim-lua/plenary.nvim" },
  --   config = function()
  --     local mason = require("mason")
  --     local mason_package = require("mason-core.package")
  --     local mason_registry = require("mason-registry")
  --     local null_ls = require("null-ls")
  --
  --     mason.setup({})
  --
  --     local null_sources = {}
  --
  --     for _, package in ipairs(mason_registry.get_installed_packages()) do
  --       local package_category = package.spec.categories[1]
  --       if package_category == mason_package.Cat.Formatter then
  --         table.insert(null_sources, null_ls.builtins.formatting[package.name])
  --       end
  --       if package_category == mason_package.Cat.Linter then
  --         table.insert(null_sources, null_ls.builtins.diagnostics[package.name])
  --       end
  --     end
  --
  --     null_ls.setup({
  --       sources = null_sources,
  --     })
  --   end,
  -- }

  use {
    "hrsh7th/vim-vsnip",
    requires = {
      {"hrsh7th/cmp-vsnip"},
    },
    config = function()
      vim.cmd [[
        let g:vsnip_snippet_dir = expand('~/.config/nvim/vsnip')

        imap <expr> <C-k>   vsnip#expandable()  ? '<Plug>(vsnip-expand)'         : '<C-k>'
        smap <expr> <C-k>   vsnip#expandable()  ? '<Plug>(vsnip-expand)'         : '<C-k>'
      ]]
    end,
  }

  use {
    "hrsh7th/nvim-cmp",
    requires = {
      {"hrsh7th/cmp-nvim-lsp"},
    },
    config = function()
      local cmp = require("cmp")
      cmp.setup({
        snippet = {
          expand = function(args)
            vim.fn["vsnip#anonymous"](args.body)
          end,
        },
        mapping = cmp.mapping.preset.insert({
          ["<C-p>"] = cmp.mapping.select_prev_item(),
          ["<C-n>"] = cmp.mapping.select_next_item(),
          ['<C-l>'] = cmp.mapping.complete(),
          ['<C-e>'] = cmp.mapping.abort(),
          ["<CR>"] = cmp.mapping.confirm { select = true },
        }),
        sources = {
          { name = "nvim_lsp" },
          { name = "vsnip" },
        },
      })
      -- Use buffer source for `/` and `?`
      cmp.setup.cmdline({ '/', '?' }, {
        mapping = cmp.mapping.preset.cmdline(),
        sources = {
          { name = 'buffer' }
        }
      })

      -- Use cmdline & path source for ':'
      cmp.setup.cmdline(':', {
        mapping = cmp.mapping.preset.cmdline(),
        sources = cmp.config.sources({
          { name = 'path' }
        }, {
          { name = 'cmdline' }
        })
      })
    end,
  }

  use {
    "ycm-core/YouCompleteMe",
    run = "python3 install.py --all",
    config = function()
      vim.cmd [[
        nnoremap <silent> <expr> <S-k> &pvw == 1 ? ":pclose<CR>" : ":YcmCompleter GetDoc<CR> <C-w>j"
        nnoremap <silent> <leader>d :YcmCompleter GoToDefinition<CR>
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

-- legacy
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
set browsedir=buffer

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
