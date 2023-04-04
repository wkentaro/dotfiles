vim.cmd [[packadd packer.nvim]]

require("packer").startup(function()
  use {"wbthomason/packer.nvim"}

  use {"rhysd/committia.vim"}

  use {"hotwatermorning/auto-git-diff"}

  use {"tomtom/tcomment_vim"}

  use {"lambdalisue/suda.vim"}

  use {"vmchale/just-vim"}

  use {
    "vim-syntastic/syntastic",
    config = function()
      vim.cmd [[
        set statusline+=%#warningmsg#
        set statusline+=%{SyntasticStatuslineFlag()}
        set statusline+=%*
        let g:syntastic_always_populate_loc_list = 1
        let g:syntastic_auto_loc_list = 1
        let g:syntastic_check_on_open = 0
        let g:syntastic_check_on_wq = 0
        let g:syntastic_mode_map = {
            \ "mode": "passive",
            \ "active_filetypes": [],
            \ "passive_filetypes": [] }
        let g:syntastic_python_checkers = ['flake8']
        nnoremap <silent> <localleader>f :SyntasticCheck<CR>
        nnoremap <silent> <localleader>ff :SyntasticReset<CR>
        let g:syntastic_loc_list_height = 5

        cabbrev <silent> bd <C-r>=(getcmdtype()==#':' && getcmdpos()==1 ? 'lclose\|bdelete' : 'bd')<CR>

        nnoremap <silent> [l :lprevious<CR>
        nnoremap <silent> ]l :lnext<CR>
      ]]
    end,
  }

  use {
    "mbbill/undotree",
    config = function()
      vim.cmd [[
        nnoremap <silent> <localleader>u :UndotreeToggle<CR>
      ]]
    end,
  }

  -- use {
  --   "nvie/vim-flake8",
  --   config = function()
  --     vim.cmd [[
  --       autocmd FileType python nnoremap <silent> <localleader>f :call Flake8()<CR>
  --     ]]
  --   end,
  -- }

  use {
    "simeji/winresizer",
    config = function()
      vim.cmd [[ let g:winresizer_start_key = '<C-\>' ]]
    end,
  }

  -- use {
  --   "akinsho/bufferline.nvim",
  --   tag = "v2.*",
  --   config = function()
  --     require("bufferline").setup{
  --       highlights = {
  --         fill = {
  --           bg = 'none',
  --         }
  --       },
  --     }
  --   end
  -- }

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
    "qpkorr/vim-renamer",
    config = function()
      vim.cmd [[
        function! MyRenamer() abort
          execute 'enew'
          setlocal nonumber
          execute 'Renamer'
        endfunction

        let g:RenamerSupportColonWToRename = 1
      ]]
    end
  }

  use {
    "mattn/vim-molder",
    config = function()
      vim.cmd [[
        function! MolderOpenDir() abort
          let l:path = b:molder_dir .. substitute(getline('.'), '/$', '', '')
          echomsg l:path
          if isdirectory(l:path)
            execute 'edit' fnameescape(l:path)
            " execute 'lcd' fnameescape(l:path)
            execute 'cd' fnameescape(l:path)
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

        fun! Confirm(msg)
            echo a:msg . ' '
            let l:answer = nr2char(getchar())

            if l:answer ==? 'y'
              return 1
            elseif l:answer ==? 'n'
              return 0
            else
              return 0
            endif
        endfun

        function! MolderDelete() abort
          let l:path = getline('.')
          if Confirm('Delete?: ' . l:path . ' (Y)es/[N]o')
            silent execute '!rm -r' l:path
            echo 'Deleted: ' . l:path
          else
            echo 'Aborted'
          endif
          call molder#reload()
        endfunction

        function! MolderDeleteSelected() range
          let l:paths = getline(a:firstline, a:lastline)
          if Confirm('Delete?: ' . join(l:paths, ' ') . ' (Y)es/[N]o')
            silent execute '!rm -r' join(l:paths, ' ')
            echo 'Deleted: ' . join(l:paths, ' ')
          else
            echo 'Aborted'
          endif
          call molder#reload()
        endfunction

        augroup vim-molder
          autocmd!
          autocmd FileType molder cd %
          autocmd FileType molder setlocal nonumber
          autocmd FileType molder nmap <buffer> <Leader>c <Plug>(molder-operations-command)
          autocmd FileType molder nmap <buffer> h <Plug>(molder-up)
          autocmd FileType molder nmap <buffer> l <Plug>(molder-open-dir)
          autocmd FileType molder nmap <buffer> e <Plug>(molder-open-file)
          autocmd FileType molder nmap <buffer> . <Plug>(molder-toggle-hidden)
          "autocmd FileType molder nmap <buffer> <C-l> <Plug>(molder-reload)
          autocmd FileType molder nmap <buffer> <Leader>r :<c-u>call MyRenamer()<CR>
          autocmd FileType molder nmap <buffer> <Leader>d :call MolderDelete()<CR>
          autocmd FileType molder xmap <buffer> <Leader>d :call MolderDeleteSelected()<CR>
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
    end,
  }

  -- use {
  --   "zbirenbaum/copilot.lua",
  --   cmd = "Copilot",
  --   event = "InsertEnter",
  --   config = function()
  --     require("copilot").setup({
  --       suggestion = { auto_trigger = true },
  --     })
  --   end,
  -- }

  -- use {
  --   "akinsho/toggleterm.nvim",
  --   config = function()
  --     require("toggleterm").setup({
  --       shade_terminals = false,
  --       size = function(term)
  --         if term.direction == "horizontal" then
  --           return vim.o.lines * 0.5
  --         elseif term.direction == "vertical" then
  --           return vim.o.columns * 0.5
  --         end
  --       end,
  --       direction = 'vertical',
  --     })
  --     vim.cmd [[
  --       inoremap <silent> <M-e> <Esc>:ToggleTerm<CR>
  --       nnoremap <silent> <M-e> :ToggleTerm<CR>
  --       tnoremap <silent> <M-e> <C-\><C-n>:ToggleTerm<CR>
  --     ]]
  --   end
  -- }

  use {"romainl/vim-cool"}

  use {"tpope/vim-fugitive",
    config = function()
      vim.cmd [[
        autocmd FileType git nmap <buffer> q :q<CR>
        autocmd FileType git set nonumber
        autocmd FileType fugitive set nonumber
        " nnoremap <silent> <localleader>gs :vsp \| :terminal tig status<CR>
        nnoremap <silent> <localleader>gs :vertical Git<CR>
        nnoremap <silent> <localleader>gd :vertical G diff %<CR>
        nnoremap <silent> <localleader>gl :vsp \| :terminal tig -c %<CR>
        nnoremap <silent> <expr> <localleader>gb ':terminal tig blame -c % +' . line(".") . '<CR>'
        nnoremap <silent> <localleader>gd :vertical G diff %<CR>
      ]]
    end,
  }

  use {
    "aperezdc/vim-template",
    config = function()
      vim.cmd [[
        let g:templates_directory = '~/.config/nvim/templates'
        let g:templates_no_builtin_templates = 1
      ]]
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

  -- use {
  --   "airblade/vim-rooter",
  --   config = function()
  --     vim.cmd [[
  --       let g:rooter_cd_cmd = 'lcd'
  --       let g:rooter_change_directory_for_non_project_files = 'current'
  --       let g:rooter_targets = '*'
  --     ]]
  --   end,
  -- }

  use {
    "catppuccin/nvim",
    as = "catppuccin",
    config = function()
      require("catppuccin").setup({
        flavour = "mocha",
        styles = {
          comments = {},
          conditionals = {},
        }
      })
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

  use {'kshenoy/vim-signature'}

  use({
    "kwkarlwang/bufjump.nvim",
    config = function()
      require("bufjump").setup({
        forward = "<M-i>",
        backward = "<M-o>",
        on_success = function()
          vim.cmd([[execute "normal! g`\"zz"]])
        end,
      })
    end,
  })

  use {
    "nvim-telescope/telescope.nvim",
    requires = {
      {"nvim-lua/plenary.nvim"},
      {"nvim-telescope/telescope-smart-history.nvim"},
    },
    config = function()
      local actions = require("telescope.actions")
      require("telescope").setup({
        defaults = {
          history = {
            path = '~/.local/share/nvim/databases/telescope_history.sqlite3',
            limit = 100,
          },
          mappings = {
            i = {
              -- ["<C-d>"] = false,  -- -> delete_buffer
              ["<C-j>"] = actions.move_selection_next,
              ["<C-k>"] = actions.move_selection_previous,
              ["<C-u>"] = false,  -- -> clear the search field
              ["<C-p>"] = actions.cycle_history_prev,
              ["<C-n>"] = actions.cycle_history_next,
              ["<C-e>"] = actions.preview_scrolling_down,
              ["<C-y>"] = actions.preview_scrolling_up,
            },
          },
        },
        pickers = {
          buffers = {
            mappings = {
              i = {
                -- ["<C-d>"] = actions.delete_buffer,
              },
            },
          },
        },
      })

      require('telescope').load_extension('smart_history')

      vim.cmd [[
        autocmd FileType TelescopePrompt call deoplete#custom#buffer_option('auto_complete', v:false)
        nnoremap <silent> <C-q> :Telescope quickfix<CR>
        nnoremap <silent> <C-p> :Telescope find_files<CR>
        nnoremap <silent> <C-n> :Telescope buffers<CR>
        nnoremap <silent> <C-s> :Telescope git_status<CR>
        nnoremap <silent> <leader>r :Telescope grep_string<CR>
        nnoremap <silent> <leader>l :Telescope lsp_references<CR>
        nnoremap <silent> <leader>f :Telescope live_grep<CR>
        "nnoremap <silent> <leader>e :Telescope current_buffer_fuzzy_find<CR>
        "nnoremap <silent> <leader>j :Telescope jumplist<CR>
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

  use {
    "hrsh7th/nvim-cmp",
    requires = {
      {"hrsh7th/cmp-buffer"},
      {"hrsh7th/cmp-path"},
      {"hrsh7th/cmp-cmdline"},
      {"hrsh7th/vim-vsnip"},
      {"hrsh7th/cmp-vsnip"},
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
          ["<C-j>"] = cmp.mapping.confirm { select = true },
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

      vim.cmd [[
        let g:vsnip_snippet_dir = expand('~/.config/nvim/vsnip')
        imap <expr> <C-j> vsnip#expandable() ? '<Plug>(vsnip-expand)' : '<C-j>'
        smap <expr> <C-j> vsnip#expandable() ? '<Plug>(vsnip-expand)' : '<C-j>'
      ]]
    end,
  }

  use {
    "williamboman/mason.nvim",
    requires = {
      {"williamboman/mason-lspconfig.nvim"},
      {"neovim/nvim-lspconfig"},
      {"hrsh7th/cmp-nvim-lsp"},
      -- {"ray-x/lsp_signature.nvim"},
      {'nvim-treesitter/nvim-treesitter', run=':TSUpdate'},
      -- {'rmagatti/goto-preview'},
    },
    config = function()
      require('mason').setup({
        ensure_installed = {
          "jedi-language-server",
          "rust-analyzer",
        }
      })
      require('mason-lspconfig').setup_handlers({ function(server)
        local opt = {
          capabilities = require('cmp_nvim_lsp').default_capabilities(
            vim.lsp.protocol.make_client_capabilities()
          )
        }
        if (server == "jedi_language_server") then
          opt.init_options = {
            diagnostics = {
              enable = false,
            },
          }
        end
        if (server == "pyright") then
          opt.settings = {
            python = {
              analysis = {
                typeCheckingMode = "off",
              },
            },
          }
        end
        require('lspconfig')[server].setup(opt)
      end })

      -- require('lsp_signature').setup()

      vim.keymap.set('n', 'K',  '<cmd>lua vim.lsp.buf.hover()<CR>')
      -- vim.keymap.set('n', '<leader>f', '<cmd>lua vim.lsp.buf.format()<CR>')
      vim.keymap.set('n', 'gl', '<cmd>lua vim.lsp.buf.references()<CR>')
      vim.keymap.set('n', 'gd', '<cmd>lua vim.lsp.buf.definition()<CR>')
      vim.keymap.set('n', '<C-k>', '<cmd>lua vim.lsp.buf.signature_help()<CR>')
      vim.keymap.set('i', '<C-k>', '<cmd>lua vim.lsp.buf.signature_help()<CR>')
      vim.keymap.set('n', '<leader>d', '<cmd>lua vim.lsp.buf.definition()<CR>')
      -- vim.keymap.set('n', 'gD', '<cmd>lua vim.lsp.buf.declaration()<CR>')
      -- vim.keymap.set('n', 'gi', '<cmd>lua vim.lsp.buf.implementation()<CR>')
      -- vim.keymap.set('n', '<leader>t', '<cmd>lua vim.lsp.buf.type_definition()<CR>')
      -- vim.keymap.set('n', 'rn', '<cmd>lua vim.lsp.buf.rename()<CR>')
      -- vim.keymap.set('n', 'ga', '<cmd>lua vim.lsp.buf.code_action()<CR>')
      -- vim.keymap.set('n', 'ge', '<cmd>lua vim.diagnostic.open_float()<CR>')
      -- vim.keymap.set('n', 'g]', '<cmd>lua vim.diagnostic.goto_next()<CR>')
      -- vim.keymap.set('n', 'g[', '<cmd>lua vim.diagnostic.goto_prev()<CR>')

      require('nvim-treesitter.configs').setup {
        ensure_installed = { "python", "comment" },
      }

      -- require('goto-preview').setup()
      -- vim.keymap.set('n', '<leader>d', require('goto-preview').goto_preview_definition)

      -- vim.cmd [[
      --   function! IsFloating(id) abort
      --       let l:cfg = nvim_win_get_config(a:id)
      --       return !empty(l:cfg.relative) || l:cfg.external
      --   endfunction
      --   nnoremap <expr> q IsFloating(win_getid()) ? ":q<CR>" : ""
      -- ]]
    end,
  }

  use {
    "jose-elias-alvarez/null-ls.nvim",
    requires = { "nvim-lua/plenary.nvim" },
    config = function()
      local mason = require("mason")
      local mason_package = require("mason-core.package")
      local mason_registry = require("mason-registry")
      local null_ls = require("null-ls")

      mason.setup({})

      local null_sources = {}

      for _, package in ipairs(mason_registry.get_installed_packages()) do
        local package_category = package.spec.categories[1]
        if package_category == mason_package.Cat.Formatter then
          table.insert(null_sources, null_ls.builtins.formatting[package.name])
        end
        if package_category == mason_package.Cat.Linter then
          table.insert(null_sources, null_ls.builtins.diagnostics[package.name])
        end
      end

      null_ls.setup({
        sources = null_sources,
      })
    end,
  }

  -- use {
  --   "ycm-core/YouCompleteMe",
  --   run = "python3 install.py --all",
  --   config = function()
  --     vim.cmd [[
  --       nnoremap <silent> <expr> <S-k> &pvw == 1 ? ":pclose<CR>" : ":YcmCompleter GetDoc<CR> <C-w>j"
  --       nnoremap <silent> <leader>d :YcmCompleter GoToDefinition<CR>
  --     ]]
  --   end
  -- }

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

  use {
    'lewis6991/gitsigns.nvim',
    config = function()
      require('gitsigns').setup()
    end
  }
end)
