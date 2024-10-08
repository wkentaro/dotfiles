# vim: set ft=zsh:
#
# ---------------------------------
# zsh options
# ---------------------------------

setopt auto_param_slash
setopt mark_dirs
setopt list_types
setopt auto_menu
setopt auto_param_keys
setopt interactive_comments
setopt magic_equal_subst
setopt complete_in_word
setopt always_last_prompt
setopt print_eight_bit
setopt no_hup
# for git reset HEAD^
unsetopt extended_glob
# for ~input:=/camera/rgb/image_raw
unsetopt magicequalsubst
# for scp server:*.zip .
unsetopt nomatch
# setopt globdots  # enable completion for dotfiles
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'
zstyle ':completion:*' accept-exact '*(N)'
zstyle ':completion:*' use-cache on
zstyle ':completion:*' cache-path ~/.zsh/cache
zstyle ':completion:*:default' menu select=1
zstyle ':completion:*:manuals' separate-sections true
zstyle ':completion:*:*:-subscript-:*' tag-order indexes parameters
zstyle ':completion:*:*files' ignored-patterns '*?.o' '*?~' '*\#'

# add color
autoload colors
colors
zstyle ':completion:*' list-colors ${(s.:.)LS_COLORS}

# history option
setopt hist_ignore_dups
setopt hist_ignore_space
setopt histignorealldups
autoload history-search-end

setopt list_packed
setopt pushd_ignore_dups

# no beep
setopt nolistbeep
setopt nobeep

# auto cd
setopt auto_cd
zstyle ':completion:*:cd:*' tag-order local-directories path-directories
zstyle ':completion:*:cd:*' ignore-parents parent pwd

# ccache
export PATH="/usr/lib/ccache:$PATH"

# prefix: /usr/local
# export PATH="/usr/lib/ccache:/usr/local/bin:/usr/local/sbin:$PATH"
# export MANPATH="/usr/local/man:$MANPATH"

# prefix: $HOME/local
export PATH="$HOME/.local/bin:$PATH"
export MANPATH="$HOME/.local/bin:$MANPATH"
# export PYTHONPATH="$HOME/.local/lib/python2.7/site-packages:$PYTHONPATH"

if [ "$(uname)" = "Darwin" ]; then
    export PATH="$HOME/.go/bin:$PATH"
#   export PATH="/usr/local/opt/python@2/libexec/bin:$PATH"
    export PATH="/usr/local/opt/python/libexec/bin:$PATH"
    export PATH="/usr/local/opt/ruby/bin:$PATH"
fi

# bookmark
# hash -d dotfiles=$HOME/.dotfiles

# Python
export VIRTUALENV_USE_DISTRIBUTE=1

# Termcolor
export TERM=xterm-256color

# Encoding
export LANG='en_US.UTF-8'
export LC_CTYPE='en_US.UTF-8'
# export LC_ALL='C'

# Editor
if which nvim &>/dev/null; then
  export EDITOR='nvim'
  alias vim=nvim
  alias vi=nvim
else
  export EDITOR='vim'
fi

# ssh
export SSH_KEY_PATH='$HOME/.ssh/id_rsa'

# GitHub
if [ -z $GITHUB_USER ]; then
  export GITHUB_USER='wkentaro'
fi

# Homebrew
export HOMEBREW_NO_AUTO_UPDATE=1

# ---------------------------------
# zsh plugins
# ---------------------------------

if [ -e ~/.zplug/init.zsh ]; then
  source ~/.zplug/init.zsh

  zplug "zpm-zsh/autoenv"
  zplug "rupa/z", use:z.sh
  zplug "zsh-users/zsh-completions"
  zplug "zsh-users/zsh-syntax-highlighting"
  zplug "wkentaro/wkentaro.zsh-theme", as:theme

  # Install plugins if there are plugins that have not been installed
  if ! zplug check --verbose; then
      printf "Install? [y/N]: "
      if read -q; then
          echo; zplug install
      fi
  fi
  # Then, source plugins and add commands to $PATH
  zplug load #--verbose
else
  echo "Please install zplug: https://github.com/zplug/zplug#installation"
fi

# autoenv
export AUTOENV_CHECK_AUTH=0

fpath=($HOME/.zsh/completions $fpath)

# ohmyzsh
DISABLE_AUTO_UPDATE=true
DISABLE_MAGIC_FUNCTIONS=true
plugins=(git python web-search vi-mode)
ZSH=$HOME/.zsh/ohmyzsh
source $ZSH/oh-my-zsh.sh

# local plugins
plugins=(
  $HOME/.zsh/plugins/git.sh
  # $HOME/.zsh/plugins/ros.sh
  $HOME/.zsh/plugins/ubuntu.sh
  # $HOME/.zsh/plugins/aws.sh
)
for plugin in $plugins; do
  source $plugin
done

autoload -U compinit && compinit

# https://github.com/wkentaro/pycd
type pycd.sh &>/dev/null && source `which pycd.sh`

# ----------------------------
# Improved less option
# ----------------------------

export LESS='--tabs=4 --LONG-PROMPT --ignore-case --RAW-CONTROL-CHARS'

# --------------------------------
# bindkey
# --------------------------------

# peco history search
# Ctrl-R
function peco_history () {
  if [ "$(uname)" = "Linux" ]; then
    BUFFER=$(history | tac | peco --query "$LBUFFER" | awk '{print substr($0, index($0, $4))}')
  elif [ "$(uname)" = "Darwin" ]; then
    BUFFER=$(history | tail -r | peco --query "$LBUFFER" | awk '{print substr($0, index($0, $4))}')
  fi
  CURSOR=$#BUFFER         # move cursor
  zle -R -c               # refresh
}
zle -N peco_history
bindkey '^R' peco_history

# History search
zle -N history-beginning-search-backward-end history-search-end
zle -N history-beginning-search-forward-end history-search-end
bindkey "^P" history-beginning-search-backward-end
bindkey "^N" history-beginning-search-forward-end

# Vi Keybind
bindkey -M vicmd '1' vi-beginning-of-line
bindkey -M vicmd '0' vi-end-of-line

# Emacs Keybind
bindkey '^A' beginning-of-line
bindkey '^E' end-of-line
bindkey "^F" forward-char
bindkey "^B" backward-char
bindkey "^D" delete-char
bindkey "^K" kill-line
bindkey "^Y" yank

# --------------------------------
# alias
# --------------------------------

# source common aliases
source $HOME/.zsh/rc/alias.sh

alias history='history -i 1'
alias reload='exec zsh'

# Global aliases {{{
alias -g A="| awk"
alias -g G="| grep"
alias -g GV="| grep -v"
alias -g H="| head"
alias -g L="| $PAGER"
alias -g P=' --help | less'
alias -g R="| ruby -e"
alias -g S="| sed"
alias -g T="| tail"
alias -g V="| vim -R -"
alias -g U=' --help | head'
alias -g W="| wc"
alias -g X="| xargs"
alias -g GM="origin/master"

# copy
if type pbcopy &>/dev/null; then
  alias -g C='| pbcopy' # osx
elif type xsel &>/dev/null; then
  alias -g C='| sed -z "$ s/\n$//" | xsel --input --clipboard'  # linux
elif type putclip &>/dev/null; then
  alias -g C='| putclip' # windows
fi
# }}}

# misc
alias which='where'

# view
alias -g L='| less'

# z command
function _z_cd ()
{
  if [ "$1" = "" ]; then
    if [ "$(uname)" = "Linux" ]; then
      dir=$(_z 2>&1 | awk '{print $2}' | tac | peco)
    else
      dir=$(_z 2>&1 | awk '{print $2}' | tail -r | peco)
    fi
  else
    dir=$1
  fi
  if [ "$dir" != "" ]; then
    _z $dir
  fi
}
alias z=_z_cd

# --------------------------------
# command line stack
# --------------------------------

show_buffer_stack() {
  POSTDISPLAY="
stack: $LBUFFER"
  zle push-line-or-edit
}
zle -N show_buffer_stack
setopt noflowcontrol
bindkey '^Q' show_buffer_stack

if [ -f $HOME/.zshrc.private ]; then
  source $HOME/.zshrc.private
fi

# ---------------------------------
# fancy-ctrl-z
# ---------------------------------

fancy-ctrl-z () {
  if [[ $#BUFFER -eq 0 ]]; then
    BUFFER="fg"
    zle accept-line
  else
    zle push-input
    zle clear-screen
  fi
}
zle -N fancy-ctrl-z
bindkey '^Z' fancy-ctrl-z

# ------------------
# cuda configuration
# ------------------

if [ "$CUDA_PATH" = "" -a -e /usr/local/cuda ]; then
  export CUDA_PATH=/usr/local/cuda
fi
if [ -e "$CUDA_PATH" ]; then
  export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib:$LD_LIBRARY_PATH
  export LIBRARY_PATH=/usr/local/lib:/usr/lib:$LIBRARY_PATH
  # export CPATH=/usr/include:$CPATH
  # export CFLAGS=-I/usr/include
  export LDFLAGS="-L/usr/local/lib -L/usr/lib"

  export PATH=$CUDA_PATH/bin:$PATH
  # export CPATH=$CUDA_PATH/include:$CPATH
  export LD_LIBRARY_PATH=$CUDA_PATH/lib64:$CUDA_PATH/lib:$CUDA_PATH/targets/x86_64-linux/lib/stubs:$LD_LIBRARY_PATH
  # export CFLAGS=-I$CUDA_PATH/include
  export LDFLAGS="-L$CUDA_PATH/lib64 -L$CUDA_PATH/lib -L$CUDA_PATH/targets/x86_64-linux/lib/stubs"
fi

show-cuda

# !!! Slow !!!
# ---------------------------------

# if [ "$(uname)" = "Linux" ]; then
#   xmodmap $HOME/.Xmodmap &>/dev/null
# fi

alias ql="qlmanage -p 2>/dev/null"

# export FZF_DEFAULT_COMMAND='rg --files --hidden -g "!.git" -g "!.DS_Store"'
# export FZF_CTRL_T_COMMAND='fd --strip-cwd-prefix'
# export FZF_CTRL_R_OPTS='--reverse'

# export PATH="/usr/local/lib/ruby/gems/3.0.0/bin:$PATH"

if [ ! -z $CONDA_DEFAULT_ENV ]; then
  conda_path=$(dirname $CONDA_PYTHON_EXE)
  conda_env=$CONDA_DEFAULT_ENV
  conda deactivate
  source $conda_path/activate $conda_env
fi

neovim_autocd() {
    [[ $NVIM ]] && neovim_autocd.py
}
chpwd_functions+=( neovim_autocd )

# function nvim () {
#   if [[ $NVIM ]]; then
#     python3 -c "import os; import neovim; nvim = neovim.attach('socket', path=os.environ['NVIM']); nvim.command('edit $*')"
#   else
#     command nvim $*
#   fi
# }

alias feh='feh --keep-zoom-vp --auto-zoom'

function k%% () {
  for job_id in $(jobs | awk '{print $1}' | sed -r 's/\[([0-9]+)\]/\1/g'); do
    kill %${job_id}
  done
}

alias codeimg="vim -c 'set nonumber foldcolumn=9 listchars= ft=python'"
