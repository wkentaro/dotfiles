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

source $HOME/.zsh/antibody/path.zsh

antibody bundle < $HOME/.zsh/antibody/bundles.txt

# for rupa/z
if [ ! -e $HOME/.z ]; then
  touch $HOME/.z
fi

# load theme
if [[ $MY_ZSH_THEME = "" ]]; then
  antibody bundle wkentaro/wkentaro.zsh-theme
else
  antibody bundle $MY_ZSH_THEME
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
  $HOME/.sh/plugins/git.sh
  $HOME/.sh/plugins/ros.sh
  $HOME/.sh/plugins/ubuntu.sh
  $HOME/.sh/plugins/aws.sh
)
for plugin in $plugins; do
  source $plugin
done

# https://github.com/wkentaro/pycd
type pycd.sh &>/dev/null && source `which pycd.sh`

# ----------------------------
# Improved less option
# ----------------------------

export LESS='--tabs=4 --LONG-PROMPT --ignore-case --RAW-CONTROL-CHARS'

# --------------------------------
# bindkey
# --------------------------------

if type percol &>/dev/null; then
  # percol history search
  # Ctrl-R
  function percol-history() {
    if [ "$(uname)" = "Linux" ]; then
      BUFFER=$(fc -l -n 1 | tac | percol --query "$LBUFFER")
    else
      BUFFER=$(fc -l -n 1 | tail -r | percol --query "$LBUFFER")
    fi
    CURSOR=$#BUFFER         # move cursor
    zle -R -c               # refresh
  }
  zle -N percol-history
  bindkey '^R' percol-history

  # Alt-T
  if [ -d "/opt/ros" ]; then
    # rostopic search
    function search-rostopic-by-percol(){
      LBUFFER=$LBUFFER$(rostopic list | percol)
      zle -R -c
    }
    zle -N search-rostopic-by-percol
    bindkey '^[p' search-rostopic-by-percol

    function ros-bind () {
      local cmd
      local -a candidates
      candidates=(rosmsg rosmsg-proto rospack rostopic rosservice)
      if [ "$LBUFFER" = "" ]; then
        cmd=commands
      elif [[ "$LBUFFER" =~ "^(roscd|roslaunch) " ]]; then
        cmd=rospack
      elif [[ "$LBUFFER" =~ "^rostopic (echo|list|info)" ]]; then
        cmd=rostopic
      elif [[ "$LBUFFER" =~ "^rosmsg (show|list)" ]]; then
        cmd=rosmsg
      elif [[ "$LBUFFER" =~ "^rosservice (list|info)" ]]; then
        cmd=rosservice
      elif [[ "$LBUFFER" =~ "^rosnode (list|info)" ]]; then
        cmd=rosnode
      elif [[ "$LBUFFER" =~ "^rosbag record" ]]; then
        cmd=rostopic
      elif [ "$LBUFFER" = "image_view " ]; then
        cmd=rostopic
      else
        cmd=$(echo $candidates | xargs -n1 | percol)
      fi
      case $cmd in
        (commands)
          local -a ros_commands
          ros_commands=(rosrun roscd rostopic rosservice rosmsg rosmsg-proto rospack rosnode)
          LBUFFER="$(echo $ros_commands | xargs -n1 | percol) "
          ;;
        (rosmsg)
          LBUFFER=$LBUFFER$(rosmsg list | percol)
          ;;
        (rosmsg-proto)
          msg=$(rosmsg list | percol)
          if [ "$msg" != "" ]; then
            LBUFFER=$LBUFFER$(rosmsg-proto msg $msg)
          fi
          ;;
        (rospack)
          LBUFFER=$LBUFFER$(rospack list | awk '{print $1}' | percol)
          ;;
        (rosservice)
          LBUFFER=$LBUFFER$(rosservice list | percol)
          ;;
        (rosservice)
          LBUFFER=$LBUFFER$(rosservice list | percol)
          ;;
        (rostopic)
          LBUFFER=$LBUFFER$(rostopic list | percol)
          ;;
        (rosnode)
          LBUFFER=$LBUFFER$(rosnode list | percol)
          ;;
        (*) ;;
      esac
      zle -R -c
    }
    zle -N ros-bind
    bindkey '^[o' ros-bind
  fi
fi

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
source $HOME/.sh/rc/alias.sh

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
      dir=$(_z 2>&1 | awk '{print $2}' | tac | percol)
    else
      dir=$(_z 2>&1 | awk '{print $2}' | tail -r | percol)
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
if [ -e $CUDA_PATH ]; then
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

export PATH="/usr/local/lib/ruby/gems/3.0.0/bin:$PATH"

if which batcat>/dev/null; then
  alias bat=batcat
fi
if which fdfind>/dev/null; then
  alias fd=fdfind
fi
alias lv='bat --pager="less -R" --style plain'

if [ ! -z $CONDA_DEFAULT_ENV ]; then
  conda_path=$(dirname $CONDA_PYTHON_EXE)
  conda_env=$CONDA_DEFAULT_ENV
  conda deactivate
  source $conda_path/activate $conda_env
fi
