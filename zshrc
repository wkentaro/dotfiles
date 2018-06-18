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
#   export PATH="/usr/local/opt/python@2/libexec/bin:$PATH"
    export PATH="/usr/local/opt/python/libexec/bin:$PATH"
fi

# bookmark
# hash -d dotfiles=$HOME/.dotfiles

# Python
export VIRTUALENV_USE_DISTRIBUTE=1

# Termcolor
export TERM=xterm-256color

# grep
if [ "$(uname)" = "Darwin" ]; then
  export GREP_OPTIONS='--color=always'
  export GREP_COLOR='1;35;40'
fi

# Encoding
export LANG='en_US.UTF-8'
export LC_CTYPE='en_US.UTF-8'
# export LC_ALL='C'

# Editor
export EDITOR='vim'

# ssh
export SSH_KEY_PATH='$HOME/.ssh/id_rsa'

# GitHub
if [ -z $GITHUB_USER ]; then
  export GITHUB_USER='wkentaro'
fi

# Server in lab
export SSH_USER='wada'

# Homebrew
export HOMEBREW_NO_AUTO_UPDATE=1

# Chainer
export CHAINER_DATASET_ROOT=$HOME/data/datasets

# Travis
[ -f $HOME/.travis/travis.sh ] && source $HOME/.travis/travis.sh

# ---------------------------------
# zsh plugins
# ---------------------------------

source $HOME/.zsh/antibody/path.zsh

antibody bundle < $HOME/.zsh/antibody/bundles.txt

# load theme
if [[ $MY_ZSH_THEME = "" ]]; then
  antibody bundle wkentaro/wkentaro.zsh-theme
else
  antibody bundle $MY_ZSH_THEME
fi

# enhancd
export ENHANCD_DISABLE_HOME=1
export ENHANCD_DISABLE_HYPHEN=1

# oh-my-zsh
DISABLE_AUTO_UPDATE=true
plugins=(git history python web-search vi-mode)
ZSH=$HOME/.zsh/oh-my-zsh
# FIXME: parser error in .zcompdump
source $ZSH/oh-my-zsh.sh
# FIXME: no completion function without this
# compinit 2>/dev/null

# https://github.com/wkentaro/pycd
type pycd.sh &>/dev/null && source `which pycd.sh`

# https://github.com/wkentaro/wstool_cd
type wstool_cd.sh &>/dev/null && source `which wstool_cd.sh`

# local plugins
plugins=(
  $HOME/.sh/plugins/git.sh
  $HOME/.sh/plugins/restart-travis.sh
  $HOME/.sh/plugins/ros.sh
  $HOME/.sh/plugins/gshell.sh
  $HOME/.sh/plugins/ubuntu.sh
)
for plugin in $plugins; do
  source $plugin
done
fpath=($HOME/.zsh/plugins $fpath)

# completion
autoload -Uz compinit
typeset -i updated_at=$(date +'%j' -r ~/.zcompdump 2>/dev/null || stat -f '%Sm' -t '%j' ~/.zcompdump 2>/dev/null)
if [ $(date +'%j') != $updated_at ]; then
  compinit -i
else
  compinit -C -i
fi

compdef _todo todo

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

# # History searching bundle. #{{{
# # zsh-users/zsh-history-substring-search
# # bind UP and DOWN arrow keys
# zmodload zsh/terminfo
# bindkey "$terminfo[kcuu1]" history-substring-search-up
# bindkey "$terminfo[kcud1]" history-substring-search-down
# # bind UP and DOWN arrow keys (compatibility fallback
# # for Ubuntu 12.04, Fedora 21, and MacOSX 10.9 users)
# bindkey '^[[A' history-substring-search-up
# bindkey '^[[B' history-substring-search-down
# # bind P and N for EMACS mode
# bindkey '^P' history-substring-search-up
# bindkey '^N' history-substring-search-down
# # bind k and j for VI mode
# bindkey -M vicmd 'k' history-substring-search-up
# bindkey -M vicmd 'j' history-substring-search-down
##}}}


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

alias reload='exec zsh'

# Global aliases {{{
alias -g A="| awk"
alias -g G="| grep"
alias -g GV="| grep -v"
alias -g H="| head"
alias -g F="| fpp"
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
  alias -g C='| xsel --input --clipboard'  # linux
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

if which autossh &>/dev/null; then
  autossh -f -N aries-tunnel
fi

# ---------------------------------
# ascii art
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

# -----------------
# Travis completion
# -----------------

[ -f ~/.travis/travis.sh ] && source ~/.travis/travis.sh

# -----------------
# ghi configuration
# -----------------

export GHI_TOKEN=$GITHUB_TOKEN

# ------------------
# cuda configuration
# ------------------

export LD_LIBRARY_PATH=/usr/local/lib:/usr/lib:$LD_LIBRARY_PATH
export LIBRARY_PATH=/usr/local/lib:/usr/lib:$LIBRARY_PATH
export CPATH=/usr/include:$CPATH
export CFLAGS=-I/usr/include
export LDFLAGS="-L/usr/local/lib -L/usr/lib"
if [ -e /usr/local/cuda ]; then
  export CUDA_PATH=/usr/local/cuda
  export PATH=$CUDA_PATH/bin:$PATH
  export CPATH=$CUDA_PATH/include:$CPATH
  export LD_LIBRARY_PATH=$CUDA_PATH/lib64:$CUDA_PATH/lib:$LD_LIBRARY_PATH
  export CFLAGS=-I$CUDA_PATH/include
  export LDFLAGS="-L$CUDA_PATH/lib64 -L$CUDA_PATH/lib"
  # cudnn
  if [ "$CUDNN_PATH" = "" ]; then
    export CUDNN_PATH=~/.cudnn/active/cuda
  fi
  export LD_LIBRARY_PATH=~/.cudnn/active/cuda/lib64:$LD_LIBRARY_PATH
  export CPATH=~/.cudnn/active/cuda/include:$CPATH
  export LIBRARY_PATH=~/.cudnn/active/cuda/lib64:$LIBRARY_PATH
fi

# ----------------------
# anaconda configuration
# ----------------------

# activate () {
#   if [ ! -e $HOME/.anaconda2/bin/activate ]; then
#     echo 'Please install anaconda'
#     return 1
#   fi
#   export _PYTHONPATH=$PYTHONPATH
#   unset PYTHONPATH
#   source $HOME/.anaconda2/bin/activate $1
# }
# _activate () {
#   local -a reply
#   if [[ ${CURRENT} = 2 ]]; then
#     reply=($(command ls ~/.anaconda2/envs))
#   fi
#   _describe 'values' reply
# }
# compdef _activate activate
# deactivate () {
#   source deactivate
#   export PYTHONPATH=$_PYTHONPATH
#   unset _PYTHONPATH
# }
# alias ac=activate
# alias da=deactivate

users_by_ps () {
  ps auxwww | awk '{print $1}' | egrep "$(command ls /home)" | sort | uniq -c | sort -nr | xargs
}

# !!! Slow !!!
# ---------------------------------

if [ "$(uname)" = "Linux" ]; then
  xmodmap $HOME/.Xmodmap &>/dev/null
fi

show_cuda

if which dropbox &>/dev/null; then
  echo "Dropbox: $(dropbox status | sed 2d)"
fi

if [ ! -z $CONDA_DEFAULT_ENV ]; then
  if [ ! -z $CONDA_PREFIX ]; then
    source $CONDA_PREFIX/bin/activate $CONDA_DEFAULT_ENV
  else
    source $(dirname $CONDA_PYTHON_EXE)/activate $CONDA_DEFAULT_ENV
  fi
fi

store_env_for_ros () {
  export _PYTHONPATH=$PYTHONPATH
  export _LD_LIBRARY_PATH=$LD_LIBRARY_PATH
  export _CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
}

restore_env_for_ros () {
  export PYTHONPATH=$_PYTHONPATH
  export LD_LIBRARY_PATH=$_LD_LIBRARY_PATH
  export CMAKE_PREFIX_PATH=$_CMAKE_PREFIX_PATH
}

# setup for go
export GOPATH=$HOME/.go
export PATH=$GOPATH/bin:$PATH

alias istcluster='ssh istcluster'
alias jenkins='ssh jenkins'
alias aries='ssh aries'

alias jhoop='ssh jhoop'
alias jdlbox7='ssh jdlbox7'
alias jdlbox8='ssh jdlbox8'
alias jdlbox9='ssh jdlbox9'
alias jdlboxs1='ssh jdlboxs1'
alias jdlboxs2='ssh jdlboxs2'

alias hoop='ssh hoop'
alias dlbox7='ssh dlbox7'
alias dlbox8='ssh dlbox8'
alias dlbox9='ssh dlbox9'
alias dlboxs1='ssh dlboxs1'
alias dlboxs2='ssh dlboxs2'
