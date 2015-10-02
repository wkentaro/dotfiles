# vim: set ft=zsh:
#
# ---------------------------------
# zsh options
# ---------------------------------

# completion
autoload -Uz compinit && compinit
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
setopt extended_glob
# setopt globdots  # enable completion for dotfiles
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'
zstyle ':completion:*' use-cache true
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

OS=$(uname)

# prefix: /usr/local
export PATH="/usr/local/bin:/usr/local/sbin:$PATH"
export MANPATH="/usr/local/man:$MANPATH"

# prefix: $HOME/local
export PATH="$HOME/.local/bin:$PATH"
export MANPATH="$HOME/.local/bin:$MANPATH"
export PYTHONPATH="$HOME/.local/lib/python2.7/site-packages:$PYTHONPATH"

# bookmark
hash -d dotfiles=$HOME/.dotfiles

# Python
export VIRTUALENV_USE_DISTRIBUTE=1

# Termcolor
export TERM=xterm-256color

# grep
if [ "$OS" = "Darwin" ]; then
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
export GITHUB_USER='wkentaro'

# Server in lab
export SSH_USER='wada'

# Travis
[ -f $HOME/.travis/travis.sh ] && source $HOME/.travis/travis.sh

# Improved less option
export LESS='--tabs=4 --no-init --LONG-PROMPT --ignore-case --quit-if-one-screen --RAW-CONTROL-CHARS'

# ---------------------------------
# zsh plugins
# ---------------------------------

source $HOME/.zsh/antibody/antibody/antibody.zsh

antibody bundle < $HOME/.zsh/antibody/bundles.txt

# oh-my-zsh
plugins=(git history pip python web-search vi-mode)
ZSH=$HOME/.zsh/oh-my-zsh
source $ZSH/oh-my-zsh.sh

# https://github.com/wkentaro/pycd
type pycd.sh &>/dev/null && source `which pycd.sh`

# https://github.com/wkentaro/wstool_cd
type wstool_cd.sh &>/dev/null && source `which wstool_cd.sh`

# local plugins
plugins=(
  $HOME/.sh/plugins/git.sh
  $HOME/.sh/plugins/browse.sh
  $HOME/.sh/plugins/restart-travis.sh
  $HOME/.zsh/plugins/demo_mode.zsh
)
for plugin in $plugins; do
  source $plugin
done
fpath=($HOME/.zsh/plugins $fpath)

# --------------------------------
# bindkey
# --------------------------------
if type percol &>/dev/null; then
  # percol history search
  # Ctrl-R
  function percol-history() {
    if [ "$OS" = "Linux" ]; then
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
      elif [[ "$LBUFFER" =~ "^(roscd|roslaunch|rosrun) " ]]; then
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
          ros_commands=(rosrun roscd rostopic rosmsg rosmsg-proto rospack rosnode)
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
    bindkey '^o' ros-bind
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
    if [ "$OS" = "Linux" ]; then
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
