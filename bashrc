# vim: set ft=sh:

OS=$(uname)

if [ "$OS" = "Darwin" ]; then
  if [ -f $(brew --prefix)/etc/bash_completion ]; then
    . $(brew --prefix)/etc/bash_completion
  fi
fi
export PATH=$HOME/.local/bin:$PATH

type wstool_cd.sh &>/dev/null && source `which wstool_cd.sh`
type pycd.sh &>/dev/null && source `which pycd.sh`

# encoding
export LC_CTYPE='en_US.UTF-8'

# terminal color
export TERM=xterm-256color

# prompt setup
parse_branch() {
  local branch
  branch=`git branch 2>/dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'`
  [ "$branch" != "" ] && printf "\e[0m on \e[0;35m$branch\e[0m"
}
show_venv() {
  if [ ! -z $CONDA_PREFIX ]; then
    printf " workon \e[0;34mconda:$CONDA_DEFAULT_ENV\e[0m"
  fi
}
PS1='${debian_chroot:+($debian_chroot)}\e[0;35m\u\e[0m at \e[0;33m\h\e[0m in \e[0;32m\w\e[0m tm \e[0;37m$(date +%H:%M)$(parse_branch)\e[0m$(show_venv)\n$ '

plugins=(
  $HOME/.sh/plugins/browse.sh
  $HOME/.sh/plugins/git.sh
  $HOME/.sh/plugins/ros.sh
)
for plugin in ${plugins[@]}; do
  source $plugin
done

# -------------------------------
# alias
# -------------------------------
# source common aliases
source $HOME/.sh/rc/alias.sh

source $HOME/.bash/rc/alias.sh

# google command
google () {
  search=""
  for term in $@; do
      search="$search%20$term"
  done
  open "http://www.google.com/search?q=$search"
}

#------------------------------------------------
# alias
# -----------------------------------------------
# Basics
alias h=history
alias md='mkdir -p'
alias rd=rmdir
alias d='dirs -v | head -10'

# cd aliases
- () {
  cd -
}
alias ..='cd ..'
alias ...='cd ../..'
alias cd..='cd ..'
alias cd...='cd ../..'
alias cd....='cd ../../..'
alias cd.....='cd ../../../..'
cd () {
  if [[ "x$*" = "x..." ]]
  then
    cd ../..
  elif [[ "x$*" = "x...." ]]
  then
    cd ../../..
  elif [[ "x$*" = "x....." ]]
  then
    cd ../../../..
  elif [[ "x$*" = "x......" ]]
  then
    cd ../../../../..
  elif [ -d ~/.autoenv ]
  then
    source ~/.autoenv/activate.sh
    autoenv_cd "$@"
  else
    builtin cd "$@"
  fi
}

# git aliases
git_current_branch() {
  git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'
}
alias current_branch=git_current_branch

export VIM_COLORSCHEME='default'

export EDITOR='vim'
export LESS='--tabs=4 --LONG-PROMPT --ignore-case --RAW-CONTROL-CHARS'

# history search bindkey
_replace_by_history() {
  local l=$(HISTTIMEFORMAT= history | tac | sed -e 's/^\s*[0-9]\+\s\+//' | percol --query "$READLINE_LINE")
  READLINE_LINE="$l"
  READLINE_POINT=${#l}
}
bind -x '"\C-r": _replace_by_history'

alias rm='rm -i'
alias cp='cp -i'
alias mv='mv -i'
