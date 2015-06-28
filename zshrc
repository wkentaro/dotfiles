# vim: set ft=zsh:
#
# ---------------------------------
# zsh initialization
# ---------------------------------
# autoload -Uz compinit && compinit
setopt histignorealldups
autoload history-search-end
setopt list_packed
setopt nolistbeep
setopt auto_cd
setopt pushd_ignore_dups
setopt hist_ignore_space
setopt nobeep
setopt hist_ignore_dups
zstyle ':completion:*:default' menu select=1
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'

# prefix: /usr/local
export PATH="/usr/local/bin:$PATH"
export MANPATH="/usr/local/man:$MANPATH"

# prefix: ~/.local
export PATH="$HOME/.local/bin:$PATH"
export MANPATH="$HOME/.local/bin:$MANPATH"

# Python
export PYTHONSTARTUP=~/.pythonstartup
export VIRTUALENV_USE_DISTRIBUTE=1

# Termcolor
export TERM=xterm-256color

# grep
if [ `uname` = 'Darwin' ]; then
  export GREP_OPTIONS='--color=always'
  export GREP_COLOR='1;35;40'
fi

# Encoding
export LANG='en_US.UTF-8'
export LC_CTYPE='en_US.UTF-8'

# Editor
export EDITOR='vim'

# ssh
export SSH_KEY_PATH='~/.ssh/id_rsa'

# GitHub
export GITHUB_USER='wkentaro'

# Server in lab
export SSH_USER='wada'

# Travis
[ -f ~/.travis/travis.sh ] && source ~/.travis/travis.sh

# Improved less option
export LESS='--tabs=4 --no-init --LONG-PROMPT --ignore-case --quit-if-one-screen --RAW-CONTROL-CHARS'

# ---------------------------------
# zsh plugins
# ---------------------------------
# source antigen
source ~/.zsh/antigen/antigen.zsh

# Load the oh-my-zsh's library.
antigen use oh-my-zsh

# Bundles from the default repo (robbyrussell's oh-my-zsh).
antigen bundle git
antigen bundle gnu-utils
antigen bundle history
antigen bundle pip
antigen bundle python
antigen bundle web-search
antigen bundle vi-mode
antigen bundle z

# Syntax highlighting bundle.
antigen bundle zsh-users/zsh-syntax-highlighting

# TOO SLOW
# # History searching bundle. #{{{
# antigen bundle zsh-users/zsh-history-substring-search
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
# #}}}

# Additional zsh completion
antigen bundle zsh-users/zsh-completions src

# use haskell to run faster
# osx: brew cask install haskell-platform
# linux: sudo apt-get install cabal-install
antigen bundle olivierverdier/zsh-git-prompt zshrc.sh
GIT_PROMPT_EXECUTABLE="haskell"
USE_ZSH_GIT_PROMPT=1

# Load the theme.
antigen theme wkentaro/wkentaro.zsh-theme wkentaro

# Tell antigen that you're done.
antigen apply

# --------------------------------
# bindkey
# --------------------------------
function percol-history() {
  LBUFFER=$(fc -l 1 | tac | percol | sed -r "s/^ *[0-9]*(\*)? *//g")
  zle -R -c
}
zle -N percol-history
bindkey '^R' percol-history
bindkey '^A' beginning-of-line
bindkey '^E' end-of-line
zle -N history-beginning-search-backward-end history-search-end
zle -N history-beginning-search-forward-end history-search-end
bindkey "^P" history-beginning-search-backward-end
bindkey "^N" history-beginning-search-forward-end
bindkey "^F" forward-char
bindkey "^B" backward-char
bindkey "^D" delete-char
bindkey "^K" kill-line
bindkey "^Y" yank

# --------------------------------
# alias
# --------------------------------
# source common aliases
source ~/.sh/rc/alias.sh

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
# copy
if which pbcopy >/dev/null 2>&1 ; then 
  alias -g C='| pbcopy' # mac
elif which xsel >/dev/null 2>&1 ; then 
  alias -g C='| xsel --input --clipboard' # ubuntu
elif which putclip >/dev/null 2>&1 ; then 
  alias -g C='| putclip' # cygwin
fi
# }}}

# view
alias -g L='| less'

# z command
function _zcd ()
{
  if [ "$1" = "" ]; then
    dir=$(_z 2>&1 | sed -E "s/^([0-9]|\.)* *//g" | tac | percol)
  else
    dir=$1
  fi
  if [ "$dir" != "" ]; then
    _z $dir
  fi
}
alias z='_zcd'

# alias for linux
if [ `uname` = 'Linux' ]; then
  source ~/.zsh/zshrc.linux
fi

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
