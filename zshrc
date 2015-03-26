######################################
# zsh initialization
######################################
autoload -Uz compinit && compinit
setopt histignorealldups sharehistory
autoload history-search-end
setopt list_packed
setopt nolistbeep
setopt share_history
setopt auto_cd
setopt pushd_ignore_dups
setopt hist_ignore_space
setopt nobeep
setopt hist_ignore_dups
setopt share_history
zstyle ':completion:*:default' menu select=1
zstyle ':completion:*' matcher-list 'm:{a-z}={A-Z}'

######################################
# env
######################################
export PATH=/usr/local/bin:~/.bin:$PATH
export TERM=xterm-256color
export PYTHONSTARTUP="~/.pythonstartup"
export VIRTUALENV_USE_DISTRIBUTE=1
if [ `uname` = 'Darwin' ]; then
  export GREP_OPTIONS='--color=always'
  export GREP_COLOR='1;35;40'
fi

export ZSHDOT=~/.zsh
export ZSH=$ZSHDOT/oh-my-zsh
DISABLE_AUTO_TITLE='true'
plugins=(git hub gitignore gnu-utils z vi-mode brew python debian osx history)
source $ZSH/oh-my-zsh.sh

# ZSH_THEME="robbyrussell"
source $ZSHDOT/wkentaro.zsh-theme/wkentaro.zsh-theme

export MANPATH="/usr/local/man:$MANPATH"
export LANG=en_US.UTF-8
export LC_CTYPE='en_US.UTF-8'
export EDITOR=vim
export SSH_KEY_PATH="~/.ssh/id_rsa"

##################################
# bindkey
##################################
source $ZSHDOT/zaw/zaw.zsh
bindkey '^R' zaw-history
bindkey '^X^P' zaw-process
# bindkey '^R' history-incremental-search-backward
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

##################################
# alias
##################################
alias v='vim'
alias vi='vim'
alias c='clear'
alias py='python'
alias ipy='ipython'
alias sl='ls'
alias emacs='emacs -nw'
alias gcln='git clone'
alias grm='git rm'
alias gmv='git mv'
alias gmpush='git push wkentaro $(current_branch)'
alias gmpull='git pull wkentaro $(current_branch)'
alias gmpnp='git pull wkentaro $(current_branch) && git push wkentaro $(current_branch)'
alias gcal='open https://www.google.com/calendar/render#g'
alias gmail='open https://mail.google.com/mail/u/0/'
alias t='tmux'
alias tls='tmux ls'
alias ta='tmux attach'
alias tat='tmux attach -t'
alias tn='tmux new'
alias tns='tmux new -s'
alias gpr='hub pull-request'
alias gbw='git browse'
alias gbd='git branch --merged | grep -v "\*" | xargs -n 1 git branch -d'
alias gf='git fetch'
alias gfa='git fetch --all'
# copy
if which pbcopy >/dev/null 2>&1 ; then 
  alias -g C='| pbcopy' # mac
elif which xsel >/dev/null 2>&1 ; then 
  alias -g C='| xsel --input --clipboard' # ubuntu
elif which putclip >/dev/null 2>&1 ; then 
  alias -g C='| putclip' # cygwin
fi
# hub
if which hub >/dev/null 2>&1; then
  eval "$(hub alias -s)"
fi
# open
if which gnome-open >/dev/null 2>&1; then
  alias open='gnome-open'
  alias o='gnome-open'
  alias o.='gnome-open .'
elif which open >/dev/null 2>&1; then
  alias o='open'
  alias o.='open .'
fi
# ls
if which dircolors >/dev/null 2>&1; then
  if [ -f ~/.colorrc ]; then
    eval `dircolors ~/.colorrc`
  fi
  if [ `uname` = 'Darwin' ]; then
    alias ls='gls --color=auto'
  else;
    alias ls='ls --color=auto'
  fi
fi

##################################
# function
##################################
function today {
    INBOXDIR=$HOME/Inbox
    today=`date +"%Y%m%d"`
    if [ ! -d ${INBOXDIR}/${today} ]; then
        mkdir ${INBOXDIR}/${today}
    fi
    cd ${INBOXDIR}/${today}
}

