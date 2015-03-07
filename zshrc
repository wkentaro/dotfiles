# autoload first
autoload -Uz compinit && compinit
autoload history-search-end

#ct Path to your oh-my-zsh installation.
export ZSH=$HOME/.oh-my-zsh
export TERM=xterm-256color
export PATH=$HOME/.bin:$PATH
export PATH=$PATH:/usr/local/opt/go/libexec/bin
export PYTHONSTARTUP="$HOME/.pythonstartup"
if [ `uname` = 'Darwin' ]; then
  # zsh
  source /usr/local/share/zsh/site-functions
  # grep
  export GREP_OPTIONS='--color=always'
  export GREP_COLOR='1;35;40'
  alias octave='/usr/local/octave/3.8.0/bin/octave-3.8.0'
  if [ -f ~/.bashrc.eus ]; then
    source ~/.bashrc.eus
  fi
fi

# Uncomment the following line to disable auto-setting terminal title.
DISABLE_AUTO_TITLE="true"

# Custom plugins may be added to ~/.oh-my-zsh/custom/plugins/
# Add wisely, as too many plugins slow down shell startup.
plugins=(git hub gitignore gnu-utils z vi-mode brew python debian osx history)

source $ZSH/oh-my-zsh.sh

# Set name of the theme to load.
# Look in ~/.oh-my-zsh/themes/
# Optionally, if you set this to "random", it'll load a random theme each
# ZSH_THEME="wkentaro"
source ~/.dotfiles/wkentaro.zsh-theme

# User configuration
export MANPATH="/usr/local/man:$MANPATH"

# You may need to manually set your language environment
export LANG=en_US.UTF-8
export LC_CTYPE='en_US.UTF-8'

# editor
export EDITOR=vim

# Compilation flags
export ARCHFLAGS="-arch x86_64"

# ssh
export SSH_KEY_PATH="~/.ssh/id_rsa"

# alias
alias v='vim'
alias vi='vim'
alias c='clear'
alias py='python'
alias ipy='ipython'
alias sl='ls'
alias emacs='emacs -nw'
alias eshell='emacs --execute "(shell)"'
alias gcln='git clone'
alias gmpush='git push wkentaro $(current_branch)'
alias gmpull='git pull wkentaro $(current_branch)'
alias gmpnp='git pull wkentaro $(current_branch) && git push wkentaro $(current_branch)'
alias gcal='open https://www.google.com/calendar/render#g'
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

if [ -f /opt/ros/indigo/setup.zsh ]; then
  source ~/.zshrc.ros
  # rossetip
  source /opt/ros/indigo/setup.zsh
fi

# bindkey
bindkey '^R' history-incremental-search-backward
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

# zsh options
setopt list_packed
setopt nolistbeep
setopt share_history

# functions
function today {
    INBOXDIR=$HOME/Inbox
    today=`date +"%Y%m%d"`
    if [ ! -d ${INBOXDIR}/${today} ]; then
        mkdir ${INBOXDIR}/${today}
    fi
    cd ${INBOXDIR}/${today}
}

