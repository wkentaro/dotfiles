#ct Path to your oh-my-zsh installation.
export ZSH=$HOME/.oh-my-zsh
export TERM=xterm-256color
if [ `uname` = 'Darwin' ]; then
    # path
    PATH="/usr/local/bin:$HOME/.bin:$PATH"
    PATH="/usr/local/opt/coreutils/libexec/gnubin:$PATH"
    PATH="/usr/local/opt/gnu-sed/libexec/gnubin:$PATH"
    export PATH="$HOME/Work/pylearn2/pylearn2/scripts:$PATH"
    export MANPATH="/usr/local/opt/coreutils/libexec/gnuman:$MANPATH"
    # grep
    export GREP_OPTIONS='--color=always'
    export GREP_COLOR='1;35;40'
    # pylearn2
    export PYLEARN2_DATA_PATH=$HOME/Work/pylearn2/data
    export PYLEARN2_VIEWER_COMMAND='open -Wn'
    alias octave='/usr/local/octave/3.8.0/bin/octave-3.8.0'
fi

# Set name of the theme to load.
# Look in ~/.oh-my-zsh/themes/
# Optionally, if you set this to "random", it'll load a random theme each
# time that oh-my-zsh is loaded.
# ZSH_THEME="robbyrussell"
ZSH_THEME="wkentaro"

# Uncomment the following line to use case-sensitive completion.
# CASE_SENSITIVE="true"

# Uncomment the following line to disable bi-weekly auto-update checks.
# DISABLE_AUTO_UPDATE="true"

# Uncomment the following line to change how often to auto-update (in days).
# export UPDATE_ZSH_DAYS=13

# Uncomment the following line to disable colors in ls.
# DISABLE_LS_COLORS="true"

# Uncomment the following line to disable auto-setting terminal title.
# DISABLE_AUTO_TITLE="true"

# Uncomment the following line to enable command auto-correction.
# ENABLE_CORRECTION="true"

# Uncomment the following line to display red dots whilst waiting for completion.
# COMPLETION_WAITING_DOTS="true"

# Uncomment the following line if you want to disable marking untracked files
# under VCS as dirty. This makes repository status check for large repositories
# much, much faster.
# DISABLE_UNTRACKED_FILES_DIRTY="true"

# Uncomment the following line if you want to change the command execution time
# stamp shown in the history command output.
# The optional three formats: "mm/dd/yyyy"|"dd.mm.yyyy"|"yyyy-mm-dd"
# HIST_STAMPS="mm/dd/yyyy"

# Would you like to use another custom folder than $ZSH/custom?
# ZSH_CUSTOM=/path/to/new-custom-folder

# Which plugins would you like to load? (plugins can be found in ~/.oh-my-zsh/plugins/*)
# Custom plugins may be added to ~/.oh-my-zsh/custom/plugins/
# Example format: plugins=(rails git textmate ruby lighthouse)
# Add wisely, as too many plugins slow down shell startup.
plugins=(git gitfast vi-mode python)

source $ZSH/oh-my-zsh.sh

# User configuration

# export MANPATH="/usr/local/man:$MANPATH"

# You may need to manually set your language environment
# export LANG=en_US.UTF-8
export LC_CTYPE='en_US.UTF-8'

# Preferred editor for local and remote sessions
export EDITOR=vim
# if [[ -n $SSH_CONNECTION ]]; then
#   export EDITOR='vim'
# else
#   export EDITOR='mvim'
# fi

# Compilation flags
# export ARCHFLAGS="-arch x86_64"

# ssh
export SSH_KEY_PATH="~/.ssh/id_rsa"

# Set personal aliases, overriding those provided by oh-my-zsh libs,
# plugins, and themes. Aliases can be placed here, though oh-my-zsh
# users are encouraged to define aliases within the ZSH_CUSTOM folder.
# For a full list of active aliases, run `alias`.
alias v='vim'
alias vi='vim'
alias c='clear'
alias h='history'
alias p='python'
alias ip='ipython'
alias sl='ls'
alias emacs='emacs -nw'
alias eshell='emacs --execute "(shell)"'
alias gcln='git clone'
alias gmpush='git push wkentaro $(current_branch)'
alias gmpull='git pull wkentaro $(current_branch)'
alias gmpnp='git pull wkentaro $(current_branch) && git push wkentaro $(current_branch)'
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
  eval `dircolors $HOME/.colorrc`
  alias ls='ls --color=auto'
fi

if [ `uname` = 'Darwin' ]; then
  source /usr/local/share/zsh/site-functions
else
  if [ -f /opt/ros/hydro/setup.zsh ]; then
    source ~/.zshrc.ros; rossetip;
    source /opt/ros/hydro/setup.zsh
  fi
fi

# bindkey
bindkey -M viins 'jj' vi-cmd-mode
bindkey -M viins '^J' vi-cmd-mode
bindkey '^R' history-incremental-search-backward
bindkey '^A' beginning-of-line
bindkey '^E' end-of-line
bindkey -M vicmd '1' beginning-of-line
bindkey -M vicmd '0' end-of-line
autoload history-search-end
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
function today () {
    INBOXDIR=$HOME/Inbox
    today=`date +"%Y%m%d"`
    if [ ! -d ${INBOXDIR}/${today} ]; then
        mkdir ${INBOXDIR}/${today}
    fi
    cd ${INBOXDIR}/${today}
}
function google () {
    search=""
    echo "Googling: $@"
    for term in $@; do
        search="$search%20$term"
    done
    xdg-open "http://www.google.com/search?q=$search"
}
function enshu () {
    cd ~/catkin_ws/enshu
    source ./devel/setup.zsh
}
function soft () {
    cd ~/catkin_ws/soft3
    source ./devel/setup.zsh
}
function semi () {
    cd ~/catkin_ws/semi
    source ./devel/setup.zsh
}
