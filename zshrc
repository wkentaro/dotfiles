# Path to your oh-my-zsh installation.
export ZSH=$HOME/.oh-my-zsh
export EDITOR=vim

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
plugins=(git vi-mode python)

source $ZSH/oh-my-zsh.sh

# User configuration

# export MANPATH="/usr/local/man:$MANPATH"

# You may need to manually set your language environment
# export LANG=en_US.UTF-8

# Preferred editor for local and remote sessions
# if [[ -n $SSH_CONNECTION ]]; then
#   export EDITOR='vim'
# else
#   export EDITOR='mvim'
# fi

# Compilation flags
# export ARCHFLAGS="-arch x86_64"

# ssh
# export SSH_KEY_PATH="~/.ssh/dsa_id"

# Set personal aliases, overriding those provided by oh-my-zsh libs,
# plugins, and themes. Aliases can be placed here, though oh-my-zsh
# users are encouraged to define aliases within the ZSH_CUSTOM folder.
# For a full list of active aliases, run `alias`.
#
# Example aliases
# alias zshconfig="mate ~/.zshrc"
# alias ohmyzsh="mate ~/.oh-my-zsh"

export LC_CTYPE='en_US.UTF-8'

# aliases
alias v='vim'
alias vi='vim'
alias c='clear'
alias h='history'
alias o='open'
alias o.='open .'
alias p='python'
alias ip='ipython'
alias gcln='git clone'

if [ `uname` = 'Darwin' ]; then
    # path
    export PATH="/usr/local/bin:$HOME/.bin:$PATH"
    export PATH="$HOME/Work/pylearn2/pylearn2/scripts:$PATH"
    # grep
    export GREP_OPTIONS='--color=always'
    export GREP_COLOR='1;35;40'
    # ls
    if [ -x /usr/local/bin/gdircolors ]; then
        eval `gdircolors $HOME/.colorrc`
        alias ls='gls --color=auto'
    fi

    # Python
    export PYTHONPATH=$PYTHONPATH:$HOME/.libs/python2.7/site-packages
    # pylearn2
    export PYLEARN2_DATA_PATH=$HOME/Work/pylearn2/data
    export PYLEARN2_VIEWER_COMMAND='open -Wn'
    # hub
    source /usr/local/share/zsh/site-functions
    eval "$(hub alias -s)"
else
    if [ -f /opt/ros/hydro/setup.zsh ]; then
        source /opt/ros/hydro/setup.zsh
        soft () {
            cd ~/catkin_ws/soft3
            source devel/setup.zsh
        }
    fi
    eval `dircolors $HOME/.colorrc`
    alias ls='ls --color=auto'
    alias emacs='emacs -nw'
    # hub
    eval "$(hub alias -s)"
fi

# bindkey
bindkey -M viins 'jj' vi-cmd-mode
bindkey '^R' history-incremental-search-backward
bindkey '^A' beginning-of-line
bindkey '^E' end-of-line
autoload history-search-end
zle -N history-beginning-search-backward-end history-search-end
zle -N history-beginning-search-forward-end history-search-end
bindkey "^P" history-beginning-search-backward-end
bindkey "^N" history-beginning-search-forward-end

# zsh options
setopt list_packed
setopt nolistbeep
setopt share_history

# functions
today () {
    INBOXDIR=$HOME/Inbox
    today=`date +"%Y%m%d"`
    if [ ! -d ${INBOXDIR}/${today} ]; then
        mkdir ${INBOXDIR}/${today}
    fi
    cd ${INBOXDIR}/${today}
}
google() {
    search=""
    echo "Googling: $@"
    for term in $@; do
        search="$search%20$term"
    done
    xdg-open "http://www.google.com/search?q=$search"
}
