# encoding
export LANG='en_US.UTF-8'
export LC_CTYPE='en_US.UTF-8'
export LC_ALL='en_US.UTF-8'

# Prompt Setup
# git branch
parse_git_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
}
PS1='${debian_chroot:+($debian_chroot)}\[\e[00;32m\]\u@\h:\[\e[01;34m\]\W\[\033[01;35m\]$(parse_git_branch)\[\e[01;35m\]\[\e[0m\]$ '

# Mac
if [ `uname` = 'Darwin' ]; then
    # Shell
    # path
    export PATH=/usr/local/bin:$HOME/.bin:$PATH
    export PATH=$PATH:$HOME/Work/pylearn2/pylearn2/scripts
    # alias
    alias l='ls -CF'
    alias ls='ls -G'
    alias ll='ls -alF'
    alias la='ls -A'
    alias vi='vim'
    alias h='history'
    alias c='clear'
    alias cw='cd $HOME/Work'
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
    # virtualenv
    export WORKON_HOME=$HOME/.virtualenvs
    source /usr/local/bin/virtualenvwrapper.sh
fi
