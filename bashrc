# Path ------------------------------------------------
export PATH=/usr/local/bin:$HOME/.bin:$PATH

# LANG ------------------------------------------------
export LANG=en_US.UTF-8

# Alias -----------------------------------------------
alias l='ls -CF'
alias ls='ls -G'
alias ll='ls -alF'
alias la='ls -A'
alias vi='vim'
alias h='history'
alias c='clear'
alias ipython='ipython --no-confirm'

# Git branch in prompt --------------------------------
parse_git_branch() {
  git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
}

# Prompt setup ----------------------------------------
PS1='${debian_chroot:+($debian_chroot)}\[\e[00;32m\]\u@\h:\[\e[01;34m\]\W\[\033[01;35m\]$(parse_git_branch)\[\e[01;35m\]\[\e[0m\]$ '

# colored ls ------------------------------------------
if [ -x /usr/local/bin/gdircolors ]; then
    eval `gdircolors ~/.colorrc`
    alias ls='gls --color=auto'
fi
