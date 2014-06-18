# pathes
export PATH=/usr/local/bin:/Users/ken/.bin:$PATH

# python
export WORKON_HOME=~/.virtualenvs
. /usr/local/bin/virtualenvwrapper.sh

# alias
alias l='ls -CF'
alias ls='ls -G'
alias ll='ls -alF'
alias la='ls -A'
alias vi='vim'
alias h='history'
alias c='clear'

# Git branch in prompt.
parse_git_branch() {
  git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/ (\1)/'
}

# Prompt setup
PS1='${debian_chroot:+($debian_chroot)}\[\033[00;32m\]mac \t \[\033[01;34m\]\W\[\033[01;35m\]$(parse_git_branch)\[\033[0m\] $ '

# Colored less
export LESS='-gj10 --no-init --quit-if-one-screen --RAW-CONTROL-CHARS'
export LESSOPEN='| /usr/local/bin/src-hilite-lesspipe.sh %s'
