# encoding
export LC_CTYPE='en_US.UTF-8'

# Prompt Setup
# git branch
current_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'
}
parse_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}
PS1='${debian_chroot:+($debian_chroot)}\[\e[00;32m\]\u@\h:\[\e[01;34m\]\W\[\033[01;35m\]$(parse_branch)\[\e[01;35m\]\[\e[0m\] $ '

# alias
alias trr='emacs --execute "(trr)"'
alias ..='cd ..'
alias ...='cd ../..'
alias 1='cd -'
alias 2='cd -2'
alias 3='cd -3'
alias 4='cd -4'
alias 5='cd -5'
alias 6='cd -6'
alias 7='cd -7'
alias 8='cd -8'
alias 9='cd -9'
alias _=sudo
alias afind='ack-grep -il'
alias c=clear
alias cd..='cd ..'
alias cd...='cd ../..'
alias cd....='cd ../../..'
alias cd.....='cd ../../../..'
alias d='dirs -v | head -10'
alias emacs='emacs -nw'
alias g=git
alias ga='git add'
alias gap='git add --patch'
alias gb='git branch'
alias gba='git branch -a'
alias gbr='git branch --remote'
alias gc='git commit -v'
alias 'gc!'='git commit -v --amend'
alias gca='git commit -v -a'
alias 'gca!'='git commit -v -a --amend'
alias gcl='git config --list'
alias gclean='git reset --hard && git clean -dfx'
alias gcln='git clone'
alias gcm='git checkout master'
alias gcmsg='git commit -m'
alias gco='git checkout'
alias gcount='git shortlog -sn'
alias gcp='git cherry-pick'
alias gcs='git commit -S'
alias gd='git diff'
alias gdc='git diff --cached'
alias gdt='git difftool'
alias gg='git gui citool'
alias gga='git gui citool --amend'
alias ggpnp='git pull origin $(current_branch) && git push origin $(current_branch)'
alias ggpull='git pull origin $(current_branch)'
alias ggpur='git pull --rebase origin $(current_branch)'
alias ggpush='git push origin $(current_branch)'
alias gmpush='git push wkentaro $(current_branch)'
alias gignore='git update-index --assume-unchanged'
alias gignored='git ls-files -v | grep "^[[:lower:]]"'
alias git=hub
alias git-svn-dcommit-push='git svn dcommit && git push github master:svntrunk'
alias gk='gitk --all --branches'
alias gl='git pull'
alias glg='git log --stat --max-count=10'
alias glgg='git log --graph --max-count=10'
alias glgga='git log --graph --decorate --all'
alias glo='git log --oneline --decorate --color'
alias globurl='noglob urlglobber '
alias glog='git log --oneline --decorate --color --graph'
alias glp=_git_log_prettily
alias gm='git merge'
alias gmt='git mergetool --no-prompt'
alias gp='git push'
alias gpoat='git push origin --all && git push origin --tags'
alias gr='git remote'
alias grba='git rebase --abort'
alias grbc='git rebase --continue'
alias grbi='git rebase -i'
alias grh='git reset HEAD'
alias grhh='git reset HEAD --hard'
alias grmv='git remote rename'
alias grrm='git remote remove'
alias grset='git remote set-url'
alias grt='cd $(git rev-parse --show-toplevel || echo ".")'
alias grup='git remote update'
alias grv='git remote -v'
alias gsd='git svn dcommit'
alias gsps='git show --pretty=short --show-signature'
alias gsr='git svn rebase'
alias gss='git status -s'
alias gst='git status'
alias gsta='git stash'
alias gstd='git stash drop'
alias gstp='git stash pop'
alias gsts='git stash show --text'
alias gts='git tag -s'
alias gunignore='git update-index --no-assume-unchanged'
alias gunwip='git log -n 1 | grep -q -c "\-\-wip\-\-" && git reset HEAD~1'
alias gup='git pull --rebase'
alias gvt='git verify-tag'
alias gwc='git whatchanged -p --abbrev-commit --pretty=medium'
alias gwip='git add -A; git ls-files --deleted -z | xargs -r0 git rm; git commit -m "--wip--"'
alias h=history
alias history='fc -l 1'
alias ip=ipython
alias ls='ls --color=auto'
alias sl='ls'
alias l='ls -lah'
alias la='ls -lAh'
alias ll='ls -lh'
alias lsa='ls -lah'
alias md='mkdir -p'
alias o=open
alias o.='open .'
alias p=python
alias please=sudo
alias po=popd
alias pu=pushd
alias pyfind='find . -name "*.py"'
alias pygrep='grep --include="*.py"'
alias rd=rmdir
alias v=vim
alias vi=vim
alias which-command=whence

# Mac
if [ `uname` = 'Darwin' ]; then
    # path
    export PATH="/usr/local/bin:$HOME/.bin:$PATH"
    export PATH="$HOME/Work/pylearn2/pylearn2/scripts:$PATH"
    # alias
    alias ls='ls -G'
    # grep
    export GREP_OPTIONS='--color=always'
    export GREP_COLOR='1;35;40'
    # ls
    if [ -x /usr/local/bin/gdircolors ]; then
        eval `gdircolors $HOME/.colorrc`
        alias ls='gls --color=auto'
    fi

    # node
    source /usr/local/etc/bash_completion.d
    # Python
    export PYTHONPATH=$PYTHONPATH:$HOME/.libs/python2.7/site-packages
    # pylearn2
    export PYLEARN2_DATA_PATH=$HOME/Work/pylearn2/data
    export PYLEARN2_VIEWER_COMMAND='open -Wn'
    # hub
    source /usr/local/share/zsh/site-functions
    eval "$(hub alias -s)"
else
    if [ -f $HOME/.colorrc ]; then
        eval `dircolors $HOME/.colorrc`
    fi
    if [ -f /opt/ros/hydro/setup.bash ]; then
        source /opt/ros/hydro/setup.bash
        semi () {
            cd ~/catkin_ws/semi
        }
    fi
    alias ls='ls --color=auto'
    alias emacs='emacs -nw'
    alias open='gnome-open'
    eval "$(hub alias -s)"
    alias i='irteusgl'
fi

google() {
    search=""
    echo "Googling: $@"
    for term in $@; do
        search="$search%20$term"
    done
    xdg-open "http://www.google.com/search?q=$search"
}
