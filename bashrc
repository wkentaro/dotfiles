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
PS1='${debian_chroot:+($debian_chroot)}\[\e[00;32m\]\u@\h:\[\e[01;34m\]\W\[\033[01;35m\]$(parse_branch)\[\e[01;35m\]\[\e[0m\]$ '

# alias
alias l='ls -CF'
alias ll='ls -alF'
alias la='ls -A'
alias v='vim'
alias vi='vim'
alias c='clear'
alias h='history'
alias p='python'
alias ip='ipython'
alias g='git'
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
alias gignore='git update-index --assume-unchanged'
alias gignored='git ls-files -v | grep "^[[:lower:]]"'
alias git-svn-dcommit-push='git svn dcommit && git push github master:svntrunk'
alias gk='gitk --all --branches'
alias gl='git pull'
alias glg='git log --stat --max-count=10'
alias glgg='git log --graph --max-count=10'
alias glgga='git log --graph --decorate --all'
alias glo='git log --oneline --decorate --color'
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

    # Python
    export PYTHONPATH=$PYTHONPATH:$HOME/.libs/python2.7/site-packages
    # pylearn2
    export PYLEARN2_DATA_PATH=$HOME/Work/pylearn2/data
    export PYLEARN2_VIEWER_COMMAND='open -Wn'
else
    alias ls='ls --color=auto'
fi
