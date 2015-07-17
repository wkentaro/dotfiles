# vim: set ft=sh:

# ----------------------------------------------------
# Basics
# ----------------------------------------------------
alias c='clear'
alias lv='less'
if which vim >/dev/null 2>&1; then
    alias v='vim -u NONE'
    alias vi='vim --noplugin'
fi
alias py='python'
alias ipy='ipython'
alias emacs='emacs -nw'

# ----------------------------------------------------
# tmux aliases
# ----------------------------------------------------
alias t='tmux'
alias tls='tmux ls'
alias ta='tmux attach'
alias tat='tmux attach -t'
alias tn='tmux new'
alias tns='tmux new -s'

# ----------------------------------------------------
# Use rlwrap commands
# ----------------------------------------------------
if which rlwrap >/dev/null 2>&1; then
    alias eus='rlwrap eus'
    alias irteusgl='rlwrap irteusgl'
    alias roseus='rlwrap roseus'
    alias irb='rlwrap irb'
    alias clisp="rlwrap -b '(){}[],#\";| ' clisp"
fi

# ----------------------------------------------------
# ros
# ----------------------------------------------------
_image_view() {
    rosrun image_view image_view image:=$1
}
if [ -d "/opt/ros" ]; then
    alias rqt_gui='rosrun rqt_gui rqt_gui'
    alias rqt_reconfigure='rosrun rqt_reconfigure rqt_reconfigure'
    alias image_view=_image_view
fi

# ----------------------------------------------------
# open aliases
# ----------------------------------------------------
if which gnome-open >/dev/null 2>&1; then  # linux
    alias open='gnome-open'
    alias o='gnome-open'
    alias o.='gnome-open .'
elif which open >/dev/null 2>&1; then  # osx
    alias o='open'
    alias o.='open .'
fi

# ----------------------------------------------------
# browser
# ----------------------------------------------------
alias gcal='open https://www.google.com/calendar/render#g >/dev/null 2>&1'
alias gmail='open https://mail.google.com/mail/u/0/ >/dev/null 2>&1'
alias github='open https://github.com >/dev/null 2>&1'

# ----------------------------------------------------
# ls aliases
# ----------------------------------------------------
alias sl='ls'
if [ $TERM = "dumb" ]; then
    # Disable colors in GVim
    alias ls="ls -F --show-control-chars"
    alias la='ls -aF --show-control-chars'
    alias ll='ls -lahF --show-control-chars'
else
    # Color settings for zsh complete candidates
    alias ls='ls -F --show-control-chars --color=always'
    alias la='ls -aF --show-control-chars --color=always'
    alias ll='ls -lahF --show-control-chars --color=always'
    if which dircolors >/dev/null 2>&1; then
        [ -f $HOME/.colorrc ] && eval `dircolors $HOME/.colorrc`
    fi
fi

# ----------------------------------------------------
# git aliases
# ----------------------------------------------------
# Use hub as git client
if which hub >/dev/null 2>&1; then
    eval "`hub alias -s`"
fi
# command aliases
alias ga.='git add .'
alias gaa='git add --all'
alias gcln='git clone'
alias grm='git rm'
alias gmv='git mv'
alias ggpush!='git push origin $(current_branch) --force'
alias gmpush='git push $GITHUB_USER $(current_branch)'
alias gmpush!='git push $GITHUB_USER $(current_branch) --force'
alias grb='git rebase'
alias grbg='git rebase origin/master'
alias gmpull='git pull $GITHUB_USER $(current_branch)'
alias gmpnp='git pull $GITHUB_USER $(current_branch) && git push $GITHUB_USER $(current_branch)'
alias gcsmg='gcmsg'
alias gpr='hub pull-request'
alias gfork='hub fork'
alias gpl='hub browse -- pulls >/dev/null 2>&1'
alias gis='hub browse -- issues >/dev/null 2>&1'
alias gbw='git browse $@ >/dev/null 2>&1'
alias gbd='git branch --merged | grep -v "\*" | xargs -n 1 git branch -d'
alias gbdr='git branch -r --merged origin/master | grep "$GITHUB_USER\\/" | sed "s/$GITHUB_USER\\///" | egrep -v "HEAD|master|develop|release" | xargs git push $GITHUB_USER --delete'
alias grbg='git rebase origin/master'
alias grbgi='git rebase -i origin/master'
alias gbD='git branch -D'
alias gf='git fetch'
alias gfa='git fetch --all'
alias gfap='git fetch --all --prune'
alias gremote2local='cbranch=$(current_branch) ; git branch --all | grep $GITHUB_USER | egrep -v "HEAD|master|develop|release" | sed "s/^  remotes\/$GITHUB_USER\///" | xargs -n1 -I{} git branch {} --track $GITHUB_USER/{} ; git checkout $cbranch'
# commit each file
_git_commit_each_file () {
    if [ "$1" != "" ]; then
        pushd `pwd` >/dev/null 2>&1
        cd $1
    fi
    changed_files=`git status -s | grep "^[A-Z]" | sed 's/^...//g' | sed 's/ -> /,/g'`
    changed_files=(`echo $changed_files`)
    for file in $changed_files; do
        msg=`echo ${file} | sed "s/.*,//g"`
        echo "[${msg}]" > /tmp/git_commit_message_template
        files=`echo ${file} | tr ',' ' '`
        git commit --only ${files} --verbose --template /tmp/git_commit_message_template || break
    done
    if [ "$1" != "" ]; then
        popd >/dev/null 2>&1
    fi
}
alias gceach=_git_commit_each_file
