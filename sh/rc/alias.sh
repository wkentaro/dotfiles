# vim: set ft=sh:

# basic
alias sudo='sudo '
alias c='clear'
alias lv='less'

# open
type gnome-open &>/dev/null && alias open=gnome-open
alias o='open'
alias o.='open .'

# browsing
alias gcal='open https://www.google.com/calendar/render#g >/dev/null 2>&1'
alias gmail='open https://mail.google.com/mail/u/0/ >/dev/null 2>&1'

# vim
type vim &>/dev/null && {
  alias v='vim'
  alias vi='vim'
  alias vii='vim --noplugin'
  alias viii='vim -u NONE'
}
type nvim &>/dev/null && {
  alias n='nvim'
  alias nvi='nvim'
  alias nvii='nvim --noplugin'
  alias nviii='nvim -u NONE'
}

# emacs
alias emacs='emacs -nw'

# python
alias py='python'
alias ipy='ipython'
alias ipp='ptipython'

# ruby
alias irb='irb --simple-prompt'

# cmatrix
alias matrix='cmatrix -sb'

# tmux
alias t='tmux'
alias tls='tmux ls'
alias ta='tmux attach'
alias tat='tmux attach -t'
alias tn='tmux new'
alias tns='tmux new -s'

# gifify
gifify () { docker run -it --rm -v $(pwd):/data maxogden/gifify $@ }

# wstool
alias wl=wstool
alias wli='wstool info'
alias wlcd='wstool_cd'
alias wlset='wstool set'
alias wlup='wstool update'

# brew
if type brew &>/dev/null; then
  alias bubu='brew update && brew upgrade && brew cleanup'
  alias bububu='bubu && brew cask update && brew cask cleanup'
fi

# ----------------------------------------------------
# pandoc
# ----------------------------------------------------
md2rst () { pandoc --from=markdown --to=rst $1 }
rst2md () { pandoc --from=rst --to=markdown $1 }

# ----------------------------------------------------
# wrapping with rlwrap
# ----------------------------------------------------
if type rlwrap &>/dev/null; then
  alias eus='rlwrap eus'
  alias irteusgl='rlwrap irteusgl'
  alias roseus='rlwrap roseus'
  alias irb='rlwrap irb'
  alias clisp="rlwrap -b '(){}[],#\";| ' clisp"
fi

# ----------------------------------------------------
# ROS
# ----------------------------------------------------
if [ -d "/opt/ros" ]; then
  alias rqt_gui='rosrun rqt_gui rqt_gui'
  alias rqt_reconfigure='rosrun rqt_reconfigure rqt_reconfigure'
  alias rqt_image_view='rosrun rqt_image_view rqt_image_view'
  image_view () { rosrun image_view image_view image:=$1 }
fi

# ----------------------------------------------------
# ls aliases
# ----------------------------------------------------
alias sl='ls'
if ls --color &>/dev/null; then
  # GNU ls
  if [ $TERM = "dumb" ]; then
    # Disable colors in GVim
    alias ls='ls -F --show-control-chars'
    alias la='ls -ahF --show-control-chars'
    alias ll='ls -lhF --show-control-chars'
    alias lsa='ls -lahF --show-control-chars'
  else
    # Color settings for zsh complete candidates
    alias ls='ls -F --show-control-chars --color=always'
    alias la='ls -ahF --show-control-chars --color=always'
    alias ll='ls -lhF --show-control-chars --color=always'
    alias lsa='ls -lahF --show-control-chars --color=always'
    if type dircolors &>/dev/null; then
      if [ -f $HOME/.colorrc ]; then
        eval `dircolors $HOME/.colorrc`
      fi
    fi
  fi
else
  # Darwin ls
  alias ls='ls -F'
  alias la='ls -ahF'
  alias ll='ls -lhF'
  alias lsa='ls -lahF'
fi

# ----------------------------------------------------
# git aliases
# ----------------------------------------------------

# Use hub as git client
type hub &>/dev/null && alias git=hub

alias ga.='git add .'

alias gmpull='git pull $GITHUB_USER $(current_branch)'
alias gmpnp='git pull $GITHUB_USER $(current_branch) && git push $GITHUB_USER $(current_branch)'

alias ggpush!='git push origin $(current_branch) --force'
alias gmpush='git push $GITHUB_USER $(current_branch)'
alias gmpush!='git push $GITHUB_USER $(current_branch) --force'

alias grbg='git rebase origin/master'
alias grbgi='git rebase -i origin/master'

alias gcsmg='gcmsg'

# for hub command
alias gpr='hub pull-request'
alias gfork='hub fork'
gpl () {
  if [ "$1" = "" ]; then
    hub browse -- pulls >/dev/null 2>&1
  else
    hub browse $1 pulls >/dev/null 2>&1
  fi
}
gis () {
  if [ "$1" = "" ]; then
    hub browse -- issues >/dev/null 2>&1
  else
    hub browse $1 issues >/dev/null 2>&1
  fi
}
alias gbw='hub browse $@ 2>/dev/null'
#}}}

# alias gbd='git branch --merged | grep -v "\*" | xargs -n 1 git branch -d'
alias gbdr='git branch -r --merged origin/master | grep "$GITHUB_USER\\/" | sed "s/$GITHUB_USER\\///" | egrep -v "HEAD|master|develop|release" | xargs git push $GITHUB_USER --delete'
alias gbD='git branch -D'
alias gremote2local='cbranch=$(current_branch) ; git branch --all | grep $GITHUB_USER | egrep -v "HEAD|master|develop|release" | sed "s/^  remotes\/$GITHUB_USER\///" | xargs -n1 -I{} git branch {} --track $GITHUB_USER/{} ; git checkout $cbranch'

# commit each file
_git_commit_each_file () {
  [ "$1" != "" ] && {
    pushd `pwd` &>/dev/null
    cd $1
  }
  changed_files=`git status -s | grep "^[A-Z]" | sed 's/^...//g' | sed 's/ -> /,/g'`
  changed_files=(`echo $changed_files`)
  for file in $changed_files; do
    msg=`echo ${file} | sed "s/.*,//g"`
    echo "[${msg}]" > /tmp/git_commit_message_template
    files=`echo ${file} | tr ',' ' '`
    git commit --only ${files} --verbose --template /tmp/git_commit_message_template || break
  done
  [ "$1" != "" ] && popd &>/dev/null
}
alias gceach=_git_commit_each_file
