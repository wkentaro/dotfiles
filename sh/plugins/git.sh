#!/bin/sh

# ----------------------------------------------------
# git aliases
# ----------------------------------------------------

# basic command
alias add='git add'
alias a='git add'
alias clone='git clone'
alias st='git status'
alias ss='git status --short'
alias ci='git commit -v'
alias commit='git commit'
alias cm='git commit -m "$@"'
alias c!='git commit -v --amend'
alias co='git checkout'
alias checkout='git checkout'
alias stash='git stash'
alias di='git diff'
alias log='git log'
alias lg='git lg'
alias submodule='git submodule'
alias pull='git pull'
alias pl='git pull'
alias push='git push'
# hub command
alias issue='hub issue'
# legit command
alias gsy='git sync'
alias sy='git sync'
alias gsw='git switch'
alias sw='git switch'
alias switch='git switch'

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
    hub browse -- pulls/$2 >/dev/null 2>&1
  else
    hub browse $1 pulls/$2 >/dev/null 2>&1
  fi
}
gis () {
  if [ "$1" = "" ]; then
    hub browse -- issues/$2 >/dev/null 2>&1
  else
    hub browse $1 issues/$2 >/dev/null 2>&1
  fi
}
alias gbw='hub browse $@ 2>/dev/null'
#}}}

# alias gbd='git branch --merged | grep -v "\*" | xargs -n 1 git branch -d'
alias gbdr='git branch -r --merged origin/master | grep "$GITHUB_USER\\/" | sed "s/$GITHUB_USER\\///" | egrep -v "HEAD|master|develop|release" | xargs git push $GITHUB_USER --delete'
alias gbD='git branch -D'
git_remote_to_local () {
  local branches
  branches=(`git branch --all | grep $GITHUB_USER | egrep -v 'HEAD|master|develop|release' | sed "s@^ *remotes/$GITHUB_USER/@@"`)
  for br in $branches; do
    git branch $br --track $GITHUB_USER/$br 2>/dev/null
  done
}
alias gremote2local=git_remote_to_local

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
