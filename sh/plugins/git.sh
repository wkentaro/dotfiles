#!/bin/sh

# ----------------------------------------------------
# git aliases
# ----------------------------------------------------

# basic command
alias grh!='git reset --hard'
alias grh~='git reset HEAD~'
alias grh~1='git reset HEAD~1'
alias grh~2='git reset HEAD~2'
alias grh~3='git reset HEAD~3'
alias g1msg='git log -1 --format="%s" | cat'

# Use hub as git client
type hub &>/dev/null && alias git=hub

alias ga.='git add .'
alias co='git checkout'

alias gbug='git branch -u origin/$(current_branch)'
alias gbum='git branch -u wkentaro/$(current_branch)'
alias gmpull='git pull $GITHUB_USER $(current_branch)'
alias gmpnp='git pull $GITHUB_USER $(current_branch) && git push $GITHUB_USER $(current_branch)'

alias gp!='git push --force'
alias ggpush!='git push origin $(current_branch) --force'
alias gmpush='git push $GITHUB_USER $(current_branch)'
alias gmpush!='git push $GITHUB_USER $(current_branch) --force'

_is_option () {
  if [[ $1 =~ "^-.*" ]]; then
    return 0
  fi
  return 1
}

grbg () {
  local branch arg
  local -a opts args
  for arg in $@; do
    if _is_option $arg; then
      opts=($arg $opts)
    else
      args=($arg $args)
    fi
  done
  if [ ${#args} -eq 0 ]; then
    branch="master"
  else
    branch=${args[1]}
  fi
  git rebase origin/$branch $opts
}
if which compdef &>/dev/null; then
  compdef _git grbg=git-checkout 2>/dev/null
fi
alias grbgi='grbg --interactive'
if which compdef &>/dev/null; then
  compdef _git grbgi=git-checkout 2>/dev/null
fi

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
gbdra () {
  local remote
  if [ $# -eq 0 ]; then
    remote=$GITHUB_USER
  else
    remote=$1
  fi
  git branch -r --merged origin/master | grep "$remote\\/" | sed "s/$remote\\///" | egrep -v "HEAD|master|develop|release" | xargs git push $remote --delete
}
gbdr () {
  local remote branch
  if [ $# -eq 0 ]; then
    remote=$GITHUB_USER
    branch=$(current_branch)
  elif [ $# -eq 1 ]; then
    remote=$GITHUB_USER
    branch=$1
  else
    remote=$1
    branch=$2
  fi
  git push $remote $branch --delete
}
if which compdef &>/dev/null; then
  compdef _git gbdr=git-checkout 2>/dev/null
fi

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

_what_ros_package () {
  looking_path=$(pwd)
  found=$(find $looking_path -maxdepth 1 -iname package.xml | wc -l)
  while [ $found -eq 0 ]; do
    looking_path=$(dirname $looking_path)
    [ "$looking_path" = "/" ] && return
    found=$(find $looking_path -maxdepth 1 -iname package.xml | wc -l)
  done
  echo $(basename $looking_path)
}
# _git_commit_verbose () {
#   tmp_file=$(mktemp -t XXXXXX)
#   ros_package=$(_what_ros_package)
#   if [ "${ros_package}" != "" ]; then
#     echo "[${ros_package}] " > ${tmp_file}
#     git commit --verbose --template ${tmp_file}
#   else
#     git commit --verbose
#   fi
# }
# alias gc='_git_commit_verbose'
alias gc='git commit --verbose'
