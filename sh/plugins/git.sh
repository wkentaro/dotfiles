#!/bin/sh

# ----------------------------------------------------
# git aliases
# ----------------------------------------------------

# basic command
alias gfa='git fetch --all'
alias grh!='git reset --hard'
alias grh~='git reset HEAD~'
alias grh~1='git reset HEAD~1'
alias grh~2='git reset HEAD~2'
alias grh~3='git reset HEAD~3'
alias g1msg='git log -1 --format="%s" | cat'
alias g1hash='git log -1 --format="%h" | cat'
grahttp () {
  if [ "$#" != 1 ]; then
    echo "Usage: $0 GITHUB_USER"
  fi
  HUB_PROTOCOL=https hub remote add $1 && git fetch $1
}
grassh () {
  if [ "$#" != 1 ]; then
    echo "Usage: $0 GITHUB_USER"
  fi
  HUB_PROTOCOL=ssh hub remote add $1 && git fetch $1
}
alias grahttpm='grahttp ${GITHUB_USER}'
alias grasshm='grassh ${GITHUB_USER}'

gcauto () {
  files=$(git status --porcelain | grep -e '^M' | awk '{print $2}')
  tmpfile=$(mktemp)
  echo "Update $files" > $tmpfile
  git commit --edit --file $tmpfile
  rm -f $tmpfile
}
alias gsti='git status --ignored'

# Use hub as git client
type hub &>/dev/null && alias git=hub

alias ga.='git add .'
alias co='git checkout'

alias gbug='git branch -u origin/$(git_current_branch)'
alias gbum='git branch -u $GITHUB_USER/$(git_current_branch)'
alias gul='git pull upstream $(git_current_branch)'
alias gupull='git pull upstream $(git_current_branch)'
alias gml='git pull $GITHUB_USER $(git_current_branch)'
alias gmpull='git pull $GITHUB_USER $(git_current_branch)'
alias gmpnp='git pull $GITHUB_USER $(git_current_branch) && git push $GITHUB_USER $(git_current_branch)'

alias gbg='git branch --all | command grep origin | sed "s/ *//g"'
alias gbu='git branch --all | command grep upstream | sed "s/ *//g"'
alias gbm='git branch --all | command grep $GITHUB_USER | sed "s/ *//g"'

ggp () {
  if [[ "$#" != 0 ]] && [[ "$#" != 1 ]]; then
    git push origin "${*}" && git branch -u origin/${1}
  else
    [[ "$#" = 0 ]] && local b="$(git_current_branch)"
    git push origin "${b:=$1}" && git branch -u origin/${b:=$1}
  fi
}
gmp () {
  if [[ "$#" != 0 ]] && [[ "$#" != 1 ]]; then
    git push $GITHUB_USER "${*}" && git branch -u $GITHUB_USER/${1}
  else
    [[ "$#" = 0 ]] && local b="$(git_current_branch)"
    git push $GITHUB_USER "${b:=$1}" && git branch -u $GITHUB_USER/${b:=$1}
  fi
}
ggp! () {
  if [[ "$#" != 0 ]] && [[ "$#" != 1 ]]; then
    git push origin "${*}" --force
  else
    [[ "$#" = 0 ]] && local b="$(git_current_branch)"
    git push origin "${b:=$1}" --force
  fi
}
gmp! () {
  if [[ "$#" != 0 ]] && [[ "$#" != 1 ]]; then
    git push $GITHUB_USER "${*}" --force
  else
    [[ "$#" = 0 ]] && local b="$(git_current_branch)"
    git push $GITHUB_USER "${b:=$1}" --force
  fi
}
alias ggpush='git push origin $(current_branch) && git push -u origin/$(git_current_branch)'
alias ggpush!='git push origin $(current_branch) --force'
alias gmpush='git push $GITHUB_USER $(current_branch) && git branch -u $GITHUB_USER/$(git_current_branch)'
alias gmpush!='git push $GITHUB_USER $(current_branch) --force'

_is_option () {
  if [[ $1 =~ "^-.*" ]]; then
    return 0
  fi
  return 1
}

grbu () {
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
  git rebase upstream/$branch $opts
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
  compdef _git grbu=git-checkout 2>/dev/null
fi
alias grbgi='grbg --interactive'
alias grbui='grbu --interactive'
if which compdef &>/dev/null; then
  compdef _git grbgi=git-checkout 2>/dev/null
  compdef _git grbui=git-checkout 2>/dev/null
fi

git_commit_m () {
  git commit -m "$*"
}
alias gcmsg='git_commit_m'
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
gbdg () {
  local branch
  if [ $# -eq 0 ]; then
    branch=master
  else
    branch=$1
  fi
  git branch -r --merged origin/master | grep "$GITHUB_USER\\/" | sed "s/$GITHUB_USER\\///" | egrep -v "HEAD|master|develop|release|${branch}" | xargs git push $GITHUB_USER --delete
}
gbdu () {
  local branch
  if [ $# -eq 0 ]; then
    branch=master
  else
    branch=$1
  fi
  git branch -r --merged upstream/$branch | grep "$GITHUB_USER\\/" | sed "s/$GITHUB_USER\\///" | egrep -v "HEAD|master|develop|release|${branch}" | xargs git push $GITHUB_USER --delete
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

alias gsui='git submodule update --init'
alias gsuir='gsui --recursive'

git_checkout_by_fzy() {
  if [ $# -eq 0 ]; then
    branch=$(git branch -a | grep -v '*' | grep -v 'HEAD' | awk '{print $1}' | sed 's,^remotes/,,g' | fzy)
    git checkout $branch
  else
    git checkout $*
  fi
}
alias gco=git_checkout_by_fzy
if which compdef &>/dev/null; then
  compdef _git git_checkout_by_fzy=git-checkout 2>/dev/null
fi

gap='git add -p'
