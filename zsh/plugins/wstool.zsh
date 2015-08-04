#!/usr/bin/env zsh

wstool_foreach () {
  local -a locations
  local e
  locations=( $(wstool info --only=path) )
  for e in $locations; do
    (cd $e && eval $@)
  done
}

wstool_info () {
  local repo target_ws
  local -a options
  for arg in ${@}; do
    if [ "${arg:0:1}" = "-" ]; then
      options=($options $arg)
    elif [ "${options[-1]}" = "-t" -o \
           "${options[-1]}" = "--target-workspace" ]; then
      options=($options $arg)
    elif [ "$repo" = "" ]; then
      repo=$arg
    else
      command wstool info $@
      return
    fi
  done
  if [ "$repo" != "" ]; then
    local -a repos
    repos=( $(command wstool info --only=localname | grep $repo) )
    [ ${#repos[@]} -gt 0 ] && {
      command wstool info $repos $options
      return
    }
  fi
  command wstool info $@
}

wstool_update () {
  local repo
  local -a options
  for arg in ${@}; do
    if [ "${arg:0:1}" = "-" ]; then
      options=($options $arg)
    elif [ "$repo" = "" ]; then
      repo=$arg
    else
      command wstool update $@
      return
    fi
  done
  if [ "$repo" != "" ]; then
    local -a repos
    repos=( $(command wstool info --only=localname | grep $repo) )
    [ ${#repos[@]} -gt 0 ] && {
      command wstool update $repos $options
      return
    }
  fi
  command wstool update $@
}

wstool_set () {
  local repo uri do_update
  local -a options
  for arg in ${@}; do
    if [ "${arg:0:1}" = "-" ]; then
      options=($options $arg)
    elif [ "$repo" = "" ]; then
      repo=$arg
    elif [ "$uri" = "" ]; then
      uri=$arg
    fi
  done
  # resolve args
  if echo $uri | egrep -q "\.git$"; then
    [ "${options[(r)--git]}" = --git ] || options=($options --git)
  fi
  # --hub option
  if [ "${options[(r)--hub]}" = "--hub" ]; then
    echo $uri | egrep -q "^https?://" || {
      # --ssh option
      if [ "${options[(r)--ssh]}" = --ssh ]; then
        uri="git@github.com:$uri.git"
        options[$options[(i)--ssh]]=()
      else
        uri="https://github.com/$uri.git"
      fi
    }
    [ "${options[(r)--git]}" = --git ] || options=($options --git)
    options[$options[(i)--hub]]=()
  fi
  # --update option
  [ "${options[(r)--update]}" = "--update" ] && {
    do_update=true
    options[$options[(i)--update]]=()
  }
  if [ "$uri" != "" ]; then
    command wstool set $repo $uri $options
    [ $do_update ] && command wstool update $repo
    return
  fi
  command wstool set $@
}

yes_or_no () {
  if [ "$1" = "" ]; then
    read REPLY\?'Are you sure? [y/n]:'
  else
    read REPLY\?"$1 [y/n]:"
  fi
  if echo $REPLY | egrep -q "^[Yy]$" ; then
    return 0
  else
    return 1
  fi
}

wstool_remove () {
  local repo do_clean
  local -a options
  for arg in ${@}; do
    if [ "${arg:0:1}" = "-" ]; then
      options=($options $arg)
    elif [ "$repo" = "" ]; then
      repo=$arg
    fi
  done
  # --clean option
  [ "${options[(r)--clean]}" = "--clean" ] && do_clean=true
  options[$options[(i)--clean]]=()
  command wstool remove $repo $options && [ $do_clean ] && [ $(yes_or_no "Remove $repo?") ] && rm -rf $repo && echo "Removed ['$repo']"
}

wstool () {
  case "$1" in
    (update|up)
      shift; wstool_update $@
      ;;
    (set)
      shift; wstool_set $@
      ;;
    (info)
      shift; wstool_info $@
      ;;
    (foreach)
      shift; wstool_foreach $@
      ;;
    (remove|rm)
      shift; wstool_remove $@
      ;;
    (cd)
      shift; wstool_cd $@
      ;;
    (*)
      command wstool $@
      ;;
  esac
}

fpath=($(dirname ${funcsourcetrace[1]%:*}) $fpath)
