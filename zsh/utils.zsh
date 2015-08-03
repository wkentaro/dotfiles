#!/usr/bin/env zsh


wstool-foreach () {
  local -a locations
  local e
  locations=( $(wstool info --only=path) )
  for e in $locations; do
    cd $e
    eval $@
  done
}

b () {
  local url
  [ "$1" = "" ] && return 1
  url=$(python -c "\
import re
url = '$1'
m = re.match('^https?://.*', url)
if m is None:
  url = 'http://' + url
print(url)
" 2>&1)
  open $url
}

workspace () {
  local ws
  ws=$(cat ~/.ws_list | xargs -n1 | percol)
  [ "$ws" = "" ] && return 1
  ws=$(echo $ws | sed "s,^~,$HOME,")
  cd $ws
}

register_workspace () {
  echo $(pwd | sed "s,^$HOME,~,") >> ~/.ws_list
}

unregister_workspace () {
  local ws
  ws=$(cat ~/.ws_list | xargs -n1 | percol)
  [ "$ws" = "" ] && return 1
  out=$(echo $ws | sed 's,/,\\/,g' | sed 's,^\~,\\~,')
  sed -i -e "/$out/d" ~/.ws_list
  echo "$ws is removed"
}

wshub () {
  # check args validity
  if [ ! $# -eq 2 ]; then
    echo "Too few arguments" >&2
    return 1
  fi
  # execute command
  local cmd localname
  cmd=$1
  uri=$2
  localname=$2
  case $1 in
    clone)
      wstool set --git $localname "https://github.com/$uri.git" -y >/dev/null || return 1
      python -c "\
from pprint import pprint
import yaml
config = yaml.load('''`wstool info --yaml $localname 2>/dev/null`''')[0]
pprint(config)
" 2>/dev/null
      wstool update $localname
    ;;
    rm|remove)
      wstool remove $localname
      rm -rf $localname
    ;;
    *)
      echo "$localname Didn't match anything"
    ;;
  esac
}