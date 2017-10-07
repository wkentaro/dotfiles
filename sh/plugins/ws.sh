#!/bin/sh
#

ws () {
  local ws
  ws=$(cat ~/.ws_list | xargs -n1 | fzy)
  [ "$ws" = "" ] && return 1
  ws=$(echo $ws | sed "s,^~,$HOME,")
  cd $ws
}

register_ws () {
  echo $(pwd | sed "s,^$HOME,~,") >> ~/.ws_list
}

unregister_ws () {
  local ws
  ws=$(cat ~/.ws_list | xargs -n1 | fzy)
  [ "$ws" = "" ] && return 1
  out=$(echo $ws | sed 's,/,\\/,g' | sed 's,^\~,\\~,')
  sed -i -e "/$out/d" ~/.ws_list
  echo "$ws is removed"
}