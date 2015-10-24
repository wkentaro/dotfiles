#!/bin/sh

browse () {
  local url os
  [ "$1" = "" ] && return 1
  url=$(python -c "\
import re
url = '$1'
m = re.match('^https?://.*', url)
if m is None:
  url = 'http://' + url
print(url)
" 2>&1)
  os=$(uname)
  if [ "$os" = "Linux" ]; then
    gnome-open $url 1>/dev/null 
  elif [ "$os" = "Darwin" ]; then
    open $url 1>/dev/null
  fi
}
alias b='browse'