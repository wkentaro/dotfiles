#!/bin/sh

browse () {
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
alias b='browse'