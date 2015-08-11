#!/usr/bin/env zsh

demo_mode () {
  local shl prpt
  shl="$1"
  if [ "$shl" = "" ]; then
    shl="zsh"
  fi
  case "$shl" in
    (bash)
      prpt='$ ' ;;
    (zsh)
      prpt='%# ' ;;
    *) ;;
  esac
  _OLD_PROMPT=$PROMPT
  PROMPT=$prpt
}

exit_demo_mode() {
  PROMPT=$_OLD_PROMPT
}