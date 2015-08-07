#!/usr/bin/env zsh

demo_mode () {
  _OLD_PROMPT=$PROMPT
  PROMPT='%# '
}

exit_demo_mode() {
  PROMPT=$_OLD_PROMPT
}