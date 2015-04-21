#!/usr/bin/env zsh

zmodload zsh/datetime

function _current_epoch() {
  echo $(( $EPOCHSECONDS / 60 / 60 / 24 ))
}

function _update_vim_update() {
  echo "LAST_EPOCH=$(_current_epoch)" >! ~/.vim-update
}

function _upgrade_vim() {
  /bin/sh .vim/upgrade.sh
  # update the vim file
  _update_vim_update
}

epoch_target=$UPDATE_VIM_DAYS
if [[ -z "$epoch_target" ]]; then
  # Default to old behavior
  epoch_target=13
fi

if [ -f ~/.vim-update ]
then
  . ~/.vim-update

  if [[ -z "$LAST_EPOCH" ]]; then
    _update_vim_update && return 0;
  fi

  epoch_diff=$(($(_current_epoch) - $LAST_EPOCH))
  if [ $epoch_diff -gt $epoch_target ]
  then
    if [ "$DISABLE_UPDATE_PROMPT" = "true" ]
    then
      _upgrade_vim
    else
      echo "[VIM] Would you like to check for updates?"
      echo "Type Y to update vim: \c"
      read line
      if [ "$line" = Y ] || [ "$line" = y ]; then
        _upgrade_vim
      else
        _update_vim_update
      fi
    fi
  fi
else
  # create the vim file
  _update_vim_update
fi

