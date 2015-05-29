#!/bin/sh

zmodload zsh/datetime

function _current_epoch() {
  echo $(( $EPOCHSECONDS / 60 / 60 / 24 ))
}

function _update_dotfiles_update() {
  echo "LAST_EPOCH=$(_current_epoch)" >! ~/.dotfiles-update
}

function _upgrade_dotfiles() {
  /bin/sh .dotfiles/upgrade_dotfiles.sh
  # update the dotfiles file
  _update_dotfiles_update
}

epoch_target=$UPDATE_DOTFILES_DAYS
if [[ -z "$epoch_target" ]]; then
  # Default to old behavior
  epoch_target=13
fi

if [ -f ~/.dotfiles-update ]
then
  . ~/.dotfiles-update

  if [[ -z "$LAST_EPOCH" ]]; then
    _update_dotfiles_update && return 0;
  fi

  epoch_diff=$(($(_current_epoch) - $LAST_EPOCH))
  if [ $epoch_diff -gt $epoch_target ]
  then
    if [ "$DISABLE_UPDATE_PROMPT" = "true" ]
    then
      _upgrade_dotfiles
    else
      echo "[dotfiles] Would you like to check for updates?"
      echo "Type Y to update dotfiles: \c"
      read line
      if [ "$line" = Y ] || [ "$line" = y ]; then
        _upgrade_dotfiles
      else
        _update_dotfiles_update
      fi
    fi
  fi
else
  # create the dotfiles file
  _update_dotfiles_update
fi

