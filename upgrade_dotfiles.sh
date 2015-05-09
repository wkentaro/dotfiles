printf '\033[0;34m%s\033[0m\n' "Upgrading DOTFILES"
cd ~/.dotfiles
if git pull --rebase --stat origin master
then
  printf '\033[0;32m%s\033[0m\n' '       __        __   ____ _  __            '
  printf '\033[0;32m%s\033[0m\n' '  ____/ /____   / /_ / __/(_)/ /___   _____ '
  printf '\033[0;32m%s\033[0m\n' ' / __  // __ \ / __// /_ / // // _ \ / ___/ '
  printf '\033[0;32m%s\033[0m\n' '/ /_/ // /_/ // /_ / __// // //  __/(__  )  '
  printf '\033[0;32m%s\033[0m\n' '\__,_/ \____/ \__//_/  /_//_/ \___//____/   '
  printf '\033[0;32m%s\033[0m\n' '                                            '
  printf '\033[0;34m%s\033[0m\n' 'Hooray! DOTFILES has been updated and/or is at the current version.'
else
  printf '\033[0;31m%s\033[0m\n' 'There was an error updating. Try again later?'
fi
/bin/sh ~/.vim/upgrade_vim_bundles.sh
