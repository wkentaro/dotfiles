printf '\033[0;34m%s\033[0m\n' "Upgrading VIM"
if vim -e -c "NeoBundleUpdate | visual | quit"
then
  printf '\033[0;32m%s\033[0m\n' '                           '
  printf '\033[0;32m%s\033[0m\n' '   __    __  __  ____ ___  '
  printf '\033[0;32m%s\033[0m\n' '   | |  / / / / / __ `__ \ '
  printf '\033[0;32m%s\033[0m\n' '   | |_/ / / / / / / / / / '
  printf '\033[0;32m%s\033[0m\n' '    \___/ /_/ /_/ /_/ /_/  '
  printf '\033[0;32m%s\033[0m\n' '                           '
  printf '\033[0;34m%s\033[0m\n' 'Hooray! VIM has been updated and/or is at the current version.'
else
  printf '\033[0;31m%s\033[0m\n' 'There was an error updating. Try again later?'
fi
