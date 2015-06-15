echo """6,8c6,9
< emulate sh # emulate POSIX
< . "$_CATKIN_SETUP_DIR/setup.sh"
< emulate zsh # back to zsh mode
---
> # emulate sh # emulate POSIX
> # . "$_CATKIN_SETUP_DIR/setup.sh"
> # emulate zsh # back to zsh mode
> emulate -R zsh -c 'source "$_CATKIN_SETUP_DIR/setup.sh"'""" \
> /tmp/hydro_setup_zsh.diff

sudo patch $1 < /tmp/hydro_setup_zsh.diff
