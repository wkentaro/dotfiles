#!/usr/bin/env zsh

setup() {
  . $_autoenv_this_dir/venv/bin/activate
  # anaconda_activate
}

_autoenv_this_dir=${0:a:h}
setup