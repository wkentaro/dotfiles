#!/usr/bin/env zsh

setup() {
  source $HERE/.anaconda2/bin/activate
  # . $HERE/venv/bin/activate
  # activate mvtk
}

HERE=${0:a:h}
setup