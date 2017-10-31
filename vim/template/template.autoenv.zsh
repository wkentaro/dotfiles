#!/usr/bin/env zsh

setup() {
  . $HERE/venv/bin/activate
  # activate mvtk
}

HERE=${0:a:h}
setup