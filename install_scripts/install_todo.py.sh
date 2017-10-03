#!/bin/bash

if [ -f ~/.local/bin/todo ]; then
  echo "[$(basename $0)] todo is already installed."
  exit 0
fi

set -x

mkdir -p ~/.local/share
cd ~/.local/share
git clone https://github.com/wkentaro/todo.py.git
ln -s $(pwd)/todo.py/todo.py ~/.local/bin/todo

set +x
