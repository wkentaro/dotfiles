#!/bin/bash

set -x

for server in localhost jenkins jhoop jdlbox1 jdlbox2 jdlbox3 jdlbox4 jdlbox5 crux jbaxter-c1; do
  timeout 3 ssh $server true && \
    ssh $server '(cd .dotfiles && git pull origin master && zsh -c "source ~/.zshrc && ./install.py")' &
done

wait

set +x
