#!/bin/bash

od="$HOME/.odrive-agent/bin"
if [ ! -e $HOME/.local/bin/odrive ]; then
  curl -L "http://dl.odrive.com/odrive-py" --create-dirs -o "$od/odrive.py"
  curl -L "http://dl.odrive.com/odriveagent-lnx-64" | tar -xvzf- -C "$od/"
  curl -L "http://dl.odrive.com/odrivecli-lnx-64" | tar -xvzf- -C "$od/"
  wget https://raw.githubusercontent.com/amagliul/odrive-utilities/master/odrivecli.py -O "$od/odrivecli.py"
  chmod u+x "$od/odrivecli.py"
  ln -sf $od/odrivecli.py $HOME/.local/bin/odrive
fi
