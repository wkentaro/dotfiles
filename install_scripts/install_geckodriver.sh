#!/bin/bash

if [ -e $HOME/.local/bin/geckodriver ]; then
  exit 0
fi

set -x

if [ "$(uname)" = "Linux" ]; then
  URL='https://github.com/mozilla/geckodriver/releases/download/v0.19.1/geckodriver-v0.19.1-linux64.tar.gz'
elif [ "$(uname)" = "Darwin" ]; then
  URL='https://github.com/mozilla/geckodriver/releases/download/v0.19.1/geckodriver-v0.19.1-macos.tar.gz'
fi

TMP_DIR=$(mktemp -d)

cd $TMP_DIR
if which wget &>/dev/null; then
  wget $URL -O geckodriver.tar.gz
else
  curl -L $URL -o geckodriver.tar.gz
fi
tar zxvf geckodriver.tar.gz
mv geckodriver $HOME/.local/bin/geckodriver
cd -

rm -rf $TMP_DIR
