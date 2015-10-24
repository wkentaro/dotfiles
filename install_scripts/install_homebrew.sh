#!/bin/sh

# Homebrew
# ~~~~~~~~


if [ "`uname`" != "Darwin" ]; then
  echo "Homebrew should be installed only on Darwin"
  exit 1
elif which brew &>/dev/null; then
  echo "Homebrew is already installed"
  exit 1
fi

ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
cat <<-EOF
Please add below in your shell config file.
  PATH=/usr/local/bin:$PATH
EOF
brew doctor
brew update
