#!/bin/sh
#

ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
echo '''

Please add below in your shell config file.
  PATH=/usr/local/bin:$PATH

'''
brew doctor
brew update

