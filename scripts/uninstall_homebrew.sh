#!/bin/sh
#
brew install wget
brew cleanup
wget https://gist.githubusercontent.com/mxcl/1173223/raw/a833ba44e7be8428d877e58640720ff43c59dbad/uninstall_homebrew.sh
bash uninstall_homebrew.sh
rm uninstall_homebrew.sh
rm -rf /usr/local/Cellar /usr/local/.git
rm -rf /usr/local/Library/Taps
