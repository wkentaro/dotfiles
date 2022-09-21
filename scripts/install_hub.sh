#!/bin/sh

if [ -f $HOME/.local/bin/hub ]; then
  exit 0
elif [ "$(uname)" = "Darwin" ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
VERSION='2.14.2'

cd $TMPDIR
URL="https://github.com/github/hub/releases/download/v$VERSION/hub-linux-amd64-$VERSION.tgz"
wget -q $URL -O hub-linux-amd64-$VERSION.tgz
tar zxf hub-linux-amd64-$VERSION.tgz

cd hub-linux-amd64-$VERSION

mkdir -p $HOME/.local/bin
cp bin/hub $HOME/.local/bin/hub

mkdir -p $HOME/.bash_completion.d
cp etc/hub.bash_completion.sh $HOME/.bash_completion.d/hub.bash_completion.sh

mkdir -p ~/.zsh/completions
cp etc/hub.zsh_completion ~/.zsh/completions/_hub

mkdir -p $HOME/.local/share/man/man1
cp share/man/man1/hub.1 $HOME/.local/share/man/man1/hub.1

rm -rf $TMPDIR
