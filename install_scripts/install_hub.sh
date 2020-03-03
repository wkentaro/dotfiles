#!/bin/sh

if [ -f /usr/local/bin/hub ]; then
  exit 0
elif [ "$(uname)" = "Darwin" ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
VERSION='2.5.1'

cd $TMPDIR
URL="https://github.com/github/hub/releases/download/v$VERSION/hub-linux-amd64-$VERSION.tgz"
wget -q $URL -O hub-linux-amd64-$VERSION.tgz
tar zxf hub-linux-amd64-$VERSION.tgz

mkdir -p $HOME/.local/bin
cp hub-linux-amd64-$VERSION/bin/hub $HOME/.local/bin/hub

mkdir -p $HOME/.bash_completion.d
cp hub-linux-amd64-$VERSION/etc/hub.bash_completion.sh $HOME/.bash_completion.d/hub.bash_completion.sh

# installed as oh-my-zsh plugin
# mkdir -p $HOME/.zsh/oh-my-zsh/completions
# cp hub-linux-amd64-$VERSION/etc/hub.zsh_completion $HOME/.zsh/oh-my-zsh/completions/_hub

mkdir -p $HOME/.local/share/man/man1
cp hub-linux-amd64-$VERSION/share/man/man1/hub.1 $HOME/.local/share/man/man1/hub.1

rm -rf $TMPDIR
