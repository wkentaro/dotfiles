#!/bin/sh

if [ -f /usr/local/bin/hub ]; then
  exit 0
elif [ "$(uname)" = "Darwin" ]; then
  exit 0
fi

sudo mkdir -p /usr/local/share/man/man1

TMPDIR=$(mktemp -d)
VERSION='2.3.0-pre10'

cd $TMPDIR
URL="https://github.com/github/hub/releases/download/v$VERSION/hub-linux-amd64-$VERSION.tgz"
wget -q $URL -O hub-linux-amd64-$VERSION.tgz
tar zxf hub-linux-amd64-$VERSION.tgz

sudo cp hub-linux-amd64-$VERSION/bin/hub /usr/local/bin/hub
sudo cp hub-linux-amd64-$VERSION/etc/hub.bash_completion.sh /etc/bash_completion.d/hub.bash_completion.sh
sudo cp hub-linux-amd64-$VERSION/etc/hub.zsh_completion /usr/local/share/zsh/site-functions/_hub
sudo cp hub-linux-amd64-$VERSION/share/man/man1/hub.1 /usr/local/share/man/man1/hub.1

rm -rf $TMPDIR
