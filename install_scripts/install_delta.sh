#!/bin/sh

if [ -e ~/.local/bin/delta ]; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

if [ "$(uname)" = "Linux" ]; then
  curl -L -O https://github.com/dandavison/delta/releases/download/0.11.3/delta-0.11.3-x86_64-unknown-linux-musl.tar.gz
elif [ "$(uname)" = "Darwin" ]; then
  curl -L -O https://github.com/dandavison/delta/releases/download/0.11.3/delta-0.11.3-x86_64-apple-darwin.tar.gz
fi

tar zxvf *.tar.gz
mv **/delta ~/.local/bin/delta

cd -
rm -rf $TMPDIR
