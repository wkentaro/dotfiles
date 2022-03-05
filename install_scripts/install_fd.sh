if which fdfind &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)

cd $TMPDIR
VERSION=8.3.2
wget -q https://github.com/sharkdp/fd/releases/download/v${VERSION}/fd_${VERSION}_amd64.deb
sudo dpkg -i fd_${VERSION}_amd64.deb
