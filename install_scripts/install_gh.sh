if [ -e /usr/bin/gh ]; then
  echo "gh is installed"
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

VERSION=2.0.0
wget https://github.com/cli/cli/releases/download/v${VERSION}/gh_${VERSION}_linux_amd64.deb
sudo dpkg -i gh_${VERSION}_linux_amd64.deb

cd -
rm -rf $TMPDIR
