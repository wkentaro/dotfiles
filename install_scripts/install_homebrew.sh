if [ "`uname`" != "Darwin" ]; then
  exit 0
elif which brew &>/dev/null; then
  exit 0
fi

ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
cat <<-EOF
Please add below in your shell config file.
  PATH=/usr/local/bin:$PATH
EOF
brew doctor
brew update
