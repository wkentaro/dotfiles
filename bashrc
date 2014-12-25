# encoding
export LC_CTYPE='en_US.UTF-8'
# terminal color
export TERM=xterm-256color
# prompt setup
current_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'
}
parse_branch() {
    git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/(\1)/'
}
PS1='${debian_chroot:+($debian_chroot)}\[\e[00;32m\]\u@\h:\[\e[01;34m\]\W\[\033[01;35m\]$(parse_branch)\[\e[01;35m\]\[\e[0m\] $ '

# Set aliases
source ~/.bash-alias

if [ `uname` = 'Darwin' ]; then
  echo ""
  # source /usr/local/share/zsh/site-functions
else
  if [ -f /opt/ros/hydro/setup.bash ]; then
    source ~/.bashrc.ros
    function enshu () {
        cd ~/catkin_ws/enshu
        source ./devel/setup.bash
    }
    function soft () {
        cd ~/catkin_ws/soft3
        source ./devel/setup.bash
    }
    function semi () {
        cd ~/catkin_ws/semi
        source ./devel/setup.bash
    }
  fi
fi

function google () {
    search=""
    echo "Googling: $@"
    for term in $@; do
        search="$search%20$term"
    done
    xdg-open "http://www.google.com/search?q=$search"
}
