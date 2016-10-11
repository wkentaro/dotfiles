# vim: set ft=sh:

# basic
alias sudo='sudo '
alias cl='clear'
alias lv='less'
if which pygmentize &>/dev/null; then
  export LESSOPEN="|$HOME/.lessfilter %s"
fi

# date
alias datestamp='date +"%Y%m%d"'
alias timestamp='date +"%Y%m%d_%H%M%S"'

# mv
alias mvi='mv -i'

# rm
alias rmi='rm -i'

# open
type gnome-open &>/dev/null && alias open=gnome-open
alias o='open'
alias o.='open .'

# browsing
alias gcal='open https://www.google.com/calendar/render#g >/dev/null 2>&1'
alias gmail='open https://mail.google.com/mail/u/0/ >/dev/null 2>&1'

# vim
type vim &>/dev/null && {
  alias vi='vim'
  alias vii='vim --noplugin'
  alias viii='vim -u NONE'
}
type nvim &>/dev/null && {
  alias nvi='nvim'
  alias nvii='nvim --noplugin'
  alias nviii='nvim -u NONE'
}
alias vim-euc='vim -c ":e ++enc=euc-jp"'
alias vim-iso='vim -c ":e ++enc=iso-2022-jp"'

# emacs
alias emacs='emacs -nw'

# python
alias py='python'
alias ipy='ipython'
alias ipp='ptipython'
alias fl='flake8'

# ruby
alias irb='irb --simple-prompt'

# cmatrix
alias matrix='cmatrix -sb'

# tmux
alias t='tmux'
alias tls='tmux ls'
tmux_percol_attach() {
    if [[ $1 == "" ]]; then
        PERCOL=percol
    else
        PERCOL="percol --query $1"
    fi

    sessions=$(tmux ls)
    [ $? -ne 0 ] && return

    if [ $(echo $sessions | wc -l) -eq 1 ]; then
      tmux attach && return
    fi

    session=$(echo $sessions | eval $PERCOL | cut -d : -f 1)
    if [[ -n "$session" ]]; then
        tmux attach -t $session
    fi
}
alias ta='tmux_percol_attach'
alias tn='tmux new'
alias tns='tmux new -s'

# gifify
gifify () {
  docker run -it --rm -v `pwd`:/data maxogden/gifify $@
}

# brew
if type brew &>/dev/null; then
  alias bubu='brew update && brew upgrade && brew cleanup'
  alias bububu='bubu && brew cask update && brew cask cleanup'
fi

# ----------------------------------------------------
# pandoc
# ----------------------------------------------------
md2rst () {
  pandoc --from=markdown --to=rst $1
}
rst2md () {
  pandoc --from=rst --to=markdown $1
}

# ----------------------------------------------------
# wrapping with rlwrap
# ----------------------------------------------------
if type rlwrap &>/dev/null; then
  alias eus='rlwrap eus'
  alias irteusgl='rlwrap irteusgl'
  alias irb='rlwrap irb'
  alias clisp="rlwrap -b '(){}[],#\";| ' clisp"
  if [ "$EMACS" = "" ]; then
    alias roseus="rlwrap -c -b '(){}.,;|' -a -pGREEN roseus"
  fi
fi

# ----------------------------------------------------
# ls aliases
# ----------------------------------------------------
alias sl='ls'
if type dircolors &>/dev/null; then
  [ -f $HOME/.dircolors.256dark ] && eval $(dircolors $HOME/.dircolors.256dark 2>/dev/null)
fi
if command ls --color &>/dev/null; then
  # GNU ls
  if [ $TERM = "dumb" ]; then
    # Disable colors in GVim
    alias ls='ls -F --show-control-chars'
    alias la='ls -ahF --show-control-chars'
    alias ll='ls -lhF --show-control-chars'
    alias lsa='ls -lahF --show-control-chars'
  else
    # Color settings for zsh complete candidates
    alias ls='ls -F --show-control-chars --color=always'
    alias la='ls -ahF --show-control-chars --color=always'
    alias ll='ls -lhF --show-control-chars --color=always'
    alias lsa='ls -lahF --show-control-chars --color=always'
  fi
else
  eval $(gdircolors $HOME/.dircolors.256dark 2>/dev/null)
  alias ls='gls -F --show-control-chars --color=always'
  alias la='gls -ahF --show-control-chars --color=always'
  alias ll='gls -lhF --show-control-chars --color=always'
  alias lsa='gls -lahF --show-control-chars --color=always'
fi

if hash gls &>/dev/null; then
  alias sleep=gsleep
fi

convert_to_gif () {
  if which ffmpeg &>/dev/null; then
    ffmpeg -i $1 -pix_fmt rgb8 -r 10 -f gif - | gifsicle --optimize=3 --delay=3
  elif which avconv &>/dev/null; then
    avconv -i $1 -pix_fmt rgb24 -r 10 -f gif - | gifsicle --optimize=3 --delay=3
  fi
}

get_lena_jpg () {
  wget https://jviolajones.googlecode.com/files/lena.jpg
}
tile_images() {
  montage $@ -geometry +2+2 $(date +%Y%m%d-%H%M%S)_output.jpg
}

ghcomment () {
  number=$(ghi list | percol | awk '{print $1}')
  ghi comment $number --list C
  ghi comment $number --verbose $@
}
ghlist () {
  ghi show $(ghi list | percol | awk '{print $1}')
}

trres () {
  list=$(ghi list -p)
  repo_slug=$(echo $list | sed -n 1p | awk '{print $2}')
  pr_num=$(echo $list | sed -n '2,$p' | percol | awk '{print $1}')
  build_num=$(travis history -R $repo_slug | grep "(PR #$pr_num)" | sed -n 1p | awk '{print $1}' | sed 's/^#//g')
  build_nums=$(travis show $build_num | percol | awk '{print $1}' | sed 's/^#//')
  for b in ${=build_nums}; do
    eval "restart_travis $repo_slug $b"
  done
}

trlog () {
  list=$(ghi list -p)
  repo_slug=$(echo $list | sed -n 1p | awk '{print $2}')
  pr_num=$(echo $list | sed -n '2,$p' | percol | awk '{print $1}')
  build_num=$(travis history -R $repo_slug | grep "(PR #$pr_num)" | sed -n 1p | awk '{print $1}' | sed 's/^#//g')
  build_num=$(travis show $build_num | percol | awk '{print $1}' | sed 's/^#//')
  travis logs $build_num
}


startbitbucket () {
    echo 'Username?'
    read username
    echo 'Password?'
    read -s password  # -s flag hides password text
    echo 'Repo name?'
    read reponame

    curl --user $username:$password https://api.bitbucket.org/1.0/repositories/ --data name=$reponame --data is_private='true'
    git remote add origin git@bitbucket.org:$username/$reponame.git
    git push -u origin --all
    git push -u origin --tags
}


slacker_notify_done () {
  "$@"
  local retcode=$?
  echo "@wkentaro '$@' is done at '$(date)' with exitcode '${retcode}'" | slacker -u wkentaro
}


# ----------------------------------------------------
# Show Setup
# ----------------------------------------------------
show_python () {
  echo "PYTHON_EXECUTABLE: $(command which python)"
}

show_ros () {
  CATKIN_TOOLS_VERSION=$(python -c "import pkg_resources; print(pkg_resources.get_distribution('catkin-tools').version)" 2>/dev/null)
  echo "ROS_DISTRO: $ROS_DISTRO"
  # echo "CATKIN_TOOLS_VERSION: $CATKIN_TOOLS_VERSION"
  echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
}

show_cuda () {
  # cuda
  CUDA_VERSION=$(command nvcc --version | sed -n 4p | sed 's/.*, release .*, V\(.*\)/\1/')
  echo "CUDA_VERSION: $CUDA_VERSION"
  # cudnn
  if [ -e $CUDA_HOME/include/cudnn.h ]; then
    CUDNN_MAJOR=$(cat $CUDA_HOME/include/cudnn.h | grep '#define CUDNN_MAJOR' | awk '{print $3}')
    CUDNN_MINOR=$(cat $CUDA_HOME/include/cudnn.h | grep '#define CUDNN_MINOR' | awk '{print $3}')
    CUDNN_PATCHLEVEL=$(cat $CUDA_HOME/include/cudnn.h | grep '#define CUDNN_PATCHLEVEL' | awk '{print $3}')
    CUDNN_VERSION="$CUDNN_MAJOR.$CUDNN_MINOR.$CUDNN_PATCHLEVEL"
    echo "CUDNN_VERSION: $CUDNN_VERSION"
  fi
}

show_dnn () {
  show_cuda
  # chainer
  CHAINER_VERSION=$(python -c "import pkg_resources; print(pkg_resources.get_distribution('chainer').version)" 2>/dev/null)
  if [ ! -z $CHAINER_VERSION ]; then
    echo "CHAINER_VERSION: $CHAINER_VERSION"
  fi
  # tensorflow
  TENSORFLOW_VERSION=$(python -c "import pkg_resources; print(pkg_resources.get_distribution('tensorflow').version)" 2>/dev/null)
  if [ ! -z $TENSORFLOW_VERSION ]; then
    echo "TENSORFLOW_VERSION: $TENSORFLOW_VERSION"
  fi
}


init_autoenv () {
  vim .autoenv.zsh
  vim .autoenv_leave.zsh
}
