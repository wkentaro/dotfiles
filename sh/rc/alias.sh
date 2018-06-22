# vim: set ft=sh:

# basic
alias sudo='sudo '
alias cl='clear'
alias lv='less'
if which pygmentize &>/dev/null; then
  export LESSOPEN="|$HOME/.lessfilter %s"
fi

alias rm='rm -i'
alias rmr='rm -r'
alias mv='mv -i'
alias cp='cp -i'

# date
alias datestamp='date +"%Y%m%d"'
alias timestamp='date +"%Y%m%d_%H%M%S"'

# mv
alias mvi='mv -i'

# rm
alias rmi='rm -i'
alias rmr='rm -r'

# open
type gnome-open &>/dev/null && alias open='gnome-open $@ 2>/dev/null'
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
alias vim-euc='vim -c ":e ++enc=euc-jp"'
alias vim-iso='vim -c ":e ++enc=iso-2022-jp"'

# emacs
alias emacs='emacs -nw'

# python
alias py='python'
alias ipy='ipython'
alias ipp='ptipython'
alias fl='flake8'

# tmux
# alias t='tmux'
# alias tls='tmux ls'
tls () {
  local user_id=$(id -u)
  if [ ! -d /tmp/tmux-$user_id ]; then
    return 1
  fi
  IFS=$'\n'
  for s in $(command ls /tmp/tmux-$user_id); do
    for t in $(tmux -S /tmp/tmux-$user_id/$s ls 2>/dev/null); do
      echo "[/tmp/tmux-$user_id/$s] $t"
    done
  done
}
tmux_fzy_attach() {
    if [[ $1 == "" ]]; then
        PERCOL=fzy
    else
        PERCOL="fzy --query $1"
    fi

    sessions=$(tls)
    [ $? -ne 0 -o "$sessions" = "" ] && return

    if [ $(echo "${sessions[*]}" | wc -l) -eq 1 ]; then
      # tmux attach && return
      socket=$(echo "${sessions[*]}" | awk '{print $1}' | sed -e 's/^\[\(.*\)\]$/\1/')
      session=$(echo "${sessions[*]}" | awk '{print $2}' | sed -e 's/^\(.*\):$/\1/')
      tmux -S $socket $TMUX_OPTIONS attach -t $session
      return 0
    fi

    session=$(echo "${sessions[*]}" | eval $PERCOL)
    if [[ -n "$session" ]]; then
      socket=$(echo $session | awk '{print $1}' | sed -e 's/^\[\(.*\)\]$/\1/')
      session=$(echo $session | awk '{print $2}' | sed -e 's/^\(.*\):$/\1/')
      tmux -S $socket $TMUX_OPTIONS attach -t $session
      return 0
    fi
}
alias ta='tmux_fzy_attach'
alias tca="TMUX_OPTIONS='-CC' tmux_fzy_attach"
alias tcns="TMUX_OPTIONS='-CC' tns"
alias tn='tmux new'
tns () {
  if [ $# -eq 0 ]; then
    echo 'Please specify session name.'
  fi
  tmux -L $1 $TMUX_OPTIONS new -s $1
}

# brew
if type brew &>/dev/null; then
  alias bubu='brew update && brew upgrade && brew cleanup'
  alias bububu='bubu && brew cask cleanup'
  alias bbb='bububu'
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
    alias ls='ls --show-control-chars'
    alias la='ls -ah --show-control-chars'
    alias ll='ls -lh --show-control-chars'
    alias lsa='ls -lah --show-control-chars'
  else
    # Color settings for zsh complete candidates
    alias ls='ls --show-control-chars --color=always'
    alias la='ls -ah --show-control-chars --color=always'
    alias ll='ls -lh --show-control-chars --color=always'
    alias lsa='ls -lah --show-control-chars --color=always'
  fi
elif which gls &>/dev/null; then
  eval $(gdircolors $HOME/.dircolors.256dark 2>/dev/null)
  alias ls='gls --show-control-chars --color=always'
  alias la='gls -ah --show-control-chars --color=always'
  alias ll='gls -lh --show-control-chars --color=always'
  alias lsa='gls -lah --show-control-chars --color=always'
fi

# ssh
# alias ssh='ssh -C -X'
# alias ssh='ssh -c arcfour'

if hash gls &>/dev/null; then
  alias sleep=gsleep
fi

convert_to_gif () {
  filename="$1"
  basename="${filename%.*}"
  if which ffmpeg &>/dev/null; then
    ffmpeg -i $1 -pix_fmt rgb8 -r 10 -f gif - | gifsicle --optimize=3 --delay=3 > ${basename}.gif
  elif which avconv &>/dev/null; then
    avconv -i $1 -pix_fmt rgb24 -r 10 -f gif - | gifsicle --optimize=3 --delay=3 > ${basename}.gif
  fi
}

slacker_notify_done () {
  "$@"
  local retcode=$?
  echo "@wkentaro '$@' is done at '$(date)' with exitcode '${retcode}'" | slacker -u wkentaro
}


# ----------------------------------------------------
# Show Setup
# ----------------------------------------------------

show_ros () {
  CATKIN_TOOLS_VERSION=$(python -c "import pkg_resources; print(pkg_resources.get_distribution('catkin-tools').version)" 2>/dev/null)
  echo "ROS_DISTRO: $ROS_DISTRO"
  # echo "CATKIN_TOOLS_VERSION: $CATKIN_TOOLS_VERSION"
  echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
}

show_cuda () {
  which nvcc &>/dev/null || return 1
  # cuda
  CUDA_VERSION=$(command nvcc --version | sed -n 4p | sed 's/.*, release .*, V\(.*\)/\1/')
  echo "CUDA_VERSION: $CUDA_VERSION"
  # cudnn
  if [ -e $CUDNN_PATH/include/cudnn.h ]; then
    CUDNN_MAJOR=$(cat $CUDNN_PATH/include/cudnn.h | grep '#define CUDNN_MAJOR' | awk '{print $3}')

    CUDNN_MINOR=$(cat $CUDNN_PATH/include/cudnn.h | grep '#define CUDNN_MINOR' | awk '{print $3}')
    CUDNN_PATCHLEVEL=$(cat $CUDNN_PATH/include/cudnn.h | grep '#define CUDNN_PATCHLEVEL' | awk '{print $3}')
    CUDNN_VERSION="$CUDNN_MAJOR.$CUDNN_MINOR.$CUDNN_PATCHLEVEL"
    echo "CUDNN_VERSION: $CUDNN_VERSION"
  fi
}

watch_gpu () {
  watch -n1 --no-title '''
  echo "====================================================================================================="
  cuda-smi
  echo "====================================================================================================="
  echo
  if which nvidia-smi &>/dev/null; then
    nvidia-smi
  fi
  '''
}
alias nvid='nvidia-smi'
alias cud='cuda-smi'


init_autoenv () {
  vim .autoenv.zsh
  vim .autoenv_leave.zsh
}


macclean () {
  find . -type f -name '.DS_Store' -delete
}

compress_pdf () {
  if [ ! $# -eq 2 ]; then
    echo "Usage: compress_pdf INPUT_FILE OUTPUT_FILE"
  fi
  local input
  local output
  input=$1
  output=$2
  gs \
    -sDEVICE=pdfwrite \
    -dCompatibilityLevel=1.4 \
    -dPDFSETTINGS=/default \
    -dNOPAUSE \
    -dQUIET \
    -dBATCH \
    -dDetectDuplicateImages \
    -dCompressFonts=true \
    -r300 \
    -sOutputFile=${output} \
    ${input}
}

alias rsync_avt='rsync -avt'

alias rsync_rlt='rsync -rltDv'

cmake_prefix.. () {
  if [ $# != 1 ]; then
    return 1
  fi

  cmake -DCMAKE_INSTALL_PREFIX:PATH=$1 ..
}

alias cud='cuda-smi'
alias tailf='tail -n1000 -f'
psf () {
  if [ ! $# -eq 1 ]; then
    echo "Usage: $0 PATTERN"
    exit 1
  fi
  ps auxwww | grep $1
}
alias pii='pip install'
alias piu='pip uninstall'

pdf2image () {
  if [ $# -ne 2 ]; then
    echo "Usage: pdf2png INPUT_FILE OUTPUT_FILE"
    exit 1
  fi
  pdf_file=$1
  out_file=$2
  convert -density 300x300 -quality 95 $pdf_file $out_file
}
