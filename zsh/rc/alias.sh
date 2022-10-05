# vim: set ft=sh:

# basic
alias sudo='sudo '
alias cl='clear'
alias lv='less'
# if which pygmentize &>/dev/null; then
#   export LESSOPEN="|$HOME/.lessfilter %s"
# fi

alias rm='rm -i'
alias rmr='rm -r'
alias mv='mv -i'
alias cp='cp -i'

# date
alias datestamp='date +"%Y%m%d"'
alias timestamp='date +"%Y%m%d_%H%M%S"'
alias timestamp2='date +"%Y-%m-%d_%H-%M-%S"'

# mv
alias mvi='mv -i'

# rm
alias rmi='rm -i'
alias rmr='rm -r'

# open
if [ "$(uname)" = "Linux" ]; then
  if type xdg-open &>/dev/null; then
    alias open='xdg-open $@ 2>/dev/null'
  else
    alias open='gnome-open $@ 2>/dev/null'
  fi
fi
alias o='open'
alias o.='open .'

# browsing
alias gcal='open https://www.google.com/calendar/render#g >/dev/null 2>&1'
alias gmail='open https://mail.google.com/mail/u/0/ >/dev/null 2>&1'
alias evernote='open https://www.evernote.com/client/web?login=true >/dev/null 2>&1'
alias trello='open https://trello.com/b/pd7DRCtJ/all >/dev/null 2>&1'

# vim
# type vim &>/dev/null && {
#   alias vi='vim'
#   alias vii='vim --noplugin'
#   alias viii='vim -u NONE'
# }
# alias vim-euc='vim -c ":e ++enc=euc-jp"'
# alias vim-iso='vim -c ":e ++enc=iso-2022-jp"'

# emacs
alias emacs='emacs -nw'

# python
alias py='python'
alias ipy='ipython'
alias ipp='ptipython'
alias fl='PYTHONWARNINGS=ignore flake8'
alias bl='black'

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
tmux_fzf_attach() {
    if [[ $1 == "" ]]; then
        PERCOL="fzf --layout reverse"
    else
        PERCOL="fzf --layout reverse --query $1"
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
alias ta='tmux_fzf_attach'
alias tca="TMUX_OPTIONS='-CC' tmux_fzy_attach"
alias tcns="TMUX_OPTIONS='-CC' tns"
alias tn='tmux new'
tns () {
  if [ $# -eq 0 ]; then
    echo 'Please specify session name.'
  fi
  # tmux -L $1 $TMUX_OPTIONS new -s $1
  tmux new -s $1
}

# brew
if type brew &>/dev/null; then
  alias bubu='brew update && brew upgrade && brew cleanup'
  alias bb='bubu'
  alias bbb='bubu'
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
if command gls &>/dev/null; then
  alias ls='gls --color=auto'
else
  alias ls='ls --color=auto'
fi
alias sl='ls'
alias lsa='ls -lha'
alias la='lsa'
# if type dircolors &>/dev/null; then
#   [ -f $HOME/.dircolors.256dark ] && eval $(dircolors $HOME/.dircolors.256dark 2>/dev/null)
# fi
# if command ls --color &>/dev/null; then
#   # GNU ls
#   if [ $TERM = "dumb" ]; then
#     # Disable colors in GVim
#     alias ls='ls --show-control-chars'
#     alias la='ls -ah --show-control-chars'
#     alias ll='ls -lh --show-control-chars'
#     alias lsa='ls -lah --show-control-chars'
#   else
#     # Color settings for zsh complete candidates
#     alias ls='ls --show-control-chars --color=always'
#     alias la='ls -ah --show-control-chars --color=always'
#     alias ll='ls -lh --show-control-chars --color=always'
#     alias lsa='ls -lah --show-control-chars --color=always'
#   fi
# else
#   export LSCOLORS=ExGxBxDxCxEgEdxbxgxcxd
#   alias ls='ls -G'
#   alias la='ls -ah'
#   alias ll='ls -lh'
#   alias lsa='ls -lah'
# fi

# ssh
# alias ssh='ssh -C -X'
# alias ssh='ssh -c arcfour'

trash() {
  if [ $# -eq 0 ]; then
    echo "usage: $0 FILES"
    return 1
  fi
  if [ $(uname) = Linux ]; then
    mv $* ~/.local/share/Trash/files
  elif [ $(uname) = Darwin ]; then
    mv $* ~/.Trash
  else
    echo "ERROR: unsupported os: $(uname)"
    return 1
  fi
}

# ----------------------------------------------------
# Show Setup
# ----------------------------------------------------

show-ros () {
  # CATKIN_TOOLS_VERSION=$(python -c "import pkg_resources; print(pkg_resources.get_distribution('catkin-tools').version)" 2>/dev/null)
  echo "ROS_DISTRO: $ROS_DISTRO"
  # echo "CATKIN_TOOLS_VERSION: $CATKIN_TOOLS_VERSION"
  echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"
}
alias show_ros=show-ros

show-cuda () {
  which nvcc &>/dev/null || return 1
  # cuda
  CUDA_VERSION=$(command nvcc --version | sed -n 4p | sed 's/.*, release .*, V\(.*\)/\1/')
  echo "CUDA_VERSION: $CUDA_VERSION"
  if [ -e $CUDA_PATH/include/cudnn_version.h ]; then
    local cudnn_major=$(grep '#define CUDNN_MAJOR' $CUDA_PATH/include/cudnn_version.h | cut -d ' ' -f 3)
    local cudnn_minor=$(grep '#define CUDNN_MINOR' $CUDA_PATH/include/cudnn_version.h | cut -d ' ' -f 3)
    local cudnn_patch=$(grep '#define CUDNN_PATCHLEVEL' $CUDA_PATH/include/cudnn_version.h | cut -d ' ' -f 3)
    echo "CUDNN_VERSION: $cudnn_major.$cudnn_minor.$cudnn_patch"
  fi
}
alias show_cuda=show-cuda

if which nvidia-smi &>/dev/null; then
  alias nvid='nvidia-smi'

  watch-gpu () {
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
fi
if which cuda-smi &>/dev/null; then
  alias cud='cuda-smi'
fi

init-autoenv() {
  vim .in
  vim .out
}

macclean () {
  find . -type f -name '.DS_Store' -delete
}

compress-pdf () {
  if [ ! $# -eq 2 ]; then
    echo "Usage: compress_pdf INPUT_FILE OUTPUT_FILE"
  fi
  local input
  local output
  input=$1
  output=$2
    # -dPDFSETTINGS=/default \
  gs \
    -sDEVICE=pdfwrite \
    -dCompatibilityLevel=1.4 \
    -dPDFSETTINGS=/prepress \
    -dNOPAUSE \
    -dQUIET \
    -dBATCH \
    -dDetectDuplicateImages \
    -dCompressFonts=true \
    -r300 \
    -sOutputFile=${output} \
    ${input}
}

cmake-prefix.. () {
  if [ $# != 1 ]; then
    return 1
  fi

  cmake -DCMAKE_INSTALL_PREFIX:PATH=$1 ..
}

alias tailf='tail -n1000 -f'
psf () {
  if [ ! $# -eq 1 ]; then
    echo "Usage: $0 PATTERN"
    return 1
  fi
  ps auxwww | egrep $USER | egrep $1 | grep -v grep
}
psk() {
  while read line; do
    read -q "REPLY?[KILL?] [$line] [yn]: "
    echo
    if [ "$REPLY" == "y" ]; then
      echo $line | awk '{print $2}' | xargs kill -9
    fi
  done
}
alias pii='pip install'
alias piu='pip uninstall'

pdf2image () {
  if [ $# -ne 2 ]; then
    echo "Usage: pdf2png INPUT_FILE OUTPUT_FILE"
    return 1
  fi
  pdf_file=$1
  out_file=$2
  convert -density 300x300 -quality 95 $pdf_file $out_file
}

alias m='make'
alias mw='make watch'

if [ $(uname) = Darwin ]; then
  alias matlab='/Applications/MATLAB_R2018b.app/bin/matlab -nodesktop -nosplash'
  alias matlab-desktop='/Applications/MATLAB_R2018b.app/bin/matlab'
else
  alias matlab='$(command matlab) -nodesktop -nosplash'
  alias matlab-desktop='$(command matlab)'
fi

meshlab () {
  if [ "$(uname)" = "Darwin" ]; then
    cmd=/Applications/meshlab.app/Contents/MacOS/meshlab
  else
    cmd=$(command which meshlab)
  fi
  if [ $# -ge 1 ]; then
    local filename=$1
    (cd $(dirname $filename) && $cmd $(basename $filename) &>/dev/null)
  else
    $cmd &>/dev/null
  fi
}

if [ "$(uname)" = "Darwin" ]; then
  meshlabserver () {
    base_dir=/Applications/meshlab.app/Contents
    DYLD_FRAMEWORK_PATH=$base_dir/Frameworks $base_dir/MacOS/meshlabserver $*
  }
fi

nhup () {
  nohup $* > nohup.$(date +%Y%m%d_%H%M%S.%N).out &
}

alias k9='kill -9'

# diff () {
#         if zstyle -t ':prezto:module:utility:diff' color
#         then
#                 if (( $+commands[colordiff] ))
#                 then
#                         command colordiff --unified "$@"
#                 elif (( $+commands[git] ))
#                 then
#                         git --no-pager diff --color=auto --no-ext-diff --no-index "$@"
#                 else
#                         command diff --unified "$@"
#                 fi
#         else
#                 command diff --unified "$@"
#         fi
# }

# if which exa &>/dev/null; then
#   alias ls=exa
# fi

function diff () {
  command diff -u "$@" | diff-so-fancy
}

# alias lt="logtable"

# alias watch="watch -t"
# watch() {
#   if [ $# -eq 0 ]; then
#     echo "usage: watch [COMMANDS]"
#     return 1
#   fi
#
#   local stdout
#   while true; do
#     stdout=$(eval "$*")
#     (clear && echo $stdout)
#   done
# }
# alias w=watch

# if which nvim &>/dev/null; then
#   alias vim=nvim
# fi

alias ic=imgcat

sshL() {
  local host=$1
  local port=$2
  ssh ${host} -L ${port}:localhost:${port}
}
if which compdef &>/dev/null; then
  compdef sshL=ssh
fi

avi-to-mp4 () {
  if [ $# -ne 1 ]; then
    echo "Usage: $0 AVI_FILE"
    return 1
  fi
  input_file=$1
  output_file=${input_file/.avi/.mp4}
  ffmpeg -i $input_file -c:v copy -c:a copy -y $output_file
}

store_env_for_ros () {
  export _PYTHONPATH=$PYTHONPATH
  export _LD_LIBRARY_PATH=$LD_LIBRARY_PATH
  export _CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH
}

restore_env_for_ros () {
  export PYTHONPATH=$_PYTHONPATH
  export LD_LIBRARY_PATH=$_LD_LIBRARY_PATH
  export CMAKE_PREFIX_PATH=$_CMAKE_PREFIX_PATH
}

alias gs='git status'
alias w='watch'

users_by_ps () {
  ps auxwww | awk '{print $1}' | egrep "$(command ls /home)" | sort | uniq -c | sort -nr | xargs
}

function gcd () {
  cd $(git rev-parse --show-toplevel)
}
alias gcd=gcd

alias skim='open -a Skim'
