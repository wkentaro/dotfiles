# vim: set ft=sh:

alias cl='clear'
alias lv='less'

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

# python
alias py='python'
alias ipy='ipython'

# herdr
select_herdr_session() {
  local jq_filter="$1"
  local query="${2:-}"
  local sessions
  sessions=$(herdr session list --json 2>/dev/null | jq -r "$jq_filter | [.name, (if .running then \"running\" else \"stopped\" end), .session_dir, .socket_path] | @tsv") || return
  if [[ -z "$sessions" ]]; then
    print -u2 'No matching Herdr sessions.'
    return 1
  fi

  local selected
  selected=$(print -r -- "$sessions" | fzf --query="$query" \
    --exact --no-sort --cycle --keep-right --tabstop=1 \
    --bind=ctrl-z:ignore,btab:up,tab:down \
    --border=sharp --height=45% --info=inline --layout=reverse \
    --delimiter=$'\t' --header=$'NAME\tSTATUS\tDIRECTORY\tSOCKET' \
    --preview='printf "Name: %s\nStatus: %s\nDirectory: %s\nSocket: %s\n" {1} {2} {3} {4}' \
    --preview-window=down,30%,sharp) || return
  print -r -- "${selected%%$'\t'*}"
}
attach_herdr_session() {
  local session
  session=$(select_herdr_session '.sessions[]' "${1:-}") || return
  [[ -n "$session" ]] && herdr session attach "$session"
}
alias h='attach_herdr_session'
start_herdr_session() {
  if (( $# == 0 )); then
    print -u2 'Please specify session name.'
    return 1
  fi
  herdr --session "$1"
}
alias hn='start_herdr_session'
kill_and_delete_session() {
  local session
  session=$(select_herdr_session '.sessions[]' "${1:-}") || return

  local is_running
  is_running=$(herdr session list --json 2>/dev/null | jq -r --arg name "$session" '.sessions[] | select(.name == $name) | .running') || return
  if [[ "$is_running" == true ]]; then
    herdr session stop "$session" || return
  fi
  [[ "$session" == default ]] && return
  herdr session delete "$session"
}
alias hk='kill_and_delete_session'

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
  command gdircolors &>/dev/null && eval "$(command gdircolors -b)"
  alias ls='command gls --color=auto'
else
  command dircolors &>/dev/null && eval "$(command dircolors -b)"
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
  command diff -u "$@" | delta
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

alias gs="git status"

users_by_ps () {
  ps auxwww | awk '{print $1}' | egrep "$(command ls /home)" | sort | uniq -c | sort -nr | xargs
}

function gcd () {
  cd $(git rev-parse --show-toplevel)
}
alias gcd=gcd

alias skim='open -a Skim'

# unalias gco
# gco () {
#   if [ $# -eq 0 ]; then
#     local branch=$(git branch | fzf | sed -e 's/^\*//' | awk '{print $1}')
#     if [ -z $branch ]; then
#       return 1
#     fi
#     git checkout $branch
#   else
#     git checkout $@
#   fi
# }

alias gbb="git-better-branch.sh"
