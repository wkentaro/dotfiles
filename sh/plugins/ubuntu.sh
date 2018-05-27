if [ "$(uname -s)" != "Linux" ]; then
  return
fi

alias aps='apt search'
compdef _aps aps='apt search'

alias ap='sudo apt-get'
compdef _ap ap='sudo apt-get'
alias apa='sudo apt-get autoclean'
compdef _apa apa='sudo apt-get autoclean'
alias app='sudo apt purge'
compdef _app app='sudo apt-get purge'
alias apu='sudo apt update'
compdef _apu apu='sudo apt-get update'
alias apuu='sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get autoremove -y && sudo apt-get autoclean'
compdef _apuu apuu='sudo apt-get update && sudo apt-get upgrade'

alias allpkgs='apt search -F "%p" --disable-columns $1'

alias shutdown='sudo shutdown -h now'

alias purge="sudo sh -c 'echo 3 >/proc/sys/vm/drop_caches'"
