if [ "$(uname -s)" != "Linux" ]; then
  return
fi

alias aps='apt search'
compdef _aps aps='apt search'

alias ap='sudo apt'
compdef _ap ap='sudo apt'
alias apa='sudo apt autoclean'
compdef _apa apa='sudo apt autoclean'
alias app='sudo apt purge'
compdef _app app='sudo apt purge'
alias apu='sudo apt update'
compdef _apu apu='sudo apt update'
alias apuu='sudo apt update && sudo apt upgrade -y && sudo apt-get autoremove -y && sudo apt-get autoclean'
compdef _apuu apuu='sudo apt update && sudo apt upgrade'

alias allpkgs='apt search -F "%p" --disable-columns $1'

alias shutdown='sudo shutdown -h now'

alias purge="sudo sh -c 'echo 3 >/proc/sys/vm/drop_caches'"
