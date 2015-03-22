autoload -U colors && colors

autoload -Uz vcs_info

zstyle ':vcs_info:*' stagedstr '%F{green}●'
zstyle ':vcs_info:*' unstagedstr '%F{yellow}●'
zstyle ':vcs_info:*' check-for-changes true
zstyle ':vcs_info:(sv[nk]|bzr):*' branchformat '%b%F{1}:%F{11}%r'
zstyle ':vcs_info:*' enable git svn
theme_precmd () {
    if [[ -z $(git ls-files --other --exclude-standard 2> /dev/null) ]] {
        zstyle ':vcs_info:*' formats '(%b%c%u%B%F{magenta})'
    } else {
        zstyle ':vcs_info:*' formats '(%b%c%u%B%F{red}●%F{magenta})'
    }
    vcs_info
}

setopt prompt_subst

PROMPT='%(!.%{$fg[red]%}.%{$fg[green]%}%n@)%m:%{$fg_bold[blue]%}%c%{$fg_bold[magenta]%}${vcs_info_msg_0_}%{$fg[cyan]%}%{$reset_color%} %% '

autoload -U add-zsh-hook
add-zsh-hook precmd  theme_precmd

