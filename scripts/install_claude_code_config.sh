#!/usr/bin/env bash
#
# Install or update Claude Code config: personal dotfile symlinks, ECC rules
# and slash commands, and external skills (vercel-labs/skills CLI).

set -euo pipefail

readonly DOTFILES_CLAUDE="${HOME}/.dotfiles/claude"
readonly CLAUDE_DIR="${HOME}/.claude"
readonly ECC_REPO="https://github.com/affaan-m/ECC.git"
readonly ECC_CACHE="${HOME}/.local/share/claude-ecc"
readonly ECC_CACHE_LEGACY="${HOME}/.cache/claude-ecc"

# Personal dotfile -> ~/.claude target. Format: "<src>:<dest>".
# Rules and skills are linked at the directory level below; add new files
# under claude/rules/ or claude/skills/ and they appear automatically.
PERSONAL_LINKS=(
  "${DOTFILES_CLAUDE}/settings.json:${CLAUDE_DIR}/settings.json"
  "${DOTFILES_CLAUDE}/statusline-command.sh:${CLAUDE_DIR}/statusline-command.sh"
)

ECC_COMMANDS=(
  code-review.md
  python-review.md
  refactor-clean.md
)

# External skills installed via vercel-labs/skills CLI.
# Format: "<owner>/<repo>:<skill>[,<skill>...]".
SKILLS_BY_REPO=(
  "wkentaro/git-hunk:git-hunk"
  "vercel-labs/agent-browser:agent-browser"
  "pbakaus/impeccable:impeccable"
  "remotion-dev/skills:remotion-best-practices"
  "coreyhaines31/marketingskills:copywriting,cro,customer-research"
  "mattpocock/skills:diagnose,grill-me,grill-with-docs,handoff,improve-codebase-architecture,prototype,setup-matt-pocock-skills,tdd,to-issues,to-prd,triage,write-a-skill,zoom-out"
)

log()  { printf '%s\n' "$*"; }
warn() { printf 'warn: %s\n' "$*" >&2; }
die()  { printf 'error: %s\n' "$*" >&2; exit 1; }

# Skip if anything already lives at $dest (preserves user customizations).
link_if_missing() {
  local src="$1" dest="$2"
  if [[ -e "${dest}" || -L "${dest}" ]]; then
    log "skip: ${dest} (exists)"
    return
  fi
  log "link: ${src} -> ${dest}"
  ln -s "${src}" "${dest}"
}

# Force $dest to be a symlink to $src, replacing wrong symlinks or stale copies.
ensure_symlink() {
  local src="$1" dest="$2"
  if [[ -L "${dest}" ]]; then
    local current
    current=$(readlink "${dest}")
    if [[ "${current}" == "${src}" ]]; then
      log "skip: ${dest} (symlink ok)"
      return
    fi
    log "relink: ${dest} -> ${src} (was -> ${current})"
    rm -f "${dest}"
  elif [[ -e "${dest}" ]]; then
    log "replace copy: ${dest} -> ${src}"
    rm -rf "${dest}"
  else
    log "link: ${dest} -> ${src}"
  fi
  ln -s "${src}" "${dest}"
}

link_personal_files() {
  local entry
  for entry in "${PERSONAL_LINKS[@]}"; do
    link_if_missing "${entry%%:*}" "${entry#*:}"
  done
}

# Mirror each top-level entry under claude/rules/ to ~/.claude/rules/.
# Files and subdirectories alike — add a new file or folder under
# claude/rules/ and it appears with no script change required.
# (~/.claude/rules/ also hosts the `ecc` symlink, so we link per-entry
# rather than symlinking the whole directory.)
link_personal_rules() {
  local entry name
  for entry in "${DOTFILES_CLAUDE}"/rules/*; do
    [[ -e "${entry}" ]] || continue
    name=$(basename "${entry}")
    ensure_symlink "${entry}" "${CLAUDE_DIR}/rules/${name}"
  done
}

link_personal_skills() {
  local skill_dir name
  for skill_dir in "${DOTFILES_CLAUDE}"/skills/*/; do
    [[ -d "${skill_dir}" ]] || continue
    name=$(basename "${skill_dir}")
    link_if_missing "${skill_dir%/}" "${CLAUDE_DIR}/skills/${name}"
  done
}

# Sweep broken symlinks under ~/.claude/rules left over from older layouts
# (e.g. ~/.claude/rules/common/wkentaro.md after the source moved). Only
# touches links whose target used to point into the dotfiles tree.
prune_stale_rule_links() {
  [[ -d "${CLAUDE_DIR}/rules" ]] || return 0

  local link target
  while IFS= read -r link; do
    target=$(readlink "${link}")
    case "${target}" in
      "${DOTFILES_CLAUDE}/"*)
        if [[ ! -e "${target}" ]]; then
          log "prune: ${link} (dangling -> ${target})"
          rm -f "${link}"
        fi
        ;;
    esac
  done < <(find "${CLAUDE_DIR}/rules" -type l)

  find "${CLAUDE_DIR}/rules" -mindepth 1 -type d -empty -delete 2>/dev/null || true
}

ensure_ecc_clone() {
  local pull="$1"

  if [[ -d "${ECC_CACHE_LEGACY}" && ! -d "${ECC_CACHE}" ]]; then
    log "migrate: ${ECC_CACHE_LEGACY} -> ${ECC_CACHE}"
    mkdir -p "$(dirname "${ECC_CACHE}")"
    mv "${ECC_CACHE_LEGACY}" "${ECC_CACHE}"
  fi

  if [[ ! -d "${ECC_CACHE}/.git" ]]; then
    log "clone: ECC -> ${ECC_CACHE}"
    mkdir -p "$(dirname "${ECC_CACHE}")"
    git clone "${ECC_REPO}" "${ECC_CACHE}"
    return
  fi

  (( pull )) || return 0
  log "update: ECC"
  git -C "${ECC_CACHE}" pull --ff-only
}

link_ecc_content() {
  ensure_symlink "${ECC_CACHE}/rules" "${CLAUDE_DIR}/rules/ecc"

  local cmd src
  for cmd in "${ECC_COMMANDS[@]}"; do
    src="${ECC_CACHE}/commands/${cmd}"
    if [[ ! -e "${src}" ]]; then
      warn "command ${cmd} not found in ECC repo"
      continue
    fi
    ensure_symlink "${src}" "${CLAUDE_DIR}/commands/${cmd}"
  done
}

install_skills_from_repo() {
  local repo="$1" csv="$2"
  local skill
  local -a parts=() args=() pending=()

  IFS=',' read -ra parts <<< "${csv}"

  for skill in "${parts[@]}"; do
    if [[ -e "${CLAUDE_DIR}/skills/${skill}" ]]; then
      log "skip: ${skill} (already installed from ${repo})"
      continue
    fi
    args+=( -s "${skill}" )
    pending+=( "${skill}" )
  done

  (( ${#pending[@]} )) || return 0

  log "install (skills add): ${pending[*]} from ${repo}"
  npx skills add "${repo}" "${args[@]}" -a claude-code -g -y
}

install_external_skills() {
  local entry
  for entry in "${SKILLS_BY_REPO[@]}"; do
    install_skills_from_repo "${entry%%:*}" "${entry#*:}"
  done
}

update_external_skills() {
  local entry
  local -a skills=() parts=()
  for entry in "${SKILLS_BY_REPO[@]}"; do
    IFS=',' read -ra parts <<< "${entry#*:}"
    skills+=( "${parts[@]}" )
  done

  log "update: external skills (${#skills[@]} skills)"
  npx skills update -g -y "${skills[@]}"
}

usage() {
  cat <<EOF
usage: $(basename "$0") [-u|--update]
  default:  install missing pieces, skip what already exists
  --update: also git-pull the ECC clone and refresh managed skills
EOF
}

main() {
  local update=0
  while (( $# )); do
    case "$1" in
      -u|--update) update=1; shift ;;
      -h|--help)   usage; return 0 ;;
      *)           die "unknown option: $1" ;;
    esac
  done

  mkdir -p \
    "${CLAUDE_DIR}/rules" \
    "${CLAUDE_DIR}/skills" \
    "${CLAUDE_DIR}/commands"

  prune_stale_rule_links
  link_personal_files
  link_personal_rules
  link_personal_skills
  ensure_ecc_clone "${update}"
  link_ecc_content
  install_external_skills

  if (( update )); then
    update_external_skills
  fi
}

main "$@"
