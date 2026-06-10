#!/usr/bin/env bash
#
# Install or update Claude Code config: personal dotfile symlinks, ECC slash
# commands, brooks-lint commands/skills, and external skills (vercel-labs/skills
# CLI).

set -euo pipefail

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly DOTFILES_AGENTS="${HOME}/.dotfiles/agents"
readonly DOTFILES_CLAUDE="${HOME}/.dotfiles/claude"
readonly AGENTS_DIR="${HOME}/.agents"
readonly CLAUDE_DIR="${HOME}/.claude"
readonly ECC_REPO="https://github.com/affaan-m/ECC.git"
readonly ECC_CACHE="${HOME}/.local/share/claude-ecc"
readonly ECC_CACHE_LEGACY="${HOME}/.cache/claude-ecc"
readonly BROOKS_REPO="https://github.com/hyhmrright/brooks-lint.git"
readonly BROOKS_CACHE="${HOME}/.local/share/claude-brooks-lint"

# Personal dotfile -> ~/.claude target. Format: "<src>:<dest>".
# Rules are linked at the directory level below; add new files under
# claude/rules/ and they appear automatically.
PERSONAL_LINKS=(
  "${DOTFILES_CLAUDE}/settings.json:${CLAUDE_DIR}/settings.json"
  "${DOTFILES_CLAUDE}/statusline-command.sh:${CLAUDE_DIR}/statusline-command.sh"
)

ECC_COMMANDS=(
  refactor-clean.md
)

# External skills installed via vercel-labs/skills CLI.
# Format: "<owner>/<repo>:<skill>[,<skill>...]".
SKILLS_BY_REPO=(
  "pbakaus/impeccable:impeccable"
  "remotion-dev/skills:remotion-best-practices"
  "coreyhaines31/marketingskills:copywriting,cro,customer-research"
  "mattpocock/skills:diagnose,grill-me,grill-with-docs,handoff,improve-codebase-architecture,prototype,setup-matt-pocock-skills,tdd,to-issues,to-prd,triage,write-a-skill,zoom-out"
  "steipete/agent-scripts:skill-cleaner"
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
# (We link per-entry rather than symlinking the whole directory so other
# tools can drop their own entries into ~/.claude/rules/ alongside ours.)
link_personal_rules() {
  local entry name
  for entry in "${DOTFILES_CLAUDE}"/rules/*; do
    [[ -e "${entry}" ]] || continue
    name=$(basename "${entry}")
    ensure_symlink "${entry}" "${CLAUDE_DIR}/rules/${name}"
  done
}

# Move every entry of a real hub dir into the repo, then remove the empty hub.
# Personal skills already live in the repo (their hub entries are self-links we
# drop); brooks links are dropped and recreated later by link_brooks_content;
# public skills and any agent-authored skill are real dirs we move in. Anything
# else (a dir already in the repo, or stray files like .DS_Store) is discarded so
# the closing rmdir succeeds.
_migrate_skills_hub() {
  local hub="$1" repo="$2"
  local entry dest target

  shopt -s dotglob nullglob
  for entry in "${hub}"/*; do
    dest="${repo}/$(basename "${entry}")"
    if [[ -L "${entry}" ]]; then
      target=$(readlink "${entry}")
      if [[ "${target}" == "${repo}/"* || -e "${dest}" || -L "${dest}" ]]; then
        rm -f "${entry}"
      else
        mv "${entry}" "${dest}"
      fi
    elif [[ -d "${entry}" && ! -e "${dest}" ]]; then
      mv "${entry}" "${dest}"
    else
      rm -rf "${entry}"
    fi
  done
  shopt -u dotglob nullglob
  rmdir "${hub}"
}

# Make the live skills hub (~/.agents/skills) a symlink to the dotfiles skills
# dir, so a skill an agent creates in the hub lands directly in the repo working
# tree. Public/external skills live there too but are gitignored (see
# generate_skills_gitignore); only personal skills get tracked. On first run the
# hub is a real dir of mixed content that _migrate_skills_hub folds into the repo.
ensure_skills_hub() {
  local hub="${AGENTS_DIR}/skills"
  local repo="${DOTFILES_AGENTS}/skills"
  local current

  mkdir -p "${repo}" "${AGENTS_DIR}"

  if [[ -L "${hub}" ]]; then
    current=$(readlink "${hub}")
    if [[ "${current}" == "${repo}" ]]; then
      log "skip: ${hub} (hub symlink ok)"
      return
    fi
    log "relink hub: ${hub} -> ${repo} (was -> ${current})"
    rm -f "${hub}"
  elif [[ -d "${hub}" ]]; then
    log "migrate hub: ${hub} -> ${repo}"
    _migrate_skills_hub "${hub}" "${repo}"
  elif [[ -e "${hub}" ]]; then
    die "${hub} exists but is not a directory or symlink"
  fi

  ln -s "${repo}" "${hub}"
  log "link: ${hub} -> ${repo}"
}

# Regenerate the skills .gitignore from the single source of truth: every skill
# we install from a marketplace (SKILLS_BY_REPO) plus brooks-lint's skills. These
# share the hub but are external, so they stay untracked; personal skills are not
# listed and therefore tracked.
generate_skills_gitignore() {
  local out="${DOTFILES_AGENTS}/skills/.gitignore"
  local entry
  local -a names=() parts=()

  for entry in "${SKILLS_BY_REPO[@]}"; do
    IFS=',' read -ra parts <<< "${entry#*:}"
    names+=( "${parts[@]}" )
  done
  for entry in "${BROOKS_CACHE}"/skills/*/; do
    [[ -d "${entry}" ]] || continue
    names+=( "$(basename "${entry%/}")" )
  done

  {
    printf '# Generated by scripts/install_claude_code_config.sh -- do not edit by hand.\n'
    printf '# External skills sharing this hub; only personal skills are tracked.\n'
    printf '%s\n' "${names[@]}" | LC_ALL=C sort -u
  } > "${out}"
  log "write: ${out} (${#names[@]} external skills ignored)"
}

ensure_skills_bridge() {
  local entry name

  if [[ -L "${CLAUDE_DIR}/skills" ]]; then
    local current
    current=$(readlink "${CLAUDE_DIR}/skills")
    if [[ "${current}" == "${AGENTS_DIR}/skills" ]]; then
      log "skip: ${CLAUDE_DIR}/skills (symlink ok)"
      return
    fi
  elif [[ -d "${CLAUDE_DIR}/skills" ]]; then
    shopt -s dotglob nullglob
    for entry in "${CLAUDE_DIR}/skills"/*; do
      name=$(basename "${entry}")
      if [[ -e "${AGENTS_DIR}/skills/${name}" || -L "${AGENTS_DIR}/skills/${name}" ]]; then
        die "conflicting skill entry exists in both ${CLAUDE_DIR}/skills and ${AGENTS_DIR}/skills: ${name}"
      fi
      log "migrate: ${entry} -> ${AGENTS_DIR}/skills/${name}"
      mv "${entry}" "${AGENTS_DIR}/skills/${name}"
    done
    shopt -u dotglob nullglob
    rmdir "${CLAUDE_DIR}/skills"
  elif [[ -e "${CLAUDE_DIR}/skills" ]]; then
    die "${CLAUDE_DIR}/skills exists but is not a directory or symlink"
  fi

  ensure_symlink "${AGENTS_DIR}/skills" "${CLAUDE_DIR}/skills"
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
  # ECC rules are intentionally not linked into ~/.claude/rules: they load
  # globally on every turn (~13k tokens) and overlap with our personal rules.
  # Only the commands below are installed; use ECC rules per-project if needed.
  if [[ -L "${CLAUDE_DIR}/rules/ecc" ]]; then
    log "unlink: ${CLAUDE_DIR}/rules/ecc (ECC rules no longer global)"
    rm -f "${CLAUDE_DIR}/rules/ecc"
  fi

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

ensure_brooks_clone() {
  local pull="$1"

  if [[ ! -d "${BROOKS_CACHE}/.git" ]]; then
    log "clone: brooks-lint -> ${BROOKS_CACHE}"
    mkdir -p "$(dirname "${BROOKS_CACHE}")"
    git clone "${BROOKS_REPO}" "${BROOKS_CACHE}"
    return
  fi

  (( pull )) || return 0
  log "update: brooks-lint"
  git -C "${BROOKS_CACHE}" pull --ff-only
}

# Link brooks-lint's six skills plus the _shared/ resource dir they reference flat into
# the ~/.agents/skills hub. vercel-labs/skills can't install these (each skill reads
# ../_shared/ as a sibling, which a per-skill install drops), but linking _shared/ as its
# own hub entry keeps that relative reference resolving. Flat (vs. nesting under one
# brooks-lint/ dir) is also what gives each skill its bare /brooks-* slash command; the
# colon-namespaced form only exists for plugin installs. We deliberately skip brooks-lint's
# commands/ wrappers: they invoke the plugin namespace (brooks-lint:brooks-review) and fail
# against a skills-dir install.
link_brooks_content() {
  local entry
  for entry in "${BROOKS_CACHE}"/skills/*/; do
    ensure_symlink "${entry%/}" "${AGENTS_DIR}/skills/$(basename "${entry}")"
  done
}

# The agent-browser CLI ships its own skills (agent-browser skills get core),
# so it is installed as a command rather than via the skills CLI.
ensure_agent_browser_cli() {
  local update="$1"

  if [[ -e "${AGENTS_DIR}/skills/agent-browser" || -L "${AGENTS_DIR}/skills/agent-browser" ]]; then
    log "remove: ${AGENTS_DIR}/skills/agent-browser (skill ships with the CLI now)"
    rm -rf "${AGENTS_DIR}/skills/agent-browser"
  fi

  if command -v agent-browser >/dev/null 2>&1; then
    (( update )) || return 0
    log "update: agent-browser CLI"
  else
    log "install: agent-browser CLI"
  fi
  npm install -g agent-browser
}

# The git-hunk CLI ships its own skill (git-hunk skills get core), so it is
# installed as a uv tool rather than via the skills CLI. install.py runs the
# install_* scripts in an arbitrary order, so uv may not be installed yet; pull
# in install_uv.sh ourselves and put its target dir on PATH.
ensure_git_hunk_cli() {
  local update="$1"

  if [[ -e "${AGENTS_DIR}/skills/git-hunk" || -L "${AGENTS_DIR}/skills/git-hunk" ]]; then
    log "remove: ${AGENTS_DIR}/skills/git-hunk (skill ships with the CLI now)"
    rm -rf "${AGENTS_DIR}/skills/git-hunk"
  fi

  export PATH="${HOME}/.local/bin:${PATH}"
  command -v uv >/dev/null 2>&1 || "${SCRIPT_DIR}/install_uv.sh"

  if command -v git-hunk >/dev/null 2>&1; then
    (( update )) || return 0
    log "update: git-hunk CLI"
    uv tool upgrade git-hunk
  else
    log "install: git-hunk CLI"
    uv tool install git-hunk
  fi
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
  --update: also git-pull the ECC and brooks-lint clones, refresh managed skills,
            and upgrade the agent-browser and git-hunk CLIs
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
    "${CLAUDE_DIR}/commands"

  ensure_skills_hub
  ensure_skills_bridge
  prune_stale_rule_links
  link_personal_files
  link_personal_rules
  ensure_ecc_clone "${update}"
  link_ecc_content
  ensure_brooks_clone "${update}"
  link_brooks_content
  generate_skills_gitignore
  ensure_agent_browser_cli "${update}"
  ensure_git_hunk_cli "${update}"
  install_external_skills

  if (( update )); then
    update_external_skills
  fi
}

main "$@"
