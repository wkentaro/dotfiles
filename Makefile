SHELL := bash

.PHONY: help install install-private dry-run uv
.DEFAULT_GOAL := help

install install-private dry-run: export PATH := $(HOME)/.local/bin:$(PATH)

help:
	@printf '\033[1;32mAvailable targets:\033[0m\n'
	@grep -E '^[a-zA-Z_-]+:.*# ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*# "} {printf "  \033[1;36m%-20s\033[0m %s\n", $$1, $$2}'

install: uv  # Install dotfiles and commands
	uv run install.py

install-private: uv  # Install dotfiles including the private repo
	uv run install.py --private

dry-run: uv  # Print install commands without executing them
	uv run install.py --dry-run

uv:
	@hash uv 2>/dev/null || bash scripts/install_uv.sh
