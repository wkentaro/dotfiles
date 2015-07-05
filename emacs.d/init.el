(setq load-path (append '("~/.emacs.d/packages") load-path))

(setq-default indent-tabs-mode nil)

(define-key global-map "\C-h" 'delete-backward-char) ; delete
(define-key global-map "\M-?" 'help-for-help) ; help

(show-paren-mode 1)

(setq show-paren-style 'mixed)

(setq kill-whole-line t)

(setq require-final-newline t)

(setq next-line-add-newlines nil)

(setq make-backup-files nil)

(setq delete-auto-save-files t)

;; avoid "Symbolic link to SVN-controlled source file; follow link? (yes or no)"
(setq vc-follow-symlinks t)

(setq completion-ignore-case t)
(setq read-file-name-completion-ignore-case t)

(icomplete-mode 1)

(setq history-length 10000)

(savehist-mode 1)

(setq recentf-max-saved-items 10000)

(setq-default indent-tabs-mode t)
(setq-default tab-width 4)

;;; auto-install
(require 'auto-install)
(setq auto-install-directory "~/.emacs.d/packages/")
(add-to-list 'load-path auto-install-directory)
; (auto-install-update-emacswiki-package-name t)
(auto-install-compatibility-setup)
(setq ediff-window-setup-function 'ediff-setup-windows-plain)

;; trr
(add-to-list 'load-path "~/.emacs.d/packages/platform-p")
(require 'platform-p)
(setq TRR:japanese nil)
(when platform-darwin-p
  (add-to-list 'load-path "/usr/local/Cellar/apel/10.8/share/emacs/site-lisp")
  (add-to-list 'load-path "/usr/local/Cellar/trr/22.0.99.5/share/emacs/site-lisp")
  (autoload 'trr "/usr/local/Cellar/trr/22.0.99.5/share/emacs/site-lisp/trr" nil t))
(when platform-linux-p
  (add-to-list 'load-path "/usr/share/emacs/site-lisp/trr22")
  (add-to-list 'load-path "/usr/local/share/emacs/24.3/site-lisp/emu")
  (autoload 'trr "/usr/share/emacs/site-lisp/trr22/trr" nil t))
