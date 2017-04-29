;; package
(add-to-list 'load-path (locate-user-emacs-file "el-get/el-get"))
(unless (require 'el-get nil 'noerror)
  (with-current-buffer
      (url-retrieve-synchronously
       "https://raw.githubusercontent.com/dimitri/el-get/master/el-get-install.el")
    (goto-char (point-max))
    (eval-print-last-sexp)))

(el-get-bundle auto-complete)
(el-get-bundle kawabata/emacs-trr)
(el-get-bundle jorgenschaefer/elpy)

;; for Homebrew
(let ((default-directory "/usr/local/share/emacs/site-lisp/"))
  (normal-top-level-add-subdirs-to-load-path))

;; basic
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
(setq completion-ignore-case t)
(setq read-file-name-completion-ignore-case t)
(icomplete-mode 1)
(setq history-length 10000)
(savehist-mode 1)
(setq recentf-max-saved-items 10000)
(setq-default indent-tabs-mode t)
(setq-default tab-width 4)

;; avoid yn for symlink
(setq vc-follow-symlinks t)

; ;; elpy
; (elpy-enable)
