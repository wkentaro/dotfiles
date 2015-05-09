;; Check platform & emacs-version

;; URL:
;;   https://github.com/ongaeshi/platform-p

;; Reference:
;;   http://d.hatena.ne.jp/tomoya/20090807/1249601308 (not me)

;; Usage:
;; (require 'platform-p)
;;
;; (when platform-darwin-p
;;   ;; Meta -> Command
;;   (setq ns-command-modifier (quote meta))
;;   (setq ns-alternate-modifier (quote super))
;;   )

;; Code 

(defun x->bool (elt) (not (not elt)))

;; emacs-version predicates
(setq emacs-version-22-p (string-match "^22" emacs-version)
      emacs-version-23-p (string-match "^23" emacs-version)
      emacs-version-23.0-p (string-match "^23\.0" emacs-version)
      emacs-version-23.1-p (string-match "^23\.1" emacs-version)
      emacs-version-23.2-p (string-match "^23\.2" emacs-version))

;; system-type predicates
(setq platform-darwin-p  (eq system-type 'darwin)
      platform-ns-p      (eq window-system 'ns)
      platform-carbon-p  (eq window-system 'mac)
      platform-linux-p   (eq system-type 'gnu/linux)
      platform-colinux-p (when platform-linux-p
                  (let ((file "/proc/modules"))
                    (and
                     (file-readable-p file)
                     (x->bool
                      (with-temp-buffer
                        (insert-file-contents file)
                        (goto-char (point-min))
                        (re-search-forward "^cofuse\.+" nil t))))))
      platform-cygwin-p  (eq system-type 'cygwin)
      platform-nt-p      (eq system-type 'windows-nt)
      platform-meadow-p  (featurep 'meadow)
      platform-windows-p (or platform-cygwin-p platform-nt-p platform-meadow-p))

(provide 'platform-p)
;;; environment-p.el ends 
