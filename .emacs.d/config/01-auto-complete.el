(when (require 'auto-complete-config nil t)
  (add-to-list 'ac-dictionary-directories
               "~/.emacs.d/auto-complete")
  (define-key ac-mode-map (kbd "M-TAB") 'auto-complete)
  (ac-config-default))
