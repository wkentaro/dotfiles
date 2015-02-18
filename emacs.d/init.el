;; (put 'upcase-region 'disabled nil)

;;; ロードパス
(setq load-path (append
                 '(;"~/.emacs.d"
                   "~/.emacs.d/packages")
                 load-path))

;;; grep
(require 'grep)
(setq grep-command-before-query "grep -nH -r -e ")
(defun grep-default-command ()
  (if current-prefix-arg
      (let ((grep-command-before-target
             (concat grep-command-before-query
                     (shell-quote-argument (grep-tag-default)))))
        (cons (if buffer-file-name
                  (concat grep-command-before-target
                          " *."
                          (file-name-extension buffer-file-name))
                (concat grep-command-before-target " ."))
              (+ (length grep-command-before-target) 1)))
    (car grep-command)))
(setq grep-command (cons (concat grep-command-before-query " .")
                         (+ (length grep-command-before-query) 1)))


;;; メニューバーを消す
(menu-bar-mode -1)

;;; ツールバーを消す
(tool-bar-mode -1)

;;; カーソルの点滅をとめる
(blink-cursor-mode 0)

;;; 対応する括弧を光らせる
(show-paren-mode 1)

;;; ウィンドウ内に収まらないときだけカッコ内も光らせる
(setq show-paren-style 'mixed)

;;; 行末の空白を表示　
;(setq-default show-trailing-whitespace t)

;;; 現在行を目立たせる　
; (global-hl-line-mode)

;;; 行の先頭をC-kを一回押すだけで行全体を表示する
(setq kill-whole-line t)

;;; 最終行に必ず一行挿入する
(setq require-final-newline t)

;;; バッファの最後でnewlineで新規行を追加するのを禁止する
(setq next-line-add-newlines nil)

;;; バックアップファイルを作らない
(setq make-backup-files nil)

;;; 終了時にオートセーブファイルを消す
(setq delete-auto-save-files t)





;;; 補完
(setq completion-ignore-case t)
(setq read-file-name-completion-ignore-case t)

;;; 部分一致の補完機能を使う
;;; p-bでprint-bufferとか
;;;;; (partial-completion-mode t)

;; 補完可能なものを随時表示
(icomplete-mode 1)



;;; 履歴数
(setq history-length 10000)

;;; ミニバッファの履歴を保存する
(savehist-mode 1)

;;; 最近開いたファイルを保存する数
(setq recentf-max-saved-items 10000)



;;; ediffを1ウィンドウで実行
(setq ediff-window-setup-function 'ediff-setup-windows-plain)

;;; diffのオプション
(setq diff-switches '("-u" "-p" "-N"))






;;; diredを便利にする
(require 'dired-x)


;;; diredから"r"でファイル名インライン編集する
(require 'wdired)
(define-key dired-mode-map "r" 'wdired-change-to-wdired-mode)


;;; ファイル名が重複していたらディレクトリ名を追加する
(require 'uniquify)
(setq uniqufy-buffer-name-style 'post-forward-angle-brackets)


;;; デフォルトのタブ
(setq-default indent-tabs-mode nil)
(setq-default tab-width 4)

;;; auto-install
(require 'auto-install)
(setq auto-install-directory "~/.emacs.d/packages/")
(add-to-list 'load-path auto-install-directory)
; (auto-install-update-emacswiki-package-name t)
(auto-install-compatibility-setup)
(setq ediff-window-setup-function 'ediff-setup-windows-plain)

;; trr
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

;; roseus
; (when platform-linux-p
;   (require 'rosemacs)
;   (invoke-rosemacs)
;   (global-set-key "\C-x\C-r" ros-keymap))
