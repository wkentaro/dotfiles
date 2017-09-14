#!/bin/bash

if [ "`uname`" != "Darwin" ]; then
  exit 0
fi

sudo mkdir -p /usr/local/texlive/texmf-local/fonts/opentype/public/hiragino/

cd /usr/local/texlive/texmf-local/fonts/opentype/public/hiragino/

sudo ln -s "/Library/Fonts/ヒラギノ明朝 Pro W3.otf" HiraMinPro-W3.otf
sudo ln -s "/Library/Fonts/ヒラギノ明朝 Pro W6.otf" HiraMinPro-W6.otf
sudo ln -s "/Library/Fonts/ヒラギノ角ゴ Pro W3.otf" HiraKakuPro-W3.otf
sudo ln -s "/Library/Fonts/ヒラギノ角ゴ Pro W6.otf" HiraKakuPro-W6.otf
sudo ln -s "/Library/Fonts/ヒラギノ角ゴ Std W8.otf" HiraKakuStd-W8.otf
sudo ln -s "/Library/Fonts/ヒラギノ丸ゴ Pro W4.otf" HiraMaruPro-W4.otf
sudo ln -s "/System/Library/Fonts/ヒラギノ明朝 ProN W3.otf" HiraMinProN-W3.otf
sudo ln -s "/System/Library/Fonts/ヒラギノ明朝 ProN W6.otf" HiraMinProN-W6.otf
sudo ln -s "/System/Library/Fonts/ヒラギノ角ゴ ProN W3.otf" HiraKakuProN-W3.otf
sudo ln -s "/System/Library/Fonts/ヒラギノ角ゴ ProN W6.otf" HiraKakuProN-W6.otf
sudo ln -s "/Library/Fonts/ヒラギノ角ゴ StdN W8.otf" HiraKakuStdN-W8.otf
sudo ln -s "/Library/Fonts/ヒラギノ丸ゴ ProN W4.otf" HiraMaruProN-W4.otf

sudo mktexlsr
sudo updmap-sys --setoption kanjiEmbed hiragino
kanji-config-updmap hiragino