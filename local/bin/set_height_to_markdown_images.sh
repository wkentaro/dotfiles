#!/bin/bash

tmpfile=$(mktemp)

nvim $tmpfile

# convert markdown images to html and set height to 300px
sed -i 's/\!\[\(.*\)\](\(.*\))/<img src="\2" alt="\1" height="300px"\/>/g' $tmpfile

cat $tmpfile | xclip -selection clipboard

rm -f $tmpfile
