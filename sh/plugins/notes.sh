#!/bin/sh

n () {
  $EDITOR ~/notes/inbox
}

nls () {
  find ~/notes -iname '*.rst' | egrep -v 'README.rst$'
}