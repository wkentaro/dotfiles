#!/usr/bin/env python
# -*- coding: utf-8 -*-

import datetime
import subprocess
import sys


def main():
    endpoint = 'http://localhost:8091/google-home-notifier'

    now = datetime.datetime.now()
    if now.minute != 0:
        sys.exit(1)

    text = [now.strftime('%-m月%-d日%-H時をお知らせします。')]

    cmd = 'curl -X POST -d "text={text}" {endpoint}'.format(
        text='\n'.join(text),
        endpoint=endpoint,
    )
    subprocess.call(cmd, shell=True)


if __name__ == '__main__':
    main()
