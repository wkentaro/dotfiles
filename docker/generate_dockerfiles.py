#!/usr/bin/env python

import datetime
import os.path as osp
import string
import sys

import pytz


here = osp.dirname(osp.realpath(__file__))


class Template(string.Template):
    delimiter = '@'


def main():
    args = sys.argv[:]
    args[0] = osp.realpath(args[0])

    template = Template(open(osp.join(here, 'Dockerfile.in')).read())
    for dist in ['trusty', 'xenial']:
        content = template.substitute({
            'timestamp': datetime.datetime.now(pytz.utc).isoformat(),
            'command': ' '.join(args),
            'upstream_image': 'ubuntu:%s' % dist,
        })
        dockerfile = osp.join(here, 'ubuntu-%s/Dockerfile' % dist)
        with open(dockerfile, 'w') as f:
            f.write(content)
        print('Generated: %s' % dockerfile)


if __name__ == '__main__':
    main()
