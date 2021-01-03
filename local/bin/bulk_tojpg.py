#!/usr/bin/env python

import argparse

import cv2
import path


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("files", nargs="+", help="files")
args = parser.parse_args()

for img_file in args.files:
    img_file = path.Path(img_file)
    out_file = img_file.parent / "jpg" / img_file.with_suffix(".jpg").basename()
    out_file.parent.makedirs_p()

    img = cv2.imread(img_file)

    cv2.imwrite(out_file, img)
