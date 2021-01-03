#!/usr/bin/env python

import argparse

import cv2
import path


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("files", nargs="+", help="files")
parser.add_argument("--scale", type=float, required=True, help="scale")
args = parser.parse_args()

for img_file in args.files:
    img_file = path.Path(img_file)
    out_file = img_file.parent / "resized" / img_file.basename()
    out_file.parent.makedirs_p()

    img = cv2.imread(img_file)
    img = cv2.resize(img, None, None, fx=args.scale, fy=args.scale)

    cv2.imwrite(out_file, img)
