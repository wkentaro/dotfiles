#!/usr/bin/env python

import argparse

import cv2
import path


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("files", nargs="+", help="files")
args = parser.parse_args()

img_file = args.files[0]
img = cv2.imread(img_file)

max_height = 500
max_width = 1000

height, width = img.shape[:2]

scale = min(max_height / height, max_width / width)

img = cv2.resize(img, None, None, fx=scale, fy=scale)

roi = cv2.selectROI(img)
x1, y1, w, h = roi
x2 = x1 + w
y2 = y1 + h

x1 = int(round(x1 / scale))
x2 = int(round(x2 / scale))
y1 = int(round(y1 / scale))
y2 = int(round(y2 / scale))

for img_file in args.files:
    img_file = path.Path(img_file)
    out_file = img_file.parent / "cropped" / img_file.basename()
    out_file.parent.makedirs_p()

    img = cv2.imread(img_file)
    img = img[y1:y2, x1:x2]

    cv2.imwrite(out_file, img)
