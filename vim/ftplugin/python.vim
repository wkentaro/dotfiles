python << EOF
import os
import sys

home = os.path.expanduser("~")
for path in [
          home + "/.anaconda2/lib/python2.7/site-packages",
          home + "/.anaconda2/envs/mvtk/lib/python2.7/site-packages",
          home + "/mvtk/mvtk",
          home + "/mvtk/scikit-image",
          home + "/mvtk/imgaug",
          home + "/mvtk/fcn",
          home + "/mask_rcnn/mask-rcnn/",
          home + "/mask_rcnn/mask-rcnn/chainer",
          home + "/vision/core",
      ]:
  if not path in sys.path:
      sys.path.insert(0, path)
EOF
