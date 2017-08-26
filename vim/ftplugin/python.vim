python << EOF
import os
import sys

home = os.path.expanduser("~")
for path in [
          home + "/.anaconda2/lib/python2.7/site-packages",
          home + "/.anaconda2/envs/mvtk/lib/python2.7/site-packages",
          home + "/Projects/mvtk/mvtk",
          home + "/Projects/mvtk/scikit-image",
          home + "/Projects/mvtk/imgaug",
          home + "/Projects/mvtk/fcn",
      ]:
  if not path in sys.path:
      sys.path.insert(0, path)
EOF
