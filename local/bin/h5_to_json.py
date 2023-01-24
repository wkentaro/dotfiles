#!/usr/bin/env python

import argparse
import json
import sys

import h5py
import numpy


def h5_to_json(group, indent=0):
    data = {}
    for field in group:
        if isinstance(group[field], h5py.Group):
            data[field] = h5_to_json(group[field], indent=indent + 2)
        elif isinstance(group[field], h5py.Dataset):
            value = group[field].value
            shape = group[field].shape
            dtype = group[field].dtype

            if shape == ():
                if isinstance(value, numpy.void):
                    data[field] = "<numpy.void: %s>" % len(value.tolist())
                elif dtype == object:
                    if isinstance(value, (str, unicode)):
                        try:
                            data[field] = json.loads(value)
                        except ValueError:
                            data[field] = value
                    else:
                        raise NotImplementedError
                else:
                    data[field] = value.item()
            else:
                if value.size <= 16:
                    data[field] = value.tolist()
                else:
                    data[field] = "<numpy.ndarray: %s %s>" % (shape, dtype)
        else:
            raise NotImplementedError
    return data


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("h5_file", help="HDF5 file to convert")
    args = parser.parse_args()

    with h5py.File(args.h5_file, "r+") as f:
        data = h5_to_json(group=f)

    json.dump(data, sys.stdout, indent=2)
    sys.stdout.write("\n")


if __name__ == "__main__":
    main()
