#!/usr/bin/env python3
"""Collect all source manifests, including packages with COLCON_IGNORE."""
import argparse
import os
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    if not args.source.is_dir():
        parser.error("Source directory does not exist: {}".format(args.source))
    packages = {}
    for directory, subdirs, files in os.walk(str(args.source)):
        subdirs[:] = sorted(d for d in subdirs if d not in
                            {".git", "__pycache__", "build", "install", "log"})
        if "package.xml" not in files:
            continue
        manifest = Path(directory) / "package.xml"
        name = ET.parse(str(manifest)).getroot().findtext("name")
        if not name or "/" in name or name in {".", ".."}:
            parser.error("Invalid package name in {}".format(manifest))
        if name in packages:
            parser.error("Duplicate package {}: {}, {}".format(name, packages[name], manifest))
        packages[name] = manifest
    if not packages:
        parser.error("No package.xml found in {}".format(args.source))
    for name, manifest in sorted(packages.items()):
        target = args.destination / name
        target.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(str(manifest), str(target / "package.xml"))
    print("Collected {} package manifests".format(len(packages)))


if __name__ == "__main__":
    main()
