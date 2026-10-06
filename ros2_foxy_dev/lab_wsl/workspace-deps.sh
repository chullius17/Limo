#!/usr/bin/env bash
set -euo pipefail

manifest_dir="$(mktemp -d)"
trap 'rm -rf "$manifest_dir"' EXIT
collect-manifests /workspace/src "$manifest_dir"
sudo apt-get update
sudo rosdep update --rosdistro foxy --include-eol-distros
sudo rosdep install --from-paths "$manifest_dir" --ignore-src -y --rosdistro foxy
