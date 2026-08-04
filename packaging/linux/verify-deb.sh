#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
  echo "usage: $0 /packages/package.deb" >&2
  exit 2
fi

package="$1"
apt-get update
apt-get install -y --no-install-recommends \
  "${package}" libgl1-mesa-dri xvfb

for executable in \
  pc_gui pc_viewer pc_player pc_convert pc_batch_convert pc_render; do
  command -v "${executable}" >/dev/null
done

xvfb-run -a pc_gui --smoke-test
pc_viewer --help >/dev/null
pc_player --help >/dev/null
pc_convert --help >/dev/null
pc_batch_convert --help >/dev/null
pc_render --help >/dev/null
