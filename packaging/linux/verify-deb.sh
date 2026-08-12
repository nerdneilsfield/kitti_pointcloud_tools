#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)); then
  echo "usage: $0 /packages/package.deb" >&2
  exit 2
fi

package="$1"
[[ "$(dpkg-deb --fsys-tarfile "${package}" | tar -tf - |
  grep -Ec '/share/doc/kitti-pointcloud-tools/THIRD_PARTY_NOTICES\.md$')" == 1 ]]
apt-get update
apt-get install -y --no-install-recommends \
  "${package}" libgl1-mesa-dri xvfb

for executable in \
  kpt_gui kpt_viewer kpt_player kpt_convert kpt_batch_convert kpt_info kpt_render; do
  command -v "${executable}" >/dev/null
done

xvfb-run -a kpt_gui --smoke-test
kpt_viewer --help >/dev/null
kpt_player --help >/dev/null
kpt_convert --help >/dev/null
kpt_batch_convert --help >/dev/null
kpt_info --help >/dev/null
kpt_render --help >/dev/null
