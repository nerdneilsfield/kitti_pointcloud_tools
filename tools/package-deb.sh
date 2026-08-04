#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)) || [[ ! "$1" =~ ^(22\.04|24\.04)$ ]]; then
  echo "usage: $0 22.04|24.04" >&2
  exit 2
fi
command -v docker >/dev/null || {
  echo "docker is required for reproducible Ubuntu packaging" >&2
  exit 1
}

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ubuntu_version="$1"
image="kpt-package-ubuntu:${ubuntu_version}"

"${repository_root}/tools/check-version.sh"
docker build \
  --build-arg "UBUNTU_VERSION=${ubuntu_version}" \
  --tag "${image}" \
  --file "${repository_root}/packaging/linux/Dockerfile" \
  "${repository_root}"
docker run --rm \
  --user "$(id -u):$(id -g)" \
  --volume "${repository_root}:/workspace" \
  "${image}" \
  "./packaging/linux/build-deb.sh" "${ubuntu_version}"

version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
filename="kitti-pointcloud-tools_${version}-1~ubuntu${ubuntu_version}_amd64.deb"
docker run --rm \
  --volume "${repository_root}/artifacts:/packages:ro" \
  --volume "${repository_root}/packaging/linux/verify-deb.sh:/verify-deb.sh:ro" \
  "ubuntu:${ubuntu_version}" \
  /verify-deb.sh "/packages/${filename}"
