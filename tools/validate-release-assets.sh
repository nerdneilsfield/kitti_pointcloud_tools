#!/usr/bin/env bash
set -euo pipefail

if (($# != 2)); then
  echo "usage: $0 VERSION ARTIFACT_DIRECTORY" >&2
  exit 2
fi

version="$1"
artifact_directory="$(cd "$2" && pwd)"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${repository_root}/tools/check-version.sh" "${version}"

expected=(
  "kitti-pointcloud-tools_${version}-1~ubuntu22.04_amd64.deb"
  "kitti-pointcloud-tools_${version}-1~ubuntu24.04_amd64.deb"
  "kitti-pointcloud-tools_${version}-1~ubuntu26.04_amd64.deb"
  "kitti-pointcloud-tools-${version}-macos-universal.dmg"
  "kitti-pointcloud-tools-${version}-windows-x64.zip"
  "kitti-pointcloud-tools-${version}.vsix"
)

for filename in "${expected[@]}"; do
  test -s "${artifact_directory}/${filename}"
done
actual_count="$(find "${artifact_directory}" -maxdepth 1 -type f | wc -l)"
if [[ "${actual_count}" -ne "${#expected[@]}" ]]; then
  echo "expected ${#expected[@]} release assets, found ${actual_count}" >&2
  find "${artifact_directory}" -maxdepth 1 -type f -print >&2
  exit 1
fi

for ubuntu_version in 22.04 24.04 26.04; do
  package="${artifact_directory}/kitti-pointcloud-tools_${version}-1~ubuntu${ubuntu_version}_amd64.deb"
  [[ "$(dpkg-deb -f "${package}" Version)" == \
    "${version}-1~ubuntu${ubuntu_version}" ]]
  [[ "$(dpkg-deb -f "${package}" Architecture)" == amd64 ]]
done

windows_package="${artifact_directory}/kitti-pointcloud-tools-${version}-windows-x64.zip"
packaged_version="$(unzip -p "${windows_package}" \
  '*/share/doc/kitti-pointcloud-tools/VERSION' | tr -d '[:space:]')"
[[ "${packaged_version}" == "${version}" ]]
for executable in \
  pc_gui pc_viewer pc_player pc_convert pc_batch_convert pc_render; do
  [[ "$(unzip -Z1 "${windows_package}" | \
    grep -Ec "/bin/${executable}\.exe$")" == 1 ]]
done

vsix_package="${artifact_directory}/kitti-pointcloud-tools-${version}.vsix"
vsix_version="$(unzip -p "${vsix_package}" extension/package.json |
  node -e 'let value=""; process.stdin.on("data", chunk => value += chunk); process.stdin.on("end", () => process.stdout.write(JSON.parse(value).version));')"
[[ "${vsix_version}" == "${version}" ]]

printf '%s\n' "${expected[@]}"
