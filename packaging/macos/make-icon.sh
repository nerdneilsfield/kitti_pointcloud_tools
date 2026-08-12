#!/usr/bin/env bash
set -euo pipefail

if (($# != 2)); then
  echo "usage: $0 SOURCE.svg DESTINATION.icns" >&2
  exit 2
fi
for command_name in qlmanage sips iconutil; do
  command -v "${command_name}" >/dev/null || {
    echo "${command_name} is required" >&2
    exit 1
  }
done

source_svg="$1"
destination="$2"
work_directory="$(mktemp -d)"
trap 'rm -rf "${work_directory}"' EXIT
iconset="${work_directory}/KPT.iconset"
mkdir -p "${iconset}"
qlmanage -t -s 1024 -o "${work_directory}" "${source_svg}" >/dev/null
source_png="${work_directory}/$(basename "${source_svg}").png"
test -s "${source_png}"
for size in 16 32 128 256 512; do
  sips -s format png -z "${size}" "${size}" "${source_png}" \
    --out "${iconset}/icon_${size}x${size}.png" >/dev/null
  double_size=$((size * 2))
  sips -s format png -z "${double_size}" "${double_size}" "${source_png}" \
    --out "${iconset}/icon_${size}x${size}@2x.png" >/dev/null
done
iconutil -c icns "${iconset}" -o "${destination}"
test -s "${destination}"
