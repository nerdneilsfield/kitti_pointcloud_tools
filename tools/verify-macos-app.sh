#!/usr/bin/env bash
set -euo pipefail

if (($# != 2)); then
  echo "usage: $0 'KPT Workbench.app' arm64|x86_64" >&2
  exit 2
fi
if [[ "$(uname -s)" != Darwin ]] || [[ ! "$2" =~ ^(arm64|x86_64)$ ]]; then
  echo "macOS verification requires Darwin and arm64|x86_64" >&2
  exit 2
fi

app="$(cd "$1" && pwd)"
expected_native_architecture="$2"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
contents="${app}/Contents"
macos="${contents}/MacOS"
frameworks="${contents}/Frameworks"
resources="${contents}/Resources"

[[ "$(uname -m)" == "${expected_native_architecture}" ]]
[[ "$(plutil -extract CFBundleShortVersionString raw \
  "${contents}/Info.plist")" == "${version}" ]]
[[ "$(plutil -extract CFBundleVersion raw \
  "${contents}/Info.plist")" == "${version}" ]]
test -s "${resources}/kpt_point_shaders.metallib"
test -s "${resources}/KPT.icns"
test -s "${resources}/THIRD_PARTY_NOTICES.md"
test ! -e "${macos}/kpt_point_shaders.metallib"
if file -b "${resources}/kpt_point_shaders.metallib" | grep -q 'Mach-O'; then
  echo "Metal shader library must be a bundle resource, not Mach-O code" >&2
  exit 1
fi

macho_dependencies() {
  otool -L "$1" | awk '/^[[:space:]]/ { print $1 }'
}

for binary in "${macos}/"* "${frameworks}/"*; do
  architectures="$(lipo -archs "${binary}")"
  [[ " ${architectures} " == *" arm64 "* ]]
  [[ " ${architectures} " == *" x86_64 "* ]]
  if macho_dependencies "${binary}" |
      grep -Ev '^(/System/Library/|/usr/lib/|@rpath/)' | grep -q .; then
    echo "non-bundle dependency remains in ${binary}" >&2
    otool -L "${binary}" >&2
    exit 1
  fi
done

codesign --verify --deep --strict --verbose=2 "${app}"
for executable in \
  kpt_gui kpt_viewer kpt_player kpt_convert kpt_batch_convert kpt_info kpt_render; do
  test -x "${macos}/${executable}"
  "${macos}/${executable}" --help >/dev/null
done
"${macos}/kpt_gui" --smoke-test
echo "Verified ${app} on ${expected_native_architecture}"
