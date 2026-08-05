#!/usr/bin/env bash
set -euo pipefail

if (($# != 0 && $# != 2)); then
  echo "usage: $0 [ARM64_STAGE X86_64_STAGE]" >&2
  exit 2
fi
if [[ "$(uname -s)" != Darwin ]]; then
  echo "macOS packaging must run on macOS" >&2
  exit 1
fi

for command_name in cmake lipo otool install_name_tool codesign hdiutil plutil; do
  command -v "${command_name}" >/dev/null || {
    echo "${command_name} is required" >&2
    exit 1
  }
done

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"${repository_root}/tools/check-version.sh"
version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"

if (($# == 0)); then
  : "${VCPKG_ROOT:?VCPKG_ROOT is required}"
  arm_build="${repository_root}/build/macos-arm64-vcpkg-release"
  x64_build="${repository_root}/build/macos-x64-vcpkg-release"
  cmake --preset macos-arm64-vcpkg-release -DKPT_ENABLE_PACKAGING=ON
  cmake --build --preset macos-arm64-vcpkg-release --parallel
  cmake --preset macos-x64-vcpkg-release -DKPT_ENABLE_PACKAGING=ON
  cmake --build --preset macos-x64-vcpkg-release --parallel
  case "$(uname -m)" in
    arm64) ctest --preset macos-arm64-vcpkg-release ;;
    x86_64) ctest --preset macos-x64-vcpkg-release ;;
  esac
  arm_stage="${repository_root}/build/package-macos-arm64-stage"
  x64_stage="${repository_root}/build/package-macos-x86_64-stage"
  "${repository_root}/packaging/macos/stage-arch.sh" \
    arm64 "${arm_build}" "${arm_stage}"
  "${repository_root}/packaging/macos/stage-arch.sh" \
    x86_64 "${x64_build}" "${x64_stage}"
else
  arm_stage="$(cd "$1" && pwd)"
  x64_stage="$(cd "$2" && pwd)"
fi

[[ "$(tr -d '[:space:]' < "${arm_stage}/ARCHITECTURE")" == arm64 ]]
[[ "$(tr -d '[:space:]' < "${x64_stage}/ARCHITECTURE")" == x86_64 ]]
[[ "$(tr -d '[:space:]' < "${arm_stage}/VERSION")" == "${version}" ]]
[[ "$(tr -d '[:space:]' < "${x64_stage}/VERSION")" == "${version}" ]]

artifact_directory="${repository_root}/artifacts"
package_root="${repository_root}/build/package-macos-universal"
app="${package_root}/KPT Workbench.app"
contents="${app}/Contents"
frameworks="${contents}/Frameworks"
macos="${contents}/MacOS"
resources="${contents}/Resources"
dmg_root="${package_root}/dmg-root"
dmg="${artifact_directory}/kitti-pointcloud-tools-${version}-macos-universal.dmg"

cmake -E remove_directory "${package_root}"
cmake -E make_directory \
  "${frameworks}" "${macos}" "${resources}" "${dmg_root}"
cp "${arm_stage}/Info.plist" "${contents}/Info.plist"
cp "${arm_stage}/resources/kpt_point_shaders.metallib" "${resources}/"
cp "${repository_root}/LICENSE" "${repository_root}/README.md" \
  "${repository_root}/README.zh-CN.md" "${resources}/"

merge_macho() {
  local arm_file="$1"
  local x64_file="$2"
  local destination="$3"
  test -f "${arm_file}"
  test -f "${x64_file}"
  lipo -create "${arm_file}" "${x64_file}" -output "${destination}"
  chmod u+w "${destination}"
}

macho_dependencies() {
  local binary="$1"
  # Universal Mach-O output contains one unindented header per architecture.
  # Dependency records are the indented lines beneath those headers.
  otool -L "${binary}" | awk '/^[[:space:]]/ { print $1 }'
}

for executable in pc_gui pc_viewer pc_player pc_convert pc_batch_convert pc_render; do
  merge_macho "${arm_stage}/bin/${executable}" \
    "${x64_stage}/bin/${executable}" "${macos}/${executable}"
  chmod +x "${macos}/${executable}"
done

dylib_names=()
seen_dylib_names="|"
for dylib in "${arm_stage}/lib/"* "${x64_stage}/lib/"*; do
  [[ -f "${dylib}" ]] || continue
  dylib_name="$(basename "${dylib}")"
  if [[ "${seen_dylib_names}" != *"|${dylib_name}|"* ]]; then
    dylib_names+=("${dylib_name}")
    seen_dylib_names+="${dylib_name}|"
  fi
done
for dylib_name in "${dylib_names[@]}"; do
  merge_macho "${arm_stage}/lib/${dylib_name}" \
    "${x64_stage}/lib/${dylib_name}" "${frameworks}/${dylib_name}"
done

rewrite_dependencies() {
  local binary="$1"
  if [[ "${binary}" == *.dylib ]]; then
    install_name_tool -id "@rpath/$(basename "${binary}")" "${binary}"
  else
    if ! otool -l "${binary}" | grep -A2 LC_RPATH | grep -q \
      '@executable_path/../Frameworks'; then
      install_name_tool -add_rpath '@executable_path/../Frameworks' "${binary}"
    fi
  fi
  while IFS= read -r dependency; do
    case "${dependency}" in
      /System/Library/*|/usr/lib/*) continue ;;
    esac
    dependency_name="$(basename "${dependency}")"
    if [[ -f "${frameworks}/${dependency_name}" ]]; then
      install_name_tool -change "${dependency}" \
        "@rpath/${dependency_name}" "${binary}"
    fi
  done < <(macho_dependencies "${binary}")
}

for binary in "${macos}/"* "${frameworks}/"*; do
  rewrite_dependencies "${binary}"
done

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

plutil -replace CFBundleDisplayName -string "KPT Workbench" \
  "${contents}/Info.plist"
plutil -replace CFBundleShortVersionString -string "${version}" \
  "${contents}/Info.plist"
plutil -replace CFBundleVersion -string "${version}" \
  "${contents}/Info.plist"
[[ "$(plutil -extract CFBundleShortVersionString raw \
  "${contents}/Info.plist")" == "${version}" ]]
[[ "$(plutil -extract CFBundleVersion raw \
  "${contents}/Info.plist")" == "${version}" ]]
for dylib in "${frameworks}/"*; do
  codesign --force --sign - "${dylib}"
done
for executable in \
  pc_viewer pc_player pc_convert pc_batch_convert pc_render; do
  codesign --force --sign - "${macos}/${executable}"
done
# Signing the bundle signs its CFBundleExecutable (pc_gui) and seals resources.
# Nested code must already be signed before this outermost inside-out step.
codesign --force --sign - "${app}"
codesign --verify --deep --strict "${app}"

cp -R "${app}" "${dmg_root}/"
ln -s /Applications "${dmg_root}/Applications"
cmake -E make_directory "${artifact_directory}"
rm -f "${dmg}"
hdiutil create -quiet -fs HFS+ -volname "KPT Workbench ${version}" \
  -srcfolder "${dmg_root}" -format UDZO "${dmg}"
test -s "${dmg}"
echo "Built ${dmg}"
