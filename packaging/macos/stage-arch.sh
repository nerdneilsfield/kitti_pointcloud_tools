#!/usr/bin/env bash
set -euo pipefail

if (($# != 3)) || [[ ! "$1" =~ ^(arm64|x86_64)$ ]]; then
  echo "usage: $0 arm64|x86_64 BUILD_DIRECTORY STAGE_DIRECTORY" >&2
  exit 2
fi

architecture="$1"
build_directory="$(cd "$2" && pwd)"
stage_directory="$3"
repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

for command_name in otool lipo; do
  command -v "${command_name}" >/dev/null || {
    echo "${command_name} is required" >&2
    exit 1
  }
done

cmake -E remove_directory "${stage_directory}"
cmake -E make_directory \
  "${stage_directory}/bin" \
  "${stage_directory}/lib" \
  "${stage_directory}/resources"

executables=(pc_gui pc_viewer pc_player pc_convert pc_batch_convert pc_render)
for executable in "${executables[@]}"; do
  source_path="${build_directory}/${executable}"
  if [[ "${executable}" == pc_gui ]]; then
    source_path="${build_directory}/pc_gui.app/Contents/MacOS/pc_gui"
    cp "${build_directory}/pc_gui.app/Contents/Info.plist" \
      "${stage_directory}/Info.plist"
  fi
  test -x "${source_path}"
  cp "${source_path}" "${stage_directory}/bin/${executable}"
done
cp "${build_directory}/kpt_point_shaders.metallib" \
  "${stage_directory}/resources/kpt_point_shaders.metallib"
printf '%s\n' "${architecture}" > "${stage_directory}/ARCHITECTURE"
cp "${repository_root}/VERSION" "${stage_directory}/VERSION"

resolve_dependency() {
  local dependency="$1"
  if [[ -f "${dependency}" ]]; then
    printf '%s\n' "${dependency}"
    return
  fi
  local basename_value
  basename_value="$(basename "${dependency}")"
  local found
  found="$(find "${build_directory}/vcpkg_installed" \
    -path '*/lib/*' -not -path '*/debug/*' -type f \
    -name "${basename_value}" -print 2>/dev/null | head -n 1 || true)"
  if [[ -z "${found}" && -n "${VCPKG_ROOT:-}" ]]; then
    found="$(find "${VCPKG_ROOT}/installed" \
      -type f -name "${basename_value}" -print 2>/dev/null | head -n 1 || true)"
  fi
  if [[ -z "${found}" ]]; then
    echo "cannot resolve non-system dependency ${dependency}" >&2
    exit 1
  fi
  printf '%s\n' "${found}"
}

declare -a queue=("${stage_directory}/bin/"*)
inspected_names="|"
while ((${#queue[@]} > 0)); do
  current="${queue[0]}"
  queue=("${queue[@]:1}")
  current_name="$(basename "${current}")"
  if [[ "${inspected_names}" == *"|${current_name}|"* ]]; then
    continue
  fi
  inspected_names+="${current_name}|"
  while IFS= read -r dependency; do
    case "${dependency}" in
      /System/Library/*|/usr/lib/*) continue ;;
    esac
    resolved="$(resolve_dependency "${dependency}")"
    destination="${stage_directory}/lib/$(basename "${resolved}")"
    if [[ ! -f "${destination}" ]]; then
      cp "${resolved}" "${destination}"
      chmod u+w "${destination}"
      queue+=("${destination}")
    fi
  done < <(otool -L "${current}" | tail -n +2 | awk '{print $1}')
done

for binary in "${stage_directory}/bin/"* "${stage_directory}/lib/"*; do
  if [[ -e "${binary}" ]]; then
    actual_architectures="$(lipo -archs "${binary}")"
    if [[ " ${actual_architectures} " != *" ${architecture} "* ]]; then
      echo "${binary} lacks ${architecture}: ${actual_architectures}" >&2
      exit 1
    fi
  fi
done
