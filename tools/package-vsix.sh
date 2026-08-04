#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
vscode_root="${repository_root}/vscode"
artifact_directory="${repository_root}/artifacts"

for command_name in cmake node npm unzip; do
  command -v "${command_name}" >/dev/null || {
    echo "${command_name} is required" >&2
    exit 1
  }
done
if [[ -z "${EMSDK:-}" || ! -f "${EMSDK}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" ]]; then
  echo "EMSDK must point to an activated Emscripten 6.0.5 SDK" >&2
  exit 1
fi
if [[ "$(emcc --version | head -n 1)" != *"6.0.5"* ]]; then
  echo "Emscripten 6.0.5 is required" >&2
  emcc --version >&2
  exit 1
fi

"${repository_root}/tools/check-version.sh"
version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
vsix="${artifact_directory}/kitti-pointcloud-tools-${version}.vsix"

npm --prefix "${vscode_root}" ci
cmake -E make_directory "${artifact_directory}"
rm -f "${vsix}"
(
  cd "${vscode_root}"
  "${vscode_root}/node_modules/.bin/vsce" package --out "${vsix}"
)
test -s "${vsix}"

contents="$(unzip -Z1 "${vsix}")"
for required in \
  extension/package.json \
  extension/dist/extension.js \
  extension/dist/webview.js \
  extension/dist/decoder.worker.js \
  extension/dist/kpt_decoder.wasm \
  extension/readme.md \
  extension/LICENSE.txt; do
  if ! grep -Fxq "${required}" <<< "${contents}"; then
    echo "VSIX lacks ${required}" >&2
    exit 1
  fi
done
if grep -E '(^|/)(tests|src|generated|webview)/|\.map$' <<< "${contents}"; then
  echo "VSIX contains excluded development files" >&2
  exit 1
fi
echo "Built ${vsix}"
