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
  extension/dist/browser.js \
  extension/dist/webview.js \
  extension/dist/decoder.worker.js \
  extension/dist/converter.node.worker.js \
  extension/dist/converter.worker.js \
  extension/dist/kpt_decoder.wasm \
  extension/readme.md \
  extension/LICENSE.txt \
  extension/THIRD_PARTY_NOTICES.md; do
  if ! grep -Fxq "${required}" <<< "${contents}"; then
    echo "VSIX lacks ${required}" >&2
    exit 1
  fi
done
if grep -E '(^|/)(tests|src|generated|webview)/|\.map$' <<< "${contents}"; then
  echo "VSIX contains excluded development files" >&2
  exit 1
fi
unzip -p "${vsix}" extension/package.json | node -e '
let value = "";
process.stdin.on("data", (chunk) => value += chunk);
process.stdin.on("end", () => {
  const manifest = JSON.parse(value);
  const commands = new Set(
    (manifest.contributes?.commands ?? []).map((entry) => entry.command),
  );
  if (manifest.publisher !== "dengqi" || manifest.name !== "pointcloud-tools") {
    throw new Error("unexpected Marketplace extension identity");
  }
  if (manifest.browser !== "./dist/browser.js") {
    throw new Error("VSIX lacks browser extension entry");
  }
  const binaryEditor = manifest.contributes?.customEditors?.find(
    (entry) => entry.viewType === "kpt.binaryPointCloudViewer",
  );
  const binaryPatterns = new Set(
    (binaryEditor?.selector ?? []).map((entry) => entry.filenamePattern),
  );
  if (binaryEditor?.priority !== "option" ||
      binaryPatterns.size !== 2 ||
      !binaryPatterns.has("*.bin") ||
      !binaryPatterns.has("*.npy")) {
    throw new Error("point-cloud custom editor must remain opt-in");
  }
  for (const command of [
    "kpt.openPointCloud", "kpt.openSequence", "kpt.convertPointCloud",
  ]) {
    if (!commands.has(command)) throw new Error(`VSIX lacks ${command}`);
  }
});
'
echo "Built ${vsix}"
