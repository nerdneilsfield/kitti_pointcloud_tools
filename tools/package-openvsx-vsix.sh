#!/usr/bin/env bash
set -euo pipefail

# Open VSX and Microsoft's Marketplace use the same public extension identity.
# Do not rewrite the VSIX publisher: dengqi.pointcloud-tools is published to both.

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
source_vsix="${1:-${repository_root}/artifacts/kitti-pointcloud-tools-${version}.vsix}"
target_vsix="${2:-${repository_root}/artifacts/dengqi.pointcloud-tools-${version}.vsix}"

for command_name in node unzip zip; do
  command -v "${command_name}" >/dev/null || {
    echo "${command_name} is required" >&2
    exit 1
  }
done
test -s "${source_vsix}" || {
  echo "source VSIX is missing: ${source_vsix}" >&2
  exit 1
}

scratch="$(mktemp -d)"
trap 'rm -rf "${scratch}"' EXIT
unzip -q "${source_vsix}" -d "${scratch}"

node - "${scratch}/extension/package.json" "${scratch}/extension.vsixmanifest" "${version}" <<'NODE'
const fs = require("node:fs");
const [manifestPath, vsixManifestPath, expectedVersion] = process.argv.slice(2);
const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
if (manifest.publisher !== "dengqi" || manifest.name !== "pointcloud-tools" ||
    manifest.version !== expectedVersion) {
  throw new Error("unexpected source VSIX identity");
}
const vsixManifest = fs.readFileSync(vsixManifestPath, "utf8");
const marketplaceIdentity = 'Id="pointcloud-tools" Version="' + expectedVersion +
  '" Publisher="dengqi"';
if (!vsixManifest.includes(marketplaceIdentity)) {
  throw new Error("unexpected source VSIX manifest identity");
}
fs.writeFileSync(manifestPath, `${JSON.stringify(manifest, null, 2)}\n`);
fs.writeFileSync(vsixManifestPath, vsixManifest);
NODE

mkdir -p "$(dirname "${target_vsix}")"
target_vsix="$(cd "$(dirname "${target_vsix}")" && pwd)/$(basename "${target_vsix}")"
rm -f "${target_vsix}"
(
  cd "${scratch}"
  # Open VSX rejects UID/GID and timestamp extra fields. Keep the VSIX payload
  # portable by emitting only standard ZIP entries.
  zip -X -q -r "${target_vsix}" .
)
test -s "${target_vsix}"

unzip -p "${target_vsix}" extension/package.json | node - "${version}" <<'NODE'
let input = "";
process.stdin.on("data", (chunk) => input += chunk);
process.stdin.on("end", () => {
  const manifest = JSON.parse(input);
  if (manifest.publisher !== "dengqi" ||
      manifest.name !== "pointcloud-tools" ||
      manifest.version !== process.argv[2]) {
    throw new Error("Open VSX package identity validation failed");
  }
});
NODE
if ! unzip -p "${target_vsix}" extension.vsixmanifest | grep -Fq \
  "Id=\"pointcloud-tools\" Version=\"${version}\" Publisher=\"dengqi\""; then
  echo "Open VSX VSIX manifest identity validation failed" >&2
  exit 1
fi

echo "Built ${target_vsix}"
