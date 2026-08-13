#!/usr/bin/env bash
set -euo pipefail

# Open VSX namespace ownership is independent from Microsoft's Marketplace.
# Keep the public VS Code package as dengqi.pointcloud-tools and derive a
# second, byte-audited package for nerdneilsfield.pointcloud-tools.

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
source_vsix="${1:-${repository_root}/artifacts/kitti-pointcloud-tools-${version}.vsix}"
target_vsix="${2:-${repository_root}/artifacts/nerdneilsfield.pointcloud-tools-${version}.vsix}"

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

node - "${scratch}/extension/package.json" "${version}" <<'NODE'
const fs = require("node:fs");
const [manifestPath, expectedVersion] = process.argv.slice(2);
const manifest = JSON.parse(fs.readFileSync(manifestPath, "utf8"));
if (manifest.publisher !== "dengqi" || manifest.name !== "pointcloud-tools" ||
    manifest.version !== expectedVersion) {
  throw new Error("unexpected source VSIX identity");
}
manifest.publisher = "nerdneilsfield";
fs.writeFileSync(manifestPath, `${JSON.stringify(manifest, null, 2)}\n`);
NODE

mkdir -p "$(dirname "${target_vsix}")"
rm -f "${target_vsix}"
(
  cd "${scratch}"
  zip -q -r "${target_vsix}" .
)
test -s "${target_vsix}"

unzip -p "${target_vsix}" extension/package.json | node - "${version}" <<'NODE'
let input = "";
process.stdin.on("data", (chunk) => input += chunk);
process.stdin.on("end", () => {
  const manifest = JSON.parse(input);
  if (manifest.publisher !== "nerdneilsfield" ||
      manifest.name !== "pointcloud-tools" ||
      manifest.version !== process.argv[2]) {
    throw new Error("Open VSX package identity validation failed");
  }
});
NODE

echo "Built ${target_vsix}"
