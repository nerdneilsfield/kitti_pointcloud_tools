#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)) || [[ ! "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "usage: $0 X.Y.Z" >&2
  exit 2
fi

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$1"

command -v npm >/dev/null || {
  echo "npm is required to update VS Code manifests" >&2
  exit 1
}

npm --prefix "${repository_root}/vscode" version "${version}" \
  --no-git-tag-version --allow-same-version >/dev/null
node -e '
const fs = require("node:fs");
const path = process.argv[1];
const version = process.argv[2];
const manifest = JSON.parse(fs.readFileSync(path, "utf8"));
manifest["version-string"] = version;
fs.writeFileSync(path, `${JSON.stringify(manifest, null, 2)}\n`);
' "${repository_root}/vcpkg.json" "${version}"
printf '%s\n' "${version}" > "${repository_root}/VERSION"

"${repository_root}/tools/check-version.sh" "${version}"
