#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
expected_version="${1:-${version}}"

if [[ ! "${version}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "VERSION is not an X.Y.Z semantic version: ${version}" >&2
  exit 1
fi
if [[ "${version}" != "${expected_version}" ]]; then
  echo "VERSION mismatch: expected ${expected_version}, found ${version}" >&2
  exit 1
fi

vcpkg_version="$(sed -n 's/^[[:space:]]*"version-string":[[:space:]]*"\([^"]*\)".*/\1/p' "${repository_root}/vcpkg.json" | head -n 1)"
package_version="$(node -e \
  'process.stdout.write(require(process.argv[1]).version)' \
  "${repository_root}/vscode/package.json")"
lock_versions="$(node -e '
const lock = require(process.argv[1]);
const versions = [lock.version, lock.packages?.[""]?.version];
if (versions.some((value) => typeof value !== "string")) process.exit(2);
process.stdout.write(versions.join("\n"));
' "${repository_root}/vscode/package-lock.json")"

failure=0
check_value() {
  local label="$1"
  local actual="$2"
  if [[ "${actual}" != "${version}" ]]; then
    echo "${label} version mismatch: expected ${version}, found ${actual}" >&2
    failure=1
  fi
}

check_value "vcpkg.json" "${vcpkg_version}"
check_value "vscode/package.json" "${package_version}"
while IFS= read -r lock_version; do
  check_value "vscode/package-lock.json" "${lock_version}"
done <<< "${lock_versions}"

if ((failure != 0)); then
  exit 1
fi
echo "Global version ${version} is consistent."
