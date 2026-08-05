#!/usr/bin/env bash
set -euo pipefail

status=0
while IFS= read -r line; do
  reference="${line#*@}"
  if [[ "${reference}" == "${line}" ||
        "${reference}" =~ ^[[:space:]]*\./ ]]; then
    continue
  fi
  sha="${reference%%[[:space:]#]*}"
  if [[ ! "${sha}" =~ ^[0-9a-fA-F]{40}$ ]]; then
    printf 'unpinned GitHub Action: %s\n' "${line}" >&2
    status=1
  fi
done < <(rg -n --glob '*.yml' --glob '*.yaml' '^[[:space:]-]*uses:' .github)

exit "${status}"
