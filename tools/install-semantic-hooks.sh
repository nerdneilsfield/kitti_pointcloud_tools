#!/usr/bin/env bash
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
hooks="${root}/.git/hooks"
mkdir -p "${hooks}"

cat > "${hooks}/pre-commit" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
root="$(git rev-parse --show-toplevel)"
node "${root}/tools/semantic-fingerprint.mjs"
EOF
chmod +x "${hooks}/pre-commit"

for name in post-merge post-rewrite; do
  cat > "${hooks}/${name}" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
root="$(git rev-parse --show-toplevel)"
rm -rf "${root}/.git/kpt-semantic-cache"
echo "Semantic cache invalidated after history rewrite. Run: node tools/semantic-fingerprint.mjs --bootstrap"
EOF
  chmod +x "${hooks}/${name}"
done

echo "Installed semantic hooks. Bootstrap once: node tools/semantic-fingerprint.mjs --bootstrap"
