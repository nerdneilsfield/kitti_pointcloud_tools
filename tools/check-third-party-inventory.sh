#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repository_root}"

check_hash() {
  local expected="$1"
  local file="$2"
  local actual
  actual="$(sha256sum "${file}" | awk '{print $1}')"
  if [[ "${actual}" != "${expected}" ]]; then
    echo "third-party inventory hash mismatch: ${file}" >&2
    echo "expected ${expected}; got ${actual}" >&2
    exit 1
  fi
  grep -Fq "\`${expected}\`" THIRD_PARTY_NOTICES.md
}

check_hash e7d3c7f17ffc991192c996e7614165d87c0522eb4f7f9d87b1c98d0536c102c8 \
  third_party/catch2/catch2/catch.hpp
check_hash d2f503cce73828374700610d46395d56723d27ceaa5c465058304dfbd230033c \
  third_party/eigen/Eigen/src/Core/util/Macros.h
check_hash 90801584c2a28033c156ca418481934a6060478120b7b7283fe9d2ef09295d7c \
  third_party/glad/include/glad/gl.h
check_hash 149704059b5d0bf551637e50042dd4de9c2cae921021f6636298911e3a5f9462 \
  third_party/glfw-3.4/LICENSE.md
check_hash 173506a2d6f7fb67990d257fb2507f188690eca39060c39469ae7bef43aae2a3 \
  third_party/imgui-1.92.8-docking/LICENSE.txt
check_hash 2b6fa0b70f219e70cb1b7314aaf0904293bf24255c0a5d30f0169ca87e9e8912 \
  third_party/ImGuiFileDialog-0.6.8/LICENSE
check_hash 4b74d7d6095f0d2bfee2c7f4ed718307089fe9736af0d07f523bd6d38336e25e \
  third_party/popl/LICENSE
check_hash a7241a379cb98d37788f82c2fc2e484c3ae85dae881fd3406f0637644850926f \
  third_party/spdlog/LICENSE
check_hash 594c2fe35d49488b4382dbfaec8f98366defca819d916ac95becf3e75f4200b3 \
  third_party/stb/stb_image.h
check_hash 7287d83639baa0be111e325ba1321c19dcd04fc7344d50461694a4d61f687b8e \
  third_party/toml/toml/toml.h

echo "Third-party inventory matches vendored sources."
