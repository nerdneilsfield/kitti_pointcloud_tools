#!/usr/bin/env bash
set -euo pipefail

if (($# != 1)) || [[ ! "$1" =~ ^(22\.04|24\.04|26\.04)$ ]]; then
  echo "usage: $0 22.04|24.04|26.04" >&2
  exit 2
fi

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ubuntu_version="$1"
package_verify="${KPT_PACKAGE_VERIFY:-1}"
if [[ ! "${package_verify}" =~ ^(0|1)$ ]]; then
  echo "KPT_PACKAGE_VERIFY must be 0 or 1" >&2
  exit 2
fi
build_directory="${repository_root}/build/package-ubuntu${ubuntu_version}"
artifact_directory="${repository_root}/artifacts"
if [[ "${package_verify}" == 1 ]]; then
  build_tests=ON
else
  build_tests=OFF
fi

cmake -S "${repository_root}" -B "${build_directory}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_TOOLCHAIN_FILE="${repository_root}/cmake/toolchains/linux-system.cmake" \
  -DKPT_BUILD_GUI=ON \
  -DKPT_GUI_BACKEND=opengl \
  -DKPT_BUILD_TESTS="${build_tests}" \
  -DKPT_BUILD_RENDER=ON \
  -DENABLE_TOOLS=ON \
  -DKPT_ENABLE_PACKAGING=ON \
  -DKPT_PACKAGE_DISTRO="ubuntu${ubuntu_version}"
cmake --build "${build_directory}" --parallel
if [[ "${package_verify}" == 1 ]]; then
  test_home="${build_directory}/test-home"
  test_cache="${test_home}/.cache"
  cmake -E make_directory "${test_cache}"
  HOME="${test_home}" XDG_CACHE_HOME="${test_cache}" \
    xvfb-run -a ctest --test-dir "${build_directory}" --output-on-failure
fi
cmake -E make_directory "${artifact_directory}"
cpack --config "${build_directory}/CPackConfig.cmake" \
  -B "${artifact_directory}"

version="$(tr -d '[:space:]' < "${repository_root}/VERSION")"
package="${artifact_directory}/kitti-pointcloud-tools_${version}-1~ubuntu${ubuntu_version}_amd64.deb"
test -s "${package}"
inspection_directory="${build_directory}/package-inspection"
cmake -E remove_directory "${inspection_directory}"
cmake -E make_directory "${inspection_directory}"
dpkg-deb --extract "${package}" "${inspection_directory}"
test -s "${inspection_directory}/usr/share/icons/hicolor/scalable/apps/kpt-workbench.svg"
for size in 16 32 48 64 128 256; do
  test -s "${inspection_directory}/usr/share/icons/hicolor/${size}x${size}/apps/kpt-workbench.png"
done
for executable in "${inspection_directory}/usr/bin/"*; do
  if strings "${executable}" |
      grep -E -q '/workspace/|/build/package-ubuntu(22|24|26)\.04/'; then
    echo "build path leaked into ${executable}" >&2
    exit 1
  fi
done
dpkg-deb --info "${package}"
dpkg-deb --contents "${package}"
echo "Built ${package}"
