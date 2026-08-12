macro(run_conan)
  message(FATAL_ERROR
    "Legacy Conan bootstrap was removed: it downloaded unverified code from "
    "retired Bintray infrastructure. Use the checked-in vcpkg manifest and "
    "a pinned toolchain preset instead.")
endmacro()
