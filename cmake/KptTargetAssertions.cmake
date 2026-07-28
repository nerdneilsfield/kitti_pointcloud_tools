include_guard(GLOBAL)

function(kpt_validate_gui_backend)
  cmake_parse_arguments(
    KPT_BACKEND
    ""
    "REQUESTED;OUT_VAR"
    ""
    ${ARGN}
  )

  if(NOT KPT_BACKEND_REQUESTED)
    message(FATAL_ERROR "kpt_validate_gui_backend requires REQUESTED")
  endif()
  if(NOT KPT_BACKEND_OUT_VAR)
    message(FATAL_ERROR "kpt_validate_gui_backend requires OUT_VAR")
  endif()

  string(TOLOWER "${KPT_BACKEND_REQUESTED}" requested)
  if(NOT requested IN_LIST _kpt_allowed_gui_backends)
    message(FATAL_ERROR
      "Unsupported KPT GUI backend '${KPT_BACKEND_REQUESTED}'. "
      "Allowed values: auto, opengl, metal")
  endif()

  set("${KPT_BACKEND_OUT_VAR}" "${requested}" PARENT_SCOPE)
endfunction()

function(kpt_resolve_gui_backend)
  cmake_parse_arguments(
    KPT_BACKEND
    "ALLOW_APPLE_OPENGL"
    "REQUESTED;SYSTEM;OUT_VAR"
    ""
    ${ARGN}
  )

  if(NOT KPT_BACKEND_SYSTEM)
    message(FATAL_ERROR "kpt_resolve_gui_backend requires SYSTEM")
  endif()
  if(NOT KPT_BACKEND_OUT_VAR)
    message(FATAL_ERROR "kpt_resolve_gui_backend requires OUT_VAR")
  endif()

  kpt_validate_gui_backend(
    REQUESTED "${KPT_BACKEND_REQUESTED}"
    OUT_VAR requested)

  if(KPT_BACKEND_SYSTEM STREQUAL "Linux"
     OR KPT_BACKEND_SYSTEM STREQUAL "Windows")
    if(requested STREQUAL "auto" OR requested STREQUAL "opengl")
      set(active_backend opengl)
    else()
      message(FATAL_ERROR
        "Backend '${requested}' is not supported on ${KPT_BACKEND_SYSTEM}")
    endif()
  elseif(KPT_BACKEND_SYSTEM STREQUAL "Darwin")
    if(requested STREQUAL "auto" OR requested STREQUAL "metal")
      set(active_backend metal)
    elseif(requested STREQUAL "opengl" AND KPT_BACKEND_ALLOW_APPLE_OPENGL)
      set(active_backend opengl)
    else()
      message(FATAL_ERROR
        "Backend '${requested}' is not enabled on Darwin; "
        "use metal or explicitly enable the migration-only OpenGL backend")
    endif()
  else()
    message(FATAL_ERROR
      "KPT GUI backend selection does not support system "
      "'${KPT_BACKEND_SYSTEM}'")
  endif()

  set("${KPT_BACKEND_OUT_VAR}" "${active_backend}" PARENT_SCOPE)
endfunction()

set(_kpt_allowed_gui_backends auto opengl metal)

function(kpt_assert_msvc_utf8 target)
  if(NOT MSVC)
    return()
  endif()
  get_target_property(options "${target}" INTERFACE_COMPILE_OPTIONS)
  if(NOT "/utf-8" IN_LIST options)
    message(FATAL_ERROR
      "${target} must publish /utf-8 for MSVC and clang-cl")
  endif()
endfunction()

function(kpt_assert_exactly_one_name)
  cmake_parse_arguments(KPT_ONE "" "ACTIVE" "CANDIDATES;AVAILABLE" ${ARGN})
  if(NOT KPT_ONE_ACTIVE IN_LIST KPT_ONE_CANDIDATES)
    message(FATAL_ERROR
      "Active target '${KPT_ONE_ACTIVE}' is not a backend candidate")
  endif()
  set(created 0)
  foreach(candidate IN LISTS KPT_ONE_CANDIDATES)
    if(candidate IN_LIST KPT_ONE_AVAILABLE)
      math(EXPR created "${created} + 1")
    endif()
  endforeach()
  if(NOT created EQUAL 1)
    message(FATAL_ERROR
      "Expected exactly one created backend target, found ${created}: "
      "${KPT_ONE_AVAILABLE}")
  endif()
  if(NOT KPT_ONE_ACTIVE IN_LIST KPT_ONE_AVAILABLE)
    message(FATAL_ERROR
      "Selected backend target '${KPT_ONE_ACTIVE}' was not created")
  endif()
endfunction()

function(kpt_assert_exactly_one_target)
  cmake_parse_arguments(KPT_ONE "" "ACTIVE" "CANDIDATES" ${ARGN})
  set(available)
  foreach(candidate IN LISTS KPT_ONE_CANDIDATES)
    if(TARGET "${candidate}")
      list(APPEND available "${candidate}")
    endif()
  endforeach()
  kpt_assert_exactly_one_name(
    ACTIVE "${KPT_ONE_ACTIVE}"
    CANDIDATES ${KPT_ONE_CANDIDATES}
    AVAILABLE ${available})
endfunction()

if(CMAKE_SCRIPT_MODE_FILE STREQUAL CMAKE_CURRENT_LIST_FILE)
  if(DEFINED KPT_EXACTLY_ONE_PROBE_ACTIVE)
    kpt_assert_exactly_one_name(
      ACTIVE "${KPT_EXACTLY_ONE_PROBE_ACTIVE}"
      CANDIDATES ${KPT_EXACTLY_ONE_PROBE_CANDIDATES}
      AVAILABLE ${KPT_EXACTLY_ONE_PROBE_AVAILABLE})
    return()
  endif()
  if(DEFINED KPT_ASSERTION_PROBE_REQUESTED)
    kpt_resolve_gui_backend(
      REQUESTED "${KPT_ASSERTION_PROBE_REQUESTED}"
      SYSTEM "${KPT_ASSERTION_PROBE_SYSTEM}"
      OUT_VAR probe_backend
    )
    return()
  endif()

  function(_kpt_assert_backend requested system expected)
    kpt_resolve_gui_backend(
      REQUESTED "${requested}"
      SYSTEM "${system}"
      OUT_VAR actual
      ${ARGN}
    )
    if(NOT actual STREQUAL expected)
      message(FATAL_ERROR
        "${system}/${requested}: expected '${expected}', got '${actual}'")
    endif()
  endfunction()

  kpt_validate_gui_backend(REQUESTED OpenGL OUT_VAR normalized_backend)
  if(NOT normalized_backend STREQUAL opengl)
    message(FATAL_ERROR
      "Backend validation did not normalize OpenGL to opengl")
  endif()

  _kpt_assert_backend(auto Linux opengl)
  _kpt_assert_backend(opengl Linux opengl)
  _kpt_assert_backend(auto Windows opengl)
  _kpt_assert_backend(opengl Windows opengl)
  _kpt_assert_backend(auto Darwin metal)
  _kpt_assert_backend(metal Darwin metal)
  _kpt_assert_backend(opengl Darwin opengl ALLOW_APPLE_OPENGL)

  foreach(case IN ITEMS "dx11|Linux" "metal|Windows" "opengl|Darwin")
    string(REPLACE "|" ";" fields "${case}")
    list(GET fields 0 requested)
    list(GET fields 1 system)
    execute_process(
      COMMAND
        "${CMAKE_COMMAND}"
        "-DKPT_ASSERTION_PROBE_REQUESTED=${requested}"
        "-DKPT_ASSERTION_PROBE_SYSTEM=${system}"
        -P "${CMAKE_CURRENT_LIST_FILE}"
      RESULT_VARIABLE result
      OUTPUT_QUIET
      ERROR_QUIET
    )
    if(result EQUAL 0)
      message(FATAL_ERROR
        "${system}/${requested}: expected backend resolution to fail")
    endif()
  endforeach()

  set(one_candidates backend_opengl backend_metal)
  kpt_assert_exactly_one_name(
    ACTIVE backend_opengl
    CANDIDATES ${one_candidates}
    AVAILABLE backend_opengl)
  foreach(case IN ITEMS
      "backend_dx11|backend_opengl"
      "backend_opengl|none"
      "backend_opengl|backend_opengl,backend_metal"
      "backend_metal|backend_opengl")
    string(REPLACE "|" ";" fields "${case}")
    list(GET fields 0 active)
    list(GET fields 1 available_csv)
    if(available_csv STREQUAL "none")
      set(available "")
    else()
      string(REPLACE "," ";" available "${available_csv}")
    endif()
    execute_process(
      COMMAND
        "${CMAKE_COMMAND}"
        "-DKPT_EXACTLY_ONE_PROBE_ACTIVE=${active}"
        "-DKPT_EXACTLY_ONE_PROBE_CANDIDATES=backend_opengl;backend_metal"
        "-DKPT_EXACTLY_ONE_PROBE_AVAILABLE=${available}"
        -P "${CMAKE_CURRENT_LIST_FILE}"
      RESULT_VARIABLE result
      OUTPUT_QUIET
      ERROR_QUIET)
    if(result EQUAL 0)
      message(FATAL_ERROR
        "Exactly-one target probe unexpectedly passed: ${case}")
    endif()
  endforeach()

  message(STATUS "KPT target assertion self-tests passed")
endif()
