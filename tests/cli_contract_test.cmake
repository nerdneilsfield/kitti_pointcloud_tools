if(NOT DEFINED KPT_INFO)
  message(FATAL_ERROR "KPT_INFO is required")
endif()

execute_process(COMMAND "${KPT_INFO}" "${CMAKE_CURRENT_LIST_DIR}/missing.pcd"
  RESULT_VARIABLE info_status OUTPUT_QUIET ERROR_QUIET)
if(info_status EQUAL 0)
  message(FATAL_ERROR "kpt_info reported success for a missing input")
endif()

if(DEFINED KPT_RENDER)
  execute_process(
    COMMAND "${KPT_RENDER}" "${CMAKE_CURRENT_LIST_DIR}/../test/data/tiny.xyz"
            -o "${CMAKE_CURRENT_BINARY_DIR}/cli-contract" --views ""
    RESULT_VARIABLE empty_views_status OUTPUT_QUIET ERROR_QUIET)
  if(empty_views_status EQUAL 0)
    message(FATAL_ERROR "kpt_render accepted an empty view list")
  endif()

  file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/cli-contract_front.png")
  execute_process(
    COMMAND "${KPT_RENDER}" "${CMAKE_CURRENT_LIST_DIR}/../test/data/tiny.xyz"
            -o "${CMAKE_CURRENT_BINARY_DIR}/cli-contract"
            --views front,front --overwrite
    RESULT_VARIABLE duplicate_status OUTPUT_QUIET ERROR_QUIET)
  if(NOT duplicate_status EQUAL 0 OR
     NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/cli-contract_front.png")
    message(FATAL_ERROR "kpt_render did not normalize duplicate views")
  endif()
  file(REMOVE "${CMAKE_CURRENT_BINARY_DIR}/cli-contract_front.png")
endif()
