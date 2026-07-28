if(NOT DEFINED PC_PLAYER OR NOT DEFINED SOURCE_DIR OR NOT DEFINED WORK_DIR)
  message(FATAL_ERROR "PC_PLAYER, SOURCE_DIR and WORK_DIR are required")
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}/输入" "${WORK_DIR}/空目录")
file(COPY "${SOURCE_DIR}/data/000123.pcd" DESTINATION "${WORK_DIR}/输入")

execute_process(
  COMMAND "${PC_PLAYER}" --help unexpected extra
  RESULT_VARIABLE help_result
  OUTPUT_QUIET ERROR_VARIABLE help_error)
if(NOT help_result EQUAL 0)
  message(FATAL_ERROR "help compatibility failed: ${help_error}")
endif()

execute_process(
  COMMAND "${PC_PLAYER}" -i "${WORK_DIR}/输入" -g "*.pcd"
          --snapshot "${WORK_DIR}/输出/快照" --snapshot-views front
          --snapshot-w 32 --snapshot-h 24
  RESULT_VARIABLE snapshot_result
  OUTPUT_VARIABLE snapshot_output ERROR_VARIABLE snapshot_error)
if(NOT snapshot_result EQUAL 0)
  message(FATAL_ERROR
    "snapshot command failed: ${snapshot_output}${snapshot_error}")
endif()
if(NOT EXISTS "${WORK_DIR}/输出/快照_000123_front.png")
  message(FATAL_ERROR "snapshot output was not created")
endif()

if(HAS_GUI)
  # Legacy SequencePlayer returned success immediately for an empty sequence.
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E env --unset=DISPLAY
            "${PC_PLAYER}" -i "${WORK_DIR}/空目录"
    RESULT_VARIABLE empty_result
    TIMEOUT 5
    OUTPUT_QUIET ERROR_VARIABLE empty_error)
  if(NOT empty_result EQUAL 0)
    message(FATAL_ERROR "empty sequence compatibility failed: ${empty_error}")
  endif()
else()
  execute_process(
    COMMAND "${PC_PLAYER}" -i "${WORK_DIR}/输入"
    RESULT_VARIABLE interactive_result
    OUTPUT_QUIET ERROR_VARIABLE interactive_error)
  if(interactive_result EQUAL 0 OR
     NOT interactive_error MATCHES "interactive pc_player requires")
    message(FATAL_ERROR
      "GUI-off interactive diagnostic failed: ${interactive_error}")
  endif()
endif()

file(REMOVE_RECURSE "${WORK_DIR}")
