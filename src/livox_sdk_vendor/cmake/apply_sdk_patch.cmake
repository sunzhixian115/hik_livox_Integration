find_program(GIT_EXECUTABLE git REQUIRED)

execute_process(
  COMMAND ${GIT_EXECUTABLE} apply --check ${PATCH_FILE}
  WORKING_DIRECTORY ${SDK_SOURCE_DIR}
  RESULT_VARIABLE check_result
  ERROR_QUIET
)

if(check_result EQUAL 0)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} apply ${PATCH_FILE}
    WORKING_DIRECTORY ${SDK_SOURCE_DIR}
    RESULT_VARIABLE apply_result
  )
  if(NOT apply_result EQUAL 0)
    message(FATAL_ERROR "Failed to apply Livox SDK route-aware IP patch")
  endif()
else()
  execute_process(
    COMMAND ${GIT_EXECUTABLE} apply --reverse --check ${PATCH_FILE}
    WORKING_DIRECTORY ${SDK_SOURCE_DIR}
    RESULT_VARIABLE reverse_result
    ERROR_QUIET
  )
  if(NOT reverse_result EQUAL 0)
    message(FATAL_ERROR "Livox SDK source does not match route-aware IP patch")
  endif()
endif()
