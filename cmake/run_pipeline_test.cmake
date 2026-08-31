# Runs the pipeline from the repository root and compares every file it writes
# against the reference output the repo ships.
#
# The R26 test had a `make check` that told you outright whether your output was
# right. This repo dropped it, so the only way to know was to diff four files by
# hand. This puts that check back, driven by ctest.
#
# Invoked as:
#   cmake -DEXE=<queue_test> -DROOT=<repo root> -P run_pipeline_test.cmake

if(NOT DEFINED EXE OR NOT DEFINED ROOT)
  message(FATAL_ERROR "EXE and ROOT must both be defined")
endif()

# The program resolves input/ and result/ relative to the working directory, so
# it has to run from the repository root rather than from build/.
execute_process(
  COMMAND "${EXE}"
  WORKING_DIRECTORY "${ROOT}"
  RESULT_VARIABLE run_result
  OUTPUT_VARIABLE run_output
  ERROR_VARIABLE run_error
)

if(NOT run_result EQUAL 0)
  message(FATAL_ERROR "queue_test exited with ${run_result}\n${run_output}${run_error}")
endif()

message(STATUS "queue_test output:\n${run_output}")

set(failures "")

foreach(index RANGE 1 4)
  set(actual "${ROOT}/result/result${index}.txt")
  set(expected "${ROOT}/result/expected_result${index}.txt")

  if(NOT EXISTS "${actual}")
    list(APPEND failures "result${index}.txt was never written")
    continue()
  endif()

  # --ignore-eol so a CRLF checkout on Windows is not reported as a difference.
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files --ignore-eol "${actual}" "${expected}"
    RESULT_VARIABLE compare_result
  )

  if(compare_result EQUAL 0)
    message(STATUS "  testcase ${index}: matches the reference")
  else()
    list(APPEND failures "result${index}.txt does not match expected_result${index}.txt")
  endif()
endforeach()

if(failures)
  string(REPLACE ";" "\n  " report "${failures}")
  message(FATAL_ERROR "pipeline output did not match the reference:\n  ${report}")
endif()

message(STATUS "all four testcases match the reference output")
