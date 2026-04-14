# cmake/run-clang-tool.cmake
# Runs clang-format or clang-tidy on git-modified source files.
#
# Required variables (pass via -D):
#   TOOL       - path to clang-format or clang-tidy
#   MODE       - "format", "format-check", or "tidy"
#   SOURCE_DIR - project source directory
#
# Optional:
#   BUILD_DIR  - build directory (for clang-tidy -p)

find_package(Git REQUIRED)

# Get staged + unstaged modified files (tracked)
execute_process(
	COMMAND ${GIT_EXECUTABLE} diff --name-only HEAD
	WORKING_DIRECTORY ${SOURCE_DIR}
	OUTPUT_VARIABLE MODIFIED_FILES
	OUTPUT_STRIP_TRAILING_WHITESPACE
	RESULT_VARIABLE GIT_RESULT
)

# Also get untracked new files
execute_process(
	COMMAND ${GIT_EXECUTABLE} ls-files --others --exclude-standard
	WORKING_DIRECTORY ${SOURCE_DIR}
	OUTPUT_VARIABLE UNTRACKED_FILES
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(ALL_CHANGED "${MODIFIED_FILES}\n${UNTRACKED_FILES}")

# Convert newline-separated list to cmake list
string(REPLACE "\n" ";" FILE_LIST "${ALL_CHANGED}")

# Filter to only source files under relevant directories
set(SOURCE_FILES)
foreach(F ${FILE_LIST})
	if(F MATCHES "\\.(cpp|c|h|hpp)$"
		AND (F MATCHES "^ode/" OR F MATCHES "^include/" OR F MATCHES "^drawstuff/" OR F MATCHES "^tests/"))
		list(APPEND SOURCE_FILES "${SOURCE_DIR}/${F}")
	endif()
endforeach()

list(LENGTH SOURCE_FILES FILE_COUNT)
if(FILE_COUNT EQUAL 0)
	message(STATUS "No modified source files found.")
	return()
endif()

message(STATUS "Found ${FILE_COUNT} modified source file(s):")
foreach(F ${SOURCE_FILES})
	file(RELATIVE_PATH REL "${SOURCE_DIR}" "${F}")
	message(STATUS "  ${REL}")
endforeach()

if(MODE STREQUAL "format")
	execute_process(
		COMMAND ${TOOL} -i ${SOURCE_FILES}
		WORKING_DIRECTORY ${SOURCE_DIR}
		RESULT_VARIABLE RESULT
	)
elseif(MODE STREQUAL "format-check")
	execute_process(
		COMMAND ${TOOL} --dry-run --Werror ${SOURCE_FILES}
		WORKING_DIRECTORY ${SOURCE_DIR}
		RESULT_VARIABLE RESULT
	)
elseif(MODE STREQUAL "tidy")
	execute_process(
		COMMAND ${TOOL} -p ${BUILD_DIR} ${SOURCE_FILES}
		WORKING_DIRECTORY ${SOURCE_DIR}
		RESULT_VARIABLE RESULT
	)
else()
	message(FATAL_ERROR "Unknown MODE: ${MODE}")
endif()

if(NOT RESULT EQUAL 0)
	message(FATAL_ERROR "${MODE} failed with exit code ${RESULT}")
endif()
