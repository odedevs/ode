if(CMAKE_VERSION VERSION_LESS "3.12.4")
  add_definitions(-D_USE_MATH_DEFINES)
else()
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  set(PREPROCESS_ONLY_FLAGS /nologo /P /C /TP)
  set(PREPROCESS_OUTPUT_FLAG "/Fi")
  set(PREPROCESS_VA_OPT ", ##")
elseif(NOT ENV{MSYSTEM_PREFIX} STREQUAL "")
  list(APPEND CMAKE_PREFIX_PATH "C:/msys64$ENV{MSYSTEM_PREFIX}" "C:/msys64/usr")
endif()

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Debug' as none was specified.")
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE
               PROPERTY STRINGS
                        "Debug"
                        "Release"
                        "MinSizeRel"
                        "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT ENV{MSYSTEM_PREFIX} STREQUAL "")
  set(PkgConfig_ROOT $ENV{MSYSTEM_PREFIX})
endif()
find_package(PkgConfig)
if(PKG_CONFIG_FOUND AND BASH_EXECUTABLE AND MSYS)
  # In order to properly use pkg-config in a msys2 environment we need to jump
  # through some hoops. CMake seems to call pkg-config without any of the bash
  # environment set up, so we need to set the proper pkg config path inside the
  # cmake process address space.
  execute_process(COMMAND ${BASH_EXECUTABLE} --login -c "echo $PKG_CONFIG_PATH"
                  OUTPUT_VARIABLE PKG_CONFIG_PATH)
  set(ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH})
endif()
