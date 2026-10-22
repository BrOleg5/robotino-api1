cmake_minimum_required(VERSION 3.16)
project(robotino_api1 VERSION 0.1.0 DESCRIPTION "Open-source client library for Festo Robotino robots with OpenRobotinoAPI 1 (before Robotino 3)" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

get_directory_property(ROBOTINO_API1_HAS_PARENT PARENT_DIRECTORY)
if(ROBOTINO_API1_HAS_PARENT)
  set(ROBOTINO_API1_TOP_LEVEL OFF)
else()
  set(ROBOTINO_API1_TOP_LEVEL ON)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
option(ROBOTINO_API1_BUILD_TESTS "Build tests" ${ROBOTINO_API1_TOP_LEVEL})
option(ROBOTINO_API1_BUILD_SIM "Build the robot emulator" ${ROBOTINO_API1_TOP_LEVEL})
option(ROBOTINO_API1_BUILD_EXAMPLES "Build examples" ${ROBOTINO_API1_TOP_LEVEL})
option(ROBOTINO_API1_BUILD_C_API "Build the C API library" ON)
option(ROBOTINO_API1_BUILD_PYTHON "Build the Python module (needs nanobind)" OFF)
option(ROBOTINO_API1_INSTALL "Generate install rules" ${ROBOTINO_API1_TOP_LEVEL})

include(GNUInstallDirs)

if(ROBOTINO_API1_TOP_LEVEL AND NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  # Executables next to the DLLs, so that they run on Windows without PATH changes.
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
endif()
find_package(Threads REQUIRED)

function(robotino_api1_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
  endif()
endfunction()

# Sockets, internal. The objects are compiled into the library; the emulator and the tests
# link them as robotino_api1_net because the shared library does not export them.
add_library(robotino_api1_net_objects OBJECT src/net/socket.cpp)
target_include_directories(robotino_api1_net_objects PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
set_target_properties(robotino_api1_net_objects PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden)
robotino_api1_warnings(robotino_api1_net_objects)

add_library(robotino_api1_net STATIC $<TARGET_OBJECTS:robotino_api1_net_objects>)
target_include_directories(robotino_api1_net PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src> $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>)
if(WIN32)
  target_link_libraries(robotino_api1_net PUBLIC ws2_32)
endif()

add_library(robotino_api1
  src/protocol/qdsa.cpp
  src/protocol/messages.cpp
  src/protocol/image.cpp
  src/state.cpp
  src/kinematics.cpp
  src/client.cpp
  $<TARGET_OBJECTS:robotino_api1_net_objects>
)
add_library(robotino_api1::robotino_api1 ALIAS robotino_api1)
target_include_directories(robotino_api1 PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_include_directories(robotino_api1 PRIVATE src)
target_link_libraries(robotino_api1 PUBLIC Threads::Threads)
if(WIN32)
  target_link_libraries(robotino_api1 PRIVATE ws2_32)
endif()
target_compile_features(robotino_api1 PUBLIC cxx_std_17)
set_target_properties(robotino_api1 PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  POSITION_INDEPENDENT_CODE ON)
if(BUILD_SHARED_LIBS)
  target_compile_definitions(robotino_api1 PRIVATE ROBOTINO_API1_BUILDING)
else()
  target_compile_definitions(robotino_api1 PUBLIC ROBOTINO_API1_STATIC)
endif()
robotino_api1_warnings(robotino_api1)

if(ROBOTINO_API1_BUILD_C_API)
  # Always shared: the C API is meant for ctypes and other languages.
  add_library(robotino_api1_c SHARED src/c_api.cpp)
  add_library(robotino_api1::robotino_api1_c ALIAS robotino_api1_c)
  target_include_directories(robotino_api1_c PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_compile_definitions(robotino_api1_c PRIVATE ROBOTINO_API1_C_BUILDING)
  target_link_libraries(robotino_api1_c PRIVATE robotino_api1)
  set_target_properties(robotino_api1_c PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    C_VISIBILITY_PRESET hidden
    CXX_VISIBILITY_PRESET hidden)
  if(WIN32)
    set_target_properties(robotino_api1_c PROPERTIES PREFIX "")  # robotino_api1_c.dll like MSVC
  endif()
  if(MINGW)
    # A self-contained DLL: no libstdc++/libgcc/winpthread DLLs next to it.
    target_link_options(robotino_api1_c PRIVATE -static)
  endif()
  robotino_api1_warnings(robotino_api1_c)
endif()

if(ROBOTINO_API1_BUILD_SIM)
  add_subdirectory(tools/sim)
endif()

if(ROBOTINO_API1_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(ROBOTINO_API1_BUILD_PYTHON)
  add_subdirectory(python)
endif()

if(ROBOTINO_API1_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(ROBOTINO_API1_INSTALL)
  include(CMakePackageConfigHelpers)
  set(ROBOTINO_API1_INSTALL_TARGETS robotino_api1)
  if(ROBOTINO_API1_BUILD_C_API)
    list(APPEND ROBOTINO_API1_INSTALL_TARGETS robotino_api1_c)
  endif()
  install(TARGETS ${ROBOTINO_API1_INSTALL_TARGETS} EXPORT robotino_api1Targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
  install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
  install(EXPORT robotino_api1Targets NAMESPACE robotino_api1:: DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robotino_api1)
  configure_package_config_file(cmake/robotino_api1Config.cmake.in
    ${PROJECT_BINARY_DIR}/robotino_api1Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robotino_api1)
  write_basic_package_version_file(${PROJECT_BINARY_DIR}/robotino_api1ConfigVersion.cmake COMPATIBILITY SameMinorVersion)
  install(FILES ${PROJECT_BINARY_DIR}/robotino_api1Config.cmake ${PROJECT_BINARY_DIR}/robotino_api1ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/robotino_api1)
endif()
