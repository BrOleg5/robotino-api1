include(FetchContent)
# Only the header is used: doctest's own CMakeLists.txt is too old for CMake 4.
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()
set(ROBOTINO_API1_DOCTEST_OPTIONS)
if(NOT CMAKE_VERSION VERSION_LESS 3.24)
  set(ROBOTINO_API1_DOCTEST_OPTIONS DOWNLOAD_EXTRACT_TIMESTAMP ON)
endif()
FetchContent_Declare(doctest
  URL https://github.com/doctest/doctest/archive/refs/tags/v2.4.11.tar.gz
  URL_HASH SHA256=632ed2c05a7f53fa961381497bf8069093f0d6628c5f26286161fbd32a560186
  ${ROBOTINO_API1_DOCTEST_OPTIONS})
FetchContent_GetProperties(doctest)
if(NOT doctest_POPULATED)
  FetchContent_Populate(doctest)
endif()

add_executable(robotino_api1_tests
  main.cpp
  test_protocol.cpp
  test_state.cpp
  test_kinematics.cpp
  test_image.cpp
  test_client.cpp
)
target_include_directories(robotino_api1_tests PRIVATE ${doctest_SOURCE_DIR})
target_compile_definitions(robotino_api1_tests PRIVATE ROBOTINO_API1_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_link_libraries(robotino_api1_tests PRIVATE robotino_api1 robotino_api1_sim)
robotino_api1_warnings(robotino_api1_tests)

add_test(NAME robotino_api1_tests COMMAND robotino_api1_tests)

if(ROBOTINO_API1_BUILD_C_API)
  # Checks that the C header compiles as C.
  enable_language(C)
  add_executable(robotino_api1_c_tests test_c_api.c)
  target_link_libraries(robotino_api1_c_tests PRIVATE robotino_api1_c)
  add_test(NAME robotino_api1_c_tests COMMAND robotino_api1_c_tests)
endif()
