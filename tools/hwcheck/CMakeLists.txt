# Hardware check programs of docs/hardware-testing.md. They share common.hpp with the examples.
foreach(check check_motors check_io)
  add_executable(${check} ${check}.cpp)
  target_include_directories(${check} PRIVATE ${PROJECT_SOURCE_DIR}/examples)
  target_link_libraries(${check} PRIVATE robotino_api1)
  robotino_api1_warnings(${check})
endforeach()
