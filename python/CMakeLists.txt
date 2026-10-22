if(NOT TARGET robotino_api1_sim)
  message(FATAL_ERROR "The Python module needs ROBOTINO_API1_BUILD_SIM=ON")
endif()

find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(_core NB_STATIC src/module.cpp)
target_link_libraries(_core PRIVATE robotino_api1 robotino_api1_sim)
install(TARGETS _core LIBRARY DESTINATION robotino_api1)
