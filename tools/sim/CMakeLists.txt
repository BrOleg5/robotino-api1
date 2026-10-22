add_library(robotino_api1_sim STATIC simulator.cpp)
target_include_directories(robotino_api1_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(robotino_api1_sim PUBLIC robotino_api1 robotino_api1_net)
robotino_api1_warnings(robotino_api1_sim)

add_executable(robotino-api1-sim main.cpp)
target_link_libraries(robotino-api1-sim PRIVATE robotino_api1_sim)
robotino_api1_warnings(robotino-api1-sim)

if(ROBOTINO_API1_INSTALL)
  install(TARGETS robotino-api1-sim RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
