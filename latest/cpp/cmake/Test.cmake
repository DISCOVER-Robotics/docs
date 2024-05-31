if(COMPILE_TESTS)
  add_subdirectory(thirdparty/googletest)
  enable_testing()
  message(STATUS "${GTEST_INCLUDE_DIRS}")
  include_directories(${GTEST_INCLUDE_DIRS})

  file(GLOB_RECURSE TEST_SOURCES "tests/*.cpp")

  foreach(source ${TEST_SOURCES})
    message(STATUS "Adding test: ${source}")
    get_filename_component(NAME ${source} NAME_WE)
    add_executable(${NAME} ${source})
    target_link_libraries(${NAME} ${PROJECT_NAME} ${DEPENDENCIES} gtest_main)
    add_dependencies(${NAME} version_header)

    include(GoogleTest)
    gtest_discover_tests(${NAME})
  endforeach()
endif()
