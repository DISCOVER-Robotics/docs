cmake_minimum_required(VERSION 3.16.3)
project(
  airbot_play
  VERSION 2.8.3
  LANGUAGES CXX)

option(GIT_CLEAN_CHECK "Check if git workspace is clean" ON)
option(COMPILE_TESTS "Compile tests" ON)
include(GNUInstallDirs)
include(cmake/Version.cmake)

set(VERSION_HEADER "${CMAKE_CURRENT_BINARY_DIR}/src/airbot/airbot.hpp")
configure_file(src/airbot/airbot.hpp.in ${VERSION_HEADER} @ONLY)
add_custom_target(
  version_header ALL
  DEPENDS ${VERSION_HEADER}
  COMMENT "Generating version header")

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)
set(JSON_Install
    ON
    CACHE INTERNAL "")
set(JSON_SystemInclude
    ON
    CACHE INTERNAL "")

find_package(Boost COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(orocos_kdl REQUIRED)
# find_package(kdl_parser REQUIRED)
find_package(spdlog REQUIRED)

add_subdirectory(thirdparty/json)
add_subdirectory(thirdparty/cpp-httplib)
add_subdirectory(thirdparty/argparse)
add_subdirectory(thirdparty/kdl_parser/kdl_parser)

file(GLOB_RECURSE SOURCES "src/airbot/*.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCES})

set(PUBLIC_DEPENDENCIES # Precompiled
    Eigen3::Eigen fmt::fmt ${orocos_kdl_LIBRARIES} kdl_parser spdlog::spdlog)
set(PRIVATE_DEPENDENCIES # Thirdparty
    nlohmann_json httplib::httplib argparse ${Boost_LIBRARIES} pthread)
target_link_libraries(${PROJECT_NAME} PUBLIC ${PUBLIC_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PRIVATE_DEPENDENCIES})

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC ${orocos_kdl_INCLUDE_DIRS}
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> ${EIGEN3_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME}
                           PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/src)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/kdl_parser/kdl_parser/include)

add_dependencies(${PROJECT_NAME} version_header)

# include(cmake/Test.cmake)

set_target_properties(
  ${PROJECT_NAME}
  PROPERTIES VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR})

file(GLOB_RECURSE TOOLS "tools/*.cpp")
foreach(source ${TOOLS})
  get_filename_component(NAME ${source} NAME_WE)
  message(STATUS "Adding tool: ${NAME}")
  add_executable(${NAME} ${source})
  target_link_libraries(${NAME} PUBLIC ${PUBLIC_DEPENCENCIES} ${PROJECT_NAME})
  target_link_libraries(${NAME} PRIVATE ${PRIVATE_DEPENDENCIES})

  target_include_directories(
    ${NAME}
    PUBLIC ${orocos_kdl_INCLUDE_DIRS}
           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
           ${EIGEN3_INCLUDE_DIR})
  target_include_directories(${NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/src)
  add_dependencies(${NAME} version_header)
  install(
    TARGETS ${NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()

find_package(Python COMPONENTS Interpreter Development)
execute_process(
  COMMAND
    "${Python_EXECUTABLE}" -c
    "from distutils.sysconfig import get_python_lib; print(get_python_lib())"
  OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
  OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS "Python site-packages directory: ${PYTHON_SITE_PACKAGES}")
install(
  DIRECTORY python/${PROJECT_NAME}/
  DESTINATION ${PYTHON_SITE_PACKAGES}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py")
install(FILES python/requirements.txt
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
message(STATUS "Installing Python requirements file to \
${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/requirements.txt")

add_custom_target(
  install_python_requirements
  COMMAND
    ${Python_EXECUTABLE} -m pip install -r
    ${CMAKE_SOURCE_DIR}/python/requirements.txt -i
    https://mirrors.ustc.edu.cn/pypi/web/simple
  COMMENT "Installing Python requirements from requirements.txt")
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} \
--build ${CMAKE_BINARY_DIR} --target install_python_requirements)")

include(cmake/Install.cmake)
include(cmake/Packing.cmake)
