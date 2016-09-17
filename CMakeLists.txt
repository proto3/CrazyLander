cmake_minimum_required(VERSION 2.8)
project(crazylander)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################################################
#PACKAGES
find_package(OpenCV REQUIRED)

################################################################################
#INCLUDES
include_directories(
    include/crazylander
)
################################################################################
#SOURCES
file(GLOB_RECURSE CRAZY_LANDER_SOURCES "src/crazylander/*.cpp")
################################################################################
#LINK
LIST(APPEND LINK_LIBS
    ${OpenCV_LIBS}
)
################################################################################
#EXECUTABLES
add_executable(${PROJECT_NAME} ${CRAZY_LANDER_SOURCES} src/main.cpp)
target_link_libraries(${PROJECT_NAME} ${LINK_LIBS})

add_executable(tag_generator ${CRAZY_LANDER_SOURCES} src/tag_generator.cpp)
target_link_libraries(tag_generator ${LINK_LIBS})
