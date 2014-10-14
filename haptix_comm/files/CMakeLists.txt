cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

########################################
# Find the haptix_comm library
find_package(haptix_comm QUIET REQUIRED)
if (NOT haptix_comm_FOUND)
  BUILD_ERROR ("Missing: Haptix Comm (libhaptix_comm-dev)")
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${HAPTIX_COMM_CFLAGS}")
include_directories(${HAPTIX_COMM_INCLUDE_DIRS})
link_directories(${HAPTIX_COMM_LIBRARY_DIRS})

########################################
add_executable(hx_controller hx_controller.c)
target_link_libraries(hx_controller ${HAPTIX_COMM_LIBRARIES} m)
