cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

########################################
# Find the haptix_comm library
find_package(haptix-comm QUIET REQUIRED)
if (NOT haptix-comm_FOUND)
  BUILD_ERROR ("Missing: Haptix Comm (libhaptix-comm-dev)")
endif()

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${HAPTIX-COMM_CFLAGS}")
include_directories(${HAPTIX-COMM_INCLUDE_DIRS})
link_directories(${HAPTIX-COMM_LIBRARY_DIRS})

########################################
add_executable(hx_controller hx_controller.c)
target_link_libraries(hx_controller ${HAPTIX-COMM_LIBRARIES} m)
