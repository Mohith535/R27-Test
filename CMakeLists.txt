cmake_minimum_required(VERSION 3.20)

project(r27_test)

# The sources use pthreads and POSIX semaphores. On glibc 2.34 and newer these
# live in libc and link by accident; on anything older they do not. Asking
# CMake for the thread library makes that explicit instead of luck.
find_package(Threads REQUIRED)

add_library(
  r27 
  src/queue.c
  src/mutex.c 
  src/en_dc.c
  src/file.c
  src/drive.c
)

target_link_libraries(
  r27
  PUBLIC
  Threads::Threads
)

target_include_directories(
  r27 
  PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/lib 
  
) 

add_executable(
  queue_test 
  src/main.c 
)
target_link_libraries(
  queue_test 
  PRIVATE 
  r27
  m
)

add_executable(
  en_dc_test
  tests/en_dc_test.c
)
target_link_libraries(
  en_dc_test
  PRIVATE
  r27
)
