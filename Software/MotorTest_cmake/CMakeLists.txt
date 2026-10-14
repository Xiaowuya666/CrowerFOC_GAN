# file-format: 1.0.0
cmake_minimum_required(VERSION 3.30)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

include(cmake/target.cmake)
include(cmake/${TOOLCHAIN_FILE})

project(MotorTest)
add_executable(MotorTest)
include(cmake/files.cmake)
include(cmake/flags.cmake)
include(cmake/components.cmake)

# Stubs to be able to add compiler flag overrides, e.g. -Os
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
  # Add additional source files here
  ws2812.c
  pwm_test.c
  phase_test.c
  config_store.c
  can_transport.c
  stm32c5xx_drivers/hal/stm32c5xx_hal_flash.c
)

# The 40 kHz FOC ISR needs optimisation (the project default is -O0).
set_source_files_properties(phase_test.c PROPERTIES COMPILE_OPTIONS "-O2")
set_source_files_properties(config_store.c can_transport.c PROPERTIES COMPILE_OPTIONS "-O2")

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional include directories here
)

target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional defined symbols here
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional compiler flags here
)

target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker flags here
)

target_link_directories(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker search directories here
)

target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC
  # Add additional linker libraries here
)
