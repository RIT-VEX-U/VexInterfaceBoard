# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD sparkfun_promicro_rp2350 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(VexInterfaceBoard C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

file(GLOB_RECURSE SOURCES "src/*" "Eigen/*")
add_executable(VexInterfaceBoard ${SOURCES})

# Add executable. Default name is the project name, version 0.1


pico_set_program_name(VexInterfaceBoard "VexInterfaceBoard")
pico_set_program_version(VexInterfaceBoard "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(VexInterfaceBoard 0)
pico_enable_stdio_usb(VexInterfaceBoard 1)

# Add the standard library to the build
target_link_libraries(VexInterfaceBoard
        pico_stdlib hardware_i2c)

# Add the standard include files to the build
target_include_directories(VexInterfaceBoard PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/..
  eigen-3.4.0/
)

pico_add_extra_outputs(VexInterfaceBoard)

