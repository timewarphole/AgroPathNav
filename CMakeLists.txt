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
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Pfinal C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable with all source files
add_executable(Pfinal 
    Pfinal.c
    l298n.c
    hcsr04.c
    pid.c
    lm35.c
    fotoresistencia.c
    eeprom.c
)

pico_set_program_name(Pfinal "Pfinal")
pico_set_program_version(Pfinal "0.1")

# Enable USB output, disable UART (consola por USB)
pico_enable_stdio_uart(Pfinal 0)
pico_enable_stdio_usb(Pfinal 1)

# Add the standard library to the build
target_link_libraries(Pfinal
    pico_stdlib
)

# Add the standard include files to the build
target_include_directories(Pfinal PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Add all required hardware libraries
target_link_libraries(Pfinal 
    hardware_pwm        # Para control PWM de motores L298N
    hardware_timer      # Para medición de tiempo en HC-SR04
    hardware_gpio       # Para interrupciones (botón, ECHO)
    hardware_adc        # Para sensores analógicos (LM35, LDR)
    hardware_i2c        # Para comunicación con EEPROM 24LC128
)

pico_add_extra_outputs(Pfinal)
