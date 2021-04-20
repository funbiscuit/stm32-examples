cmake_minimum_required(VERSION 3.1)
project(h7_i2c_eeprom LANGUAGES C CXX)

#Sample cmake config, not intended for actual build, only for code completion
set(CMAKE_C_STANDARD 11)

add_library(h7_i2c_eeprom STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/main.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32h7xx_hal_msp.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32h7xx_hal_timebase_tim.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/stm32h7xx_it.c
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/system_stm32h7xx.c
        )

target_sources(h7_i2c_eeprom PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Src/user_run.c
        )

target_include_directories(h7_i2c_eeprom PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
        ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
        )

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Src HAL_SRC)

target_sources(h7_i2c_eeprom PRIVATE ${HAL_SRC})

add_compile_definitions(USE_HAL_DRIVER)
add_compile_definitions(STM32H743xx)
