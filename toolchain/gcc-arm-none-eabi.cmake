set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_COMPILER_ID_RUN 1)
set(CMAKE_C_COMPILER_ID GNU)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror -Wextra -Wpedantic -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")


# linker file
set(LINKER_SCRIPT "")
if(MCU_TYPE STREQUAL "HC32F4A0xG") # PARENT_SCOPE
    if(BOOTLOADER)
        if(RT-THREAD)
            set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../rt-thread/linker/bootloader/HC32F4A0xG.ld" )
        else()
            set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/../linker/bootloader/HC32F4A0xG.ld" )
        endif()
    elseif(APP)
        if(RT-THREAD)
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../rt-thread/linker/app/HC32F4A0xG.ld" )
        else()
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../linker/app/HC32F4A0xG.ld" )
        endif()
    else()
        if(RT-THREAD)
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../rt-thread/linker/normal/HC32F4A0xG.ld" )
        else()
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../cmsis/Device/HDSC/hc32f4xx/Source/GCC/linker/HC32F4A0xG.ld" )
        endif()
    endif()
elseif(MCU_TYPE STREQUAL "HC32F4A0xI")
    if(BOOTLOADER)
        if(RT-THREAD)
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../rt-thread/linker/bootloader/HC32F4A0xI.ld" )
        else()
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../linker/bootloader/HC32F4A0xI.ld" )
       endif() 
    elseif(APP)
        if(RT-THREAD)
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../rt-thread/linker/app/HC32F4A0xI.ld" )
        else()
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../linker/app/HC32F4A0xI.ld" )
        endif()
    else()
        if(RT-THREAD)
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../rt-thread/linker/normal/HC32F4A0xI.ld" )
        else()
            set(LINKER_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../cmsis/Device/HDSC/hc32f4xx/Source/GCC/linker/HC32F4A0xI.ld" )
        endif()
    endif()
else()
    message(FATAL_ERROR "Please enter the MCU model.")
endif()

if(NOT DEFINED FIRMWARE_NAME)
    set(FIRMWARE_NAME "${CMAKE_PROJECT_NAME}")
endif()

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${LINKER_SCRIPT}\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -u_printf_float -u_scanf_float")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${FIRMWARE_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--start-group -lstdc++ -lsupc++ -lc -lm -Wl,--end-group")
# if use arm-none-eabi-gcc > 10.3 , need add -Wl,-no-warn-rwx-segments,if =10.3 ,no need add -Wl,-no-warn-rwx-segments
# set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS } -Wl,-no-warn-rwx-segments")
