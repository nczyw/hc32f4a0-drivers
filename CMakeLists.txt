cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)

set(Drivers_Defines
    HC32F4A0
    USE_DDL_DRIVER
#    $<$<CONFIG:Debug>:__DEBUG>
    __DEBUG
)

# 驱动头文件
set(Drivers_Hearers
    "${CMAKE_CURRENT_LIST_DIR}/mcu/inc"
    "${CMAKE_CURRENT_LIST_DIR}/hc32_ll_driver/inc"
    "${CMAKE_CURRENT_LIST_DIR}/cmsis/Include"
    "${CMAKE_CURRENT_LIST_DIR}/cmsis/Device/HDSC/hc32f4xx/Include"
)

# 源文件
file(GLOB Drivers_Sources
#    "${CMAKE_CURRENT_LIST_DIR}/cmsis/Device/HDSC/hc32f4xx/Source/GCC/startup_hc32f4a0.S"
    "${CMAKE_CURRENT_LIST_DIR}/cmsis/Device/HDSC/hc32f4xx/Source/system_hc32f4a0.c"
    "${CMAKE_CURRENT_LIST_DIR}/hc32_ll_driver/src/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/mcu/src/*.c"
)
list(REMOVE_ITEM Drivers_Sources "${CMAKE_CURRENT_LIST_DIR}/hc32_ll_driver/src/hc32_ll_usb.c")

if(BOOTLOADER AND APP)
    message(FATAL_ERROR "Cannot define both BOOTLOADER and APP. Choose only one.")
endif()

if(BOOTLOADER)
    list(APPEND Drivers_Defines USER_BOOTLOADER)
    message(STATUS "Create bootloader project.")
elseif(APP)
    list(REMOVE_ITEM Drivers_Sources "${CMAKE_CURRENT_LIST_DIR}/hc32_ll_driver/src/hc32_ll_icg.c")
    list(APPEND Drivers_Defines USER_APP)
    list(APPEND Drivers_Defines VECT_TAB_OFFSET=0x20000)
    message(STATUS "Create app project.")
else()
    message(STATUS "Create nomal project.")
endif()
if(RT-THREAD)
    message(STATUS "Use RT-Thread.")
    list(APPEND Drivers_Defines RTTHREAD)
    list(APPEND Drivers_Sources
        "${CMAKE_CURRENT_LIST_DIR}/rt-thread/mcu/startup_hc32f4a0.S"
    )
else()
    list(APPEND Drivers_Sources
        "${CMAKE_CURRENT_LIST_DIR}/cmsis/Device/HDSC/hc32f4xx/Source/GCC/startup_hc32f4a0.S"
    )
endif()


add_library(hc32f4a0 INTERFACE)
target_include_directories(hc32f4a0 INTERFACE ${Drivers_Hearers})
target_compile_definitions(hc32f4a0 INTERFACE ${Drivers_Defines})

# create drivers object library
add_library(hc32f4a0_driver OBJECT)
# message(STATUS "Drivers_Sources: ${Drivers_Sources}")
target_sources(hc32f4a0_driver PRIVATE ${Drivers_Sources})
target_link_libraries(hc32f4a0_driver PUBLIC hc32f4a0)

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE hc32f4a0_driver)


# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()