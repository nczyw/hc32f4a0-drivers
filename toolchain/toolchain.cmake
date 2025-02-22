set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_C_STANDARD 17)
set(CMAKE_SYSTEM_NAME Generic)
set(TOOLCHAIN_PREFIX arm-none-eabi)

# 指定编译工具
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}-ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}-objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}-objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}-size)
set(CMAKE_READELF ${TOOLCHAIN_PREFIX}-readelf)
set(CMAKE_NM ${TOOLCHAIN_PREFIX}-nm)

# 避免 CMake 尝试运行目标可执行文件
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# 添加 -specs=nosys.specs 以避免 _close 未实现的警告
if(NOT "${CMAKE_EXE_LINKER_FLAGS}" MATCHES "-specs=nosys.specs")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -specs=nosys.specs")
endif()