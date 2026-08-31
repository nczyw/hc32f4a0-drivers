# 小华单片机hc32f4a0系列cmake驱动
## 介绍
 - 驱动来自小华的2.2.3版本,修改了部分代码,添加了部分功能.
 - 支持配置为`bootloader,app,normal`三种模式.
 - 还支持是否有`rt-thread`系统.
 - bootloader模式下,flash前128KB为bootloader,其余为app.
 - 添加了`printf`支持浮点打印.
 - 当使用gcc-arm-none-eabi,11及以上版本的编译器时，编译时会出现`LOAD segment with RWX permissions 警告`,`toolchain`中可以忽略这个警告，但是忽略参数11版本以下的又不支持,所以保留了这个警告,修改链接脚本也可以消除此警告,但是同样不支持11版本以下的编译器,固没做修改.
 - 使用严格的编程模式,任何警告都视为错误,停止编译,可以养成好编程习惯.
## 修改内容
 - `hc32_ll_can.c`中`const static uint8_t m_au8DLC2WordSize[16U]`修改为`static const uint8_t m_au8DLC2WordSize[16U] `将`static`放在最前面,以规范代码,去除警告.
 - 当有`rt-thread`
   - 修改了`startup_hc32f4a0.S`中`bl main`修改为`bl entry`来支持`rt-thread`系统.
   - 修改了`HC32F4A0xI.ld`和`HC32F4A0xG.ld`链接脚本,来支持`rt-thread`系统相关自动初始化函数,来保证不被编译器优化.
   - 去除了usb驱动,需要其它文件,还没研究.
## 使用前准备
- [Cmake下载](https://cmake.org/download/)
  - 安装Cmake,会自动将cmake安装到环境变量中,如果`cmake`提示未找到要手动将cmake的bin目录加入到环境变量中.
- [MinGW下载](https://github.com/niXman/mingw-builds-binaries/releases)
  - 安装MinGW编译器,如果是是解压类型,解压后把bin目录加入到环境变量中.
- [gcc-arm-none-eabi下载](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  - 安装gcc-arm-none-eabi,如果是解压型的,解压后把bin目录加入到环境变量中.
- [pyOCD安装方法](https://github.com/pyocd/pyOCD),这是下载和调试mcu的工具,只编译的话,不需要.
  - 还可以安装libusb,来显示更多调试信息,比如断言.
## Cmake 配置例子
 - 在自己的项目中添加本驱动
 ```bash
 git submodule add https://github.com/nczyw/hc32f4a0-drivers.git drivers
 ```
 - 项目CMake例子
```cmake
cmake_minimum_required(VERSION 3.27)

# Setup compiler settings
set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME myproject)

# Driver function selection
set(MCU_TYPE "HC32F4A0xI" CACHE STRING "Set MCU Type")
set(BOOTLOADER OFF CACHE BOOL "Set to ON if bootloader is enabled")
set(APP OFF CACHE BOOL "Set to ON if app is enabled")
set(RT-THREAD OFF CACHE BOOL "Set to ON if RT-Thread is enabled")

# Include toolchain file
include("drivers/toolchain/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} VERSION 1.0.0)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})
# "Generate hex and bin files and associate them with the clean target."
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    BYPRODUCTS 
        ${CMAKE_PROJECT_NAME}.hex
        ${CMAKE_PROJECT_NAME}.bin
)
# Include the drivers,Drivers can be loaded using the submodule management method, which is recommended.
add_subdirectory("drivers")

set(DEFINES
    # Add user defines
)

file(GLOB SOURCES
    # Add user source files
)

set(HEADERS
    # Add user headers
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ${DEFINES}
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${SOURCES}
)
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${HEADERS}
)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)
```