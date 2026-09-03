# 小华单片机hc32f4a0系列cmake驱动
## 介绍
 - 驱动来自小华的2.2.3版本,修改了部分代码,添加了部分功能.
 - 支持配置为`bootloader,app,normal`三种模式.
 - 还支持是否有`rt-thread`系统.
 - bootloader模式下,flash前128KB为bootloader,其余为app.
 - 添加了`printf`支持浮点打印.
 - 当使用gcc-arm-none-eabi,11及以上版本的编译器时，编译时会出现`LOAD segment with RWX permissions 警告`,`toolchain`中可以忽略这个警告，但是忽略参数11版本以下的又不支持,所以保留了这个警告,修改链接脚本也可以消除此警告,但是同样不支持11版本以下的编译器,固没做修改.
 - 使用严格的编程模式,任何警告都视为错误,停止编译,可以养成好编程习惯.
 - 将C库修改为`nano.specs`, 以前使用的`nosys.specs`使用C库一些函数后固件会变的非常大
## 修改内容
 - `hc32_ll_can.c`中`const static uint8_t m_au8DLC2WordSize[16U]`修改为`static const uint8_t m_au8DLC2WordSize[16U] `将`static`放在最前面,以规范代码,去除警告.
 - 当有`rt-thread`
   - 修改了`startup_hc32f4a0.S`中`bl main`修改为`bl entry`来支持`rt-thread`系统.
   - 修改了`HC32F4A0xI.ld`和`HC32F4A0xG.ld`链接脚本,来支持`rt-thread`系统相关自动初始化函数,来保证不被编译器优化.
   - 去除了usb驱动,需要其它文件,还没研究.
- 添加`hc32_ll_hash_ex`驱动文件,实现了`sha256`和 `hmac` 分段添加并运算功能.
- 链接脚本中添加了`__app_start`,`__app_end`,`__app_size`,用来指示当前固件在flash中开始地址,结束地址,大小,方便用来校验固件,使用方法:`extern char __app_size;`,`static uint32_t firmware_size  = (uint32_t)&__app_size;`
- 添加了`set(CMAKE_EXPORT_COMPILE_COMMANDS ON)`生成`compile_commands.json`文件,用于代码补全.
## 使用前准备
- [Cmake下载](https://cmake.org/download/)
  - 安装Cmake,会自动将cmake安装到环境变量中,如果`cmake`提示未找到要手动将cmake的bin目录加入到环境变量中.
- [Ninja下载](https://github.com/ninja-build/ninja/releases/)
  - 安装Ninja,如果是是解压类型,解压后把目录加入到环境变量中.
- [gcc-arm-none-eabi下载](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
  - 安装gcc-arm-none-eabi,如果是解压型的,解压后把bin目录加入到环境变量中.
- [pyOCD安装方法](https://github.com/pyocd/pyOCD),这是下载和调试mcu的工具,只编译的话,不需要.
  - 还可以安装libusb,来显示更多调试信息,比如断言.
- pyocd烧写方法:`pyocd flash -t hc32f4a0xi firmware.elf`
## Cmake 配置例子
 - 在自己的项目中添加本驱动
 - 添加了`PROJECT_VERSION_MAJOR`,`PROJECT_VERSION_MINOR`,`PROJECT_VERSION_PATCH` 宏定义, 下面是使用一个`uint32_t`来表示版本的示例
 ```c
 uint32_t version_get(void){
    return ((uint32_t)(PROJECT_VERSION_MAJOR & 0x3FFU) << 22U) |
           ((uint32_t)(PROJECT_VERSION_MINOR & 0x7FFU) << 11U) |
           ((uint32_t)(PROJECT_VERSION_PATCH & 0x7FFU));
 }
 ```
 ```bash
 # 添加驱动为子模块
 git submodule add https://github.com/nczyw/hc32f4a0-drivers.git drivers
 # 初始化子模块
 git submodule init
 # 更新子模块
 git submodule update --init --recursive
 # 创建编译文件夹
 mkdir build
 cd build
 # 编译(linux)
 cmake -DCMAKE_BUILD_TYPE=Release .. -G Ninja
 cmake --build .
 # 编译(windows)
 cmake -DCMAKE_BUILD_TYPE=Release .. -G Ninja 
 cmake --build .
 # 烧写
 pyocd flash -t hc32f4a0xi myproject.elf
 ```
 - vsocde打开项目时显示全红,但是可以编译的解决方法,因cmake中添加了`set(CMAKE_EXPORT_COMPILE_COMMANDS ON)`,会自动在编译目录下生成`compile_commands.json`文件,在项目下的`.vscode`目录下创建一个`c_cpp_properties.json`文件,并添加以下内容,保存后,会自动在结尾添加`version: 4`,版本字样,可能不是4,`name`可以自己设置,`compile_commands.json`的生成目录也可以自己设置和cmake的生成目录一致.
 ```json
 {
    "configurations": [
        {
            "name": "HC32F4A0",
            "compilerPath": "arm-none-eabi-gcc",
            "intelliSenseMode": "windows-gcc-arm",
            "cStandard": "gnu17",
            "cppStandard": "gnu++17",
            "compileCommands": [
                "build/compile_commands.json"
            ]
        }
    ]
}
 ```
 - 项目CMake例子
```cmake
cmake_minimum_required(VERSION 3.27)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    PROJECT_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    PROJECT_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    PROJECT_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)
```