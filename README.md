# 小华单片机hc32f4a0系列cmake驱动

## 介绍

- 驱动来自小华的2.4版本,修改了部分代码,添加了部分功能.
- 支持配置为`bootloader,app,normal`三种模式.
- 还支持是否有`rt-thread`系统.
- bootloader模式下,flash前128KB为bootloader,其余为app.
- printf和scanf 默认关闭, 来减少固件大小, 如有需要,可以在CMakeLists.txt中开启, 把ENABLE_PRINTF_FLOAT和ENABLE_SCANF_FLOAT设置为ON.
- 当使用gcc-arm-none-eabi,11及以上版本的编译器时，编译时会出现`LOAD segment with RWX permissions 警告`,`toolchain`中可以忽略这个警告，是忽略参数11版本以下的又不支持,所以保留了这个警告,修改链接脚本也可以消除此警告,但是同样不支持11版本以下的编译器,固没做修改.
- 使用严格的编程模式,任何警告都视为错误,停止编译,可以养成好编程习惯.
- 将C库修改为`nano.specs`, 以前使用的`nosys.specs`使用C库一些函数后固件会变的非常大.

## 修改内容

- `hc32_ll_can.c`中`const static uint8_t m_au8DLC2WordSize[16U]`修改为`static const uint8_t m_au8DLC2WordSize[16U] `将`static`放在前面,以规范代码,去除警告.
- 当有`rt-thread`
  - 修改了`startup_hc32f4a0.S`中`bl main`修改为`bl entry`来支持`rt-thread`系统.
  - 修改了`HC32F4A0xI.ld`和`HC32F4A0xG.ld`链接脚本,来支持`rt-thread`系统相关自动初始化函数,来保证不被编译器优化.
- 恢复了 USB驱动支持, 修改了USB文件延时依赖; 使用USB时,必须要在顶层cmake中修改`option(USE_USB ON)`, 然后定义头文件, `usb_app_conf.h` .
- 添加`hc32_ll_hash_ex`驱动文件,实现了`SHA256`和`HMAC-SHA256`分段流式运算功能.
- 添加`hc32_ll_aes_ex`驱动文件,实现了`ECB`/`CBC`/`CTR`模式下的分段流式AES加解密,ECB/CBC自动处理PKCS#7 padding.
- 链接脚本中添加了`__app_start`,`__app_end`,`__app_size`,用来指示当前固件在flash中开始地址,结束地址,大小,方便用来校验固件,使用方法:`extern char __app_size;`,`static uint32_t firmware_size  = (uint32_t)&__app_size;`
- 添加了`set(CMAKE_EXPORT_COMPILE_COMMANDS ON)`生成`compile_commands.json`文件,用于代码补全.
- toolchain中添加了对版本号和git的支持, 编译的文件名自动会包含版本号`FIRMWARE_NAME` 不定义时,默认使用`CMAKE_PROJECT_NAME`

## 扩展驱动使用方法

所有扩展驱动的状态都保存在**调用方自己持有的 ctx 结构体**里(栈上/静态/堆皆可),不使用文件级全局变量,因此不同上下文交替调用同一个扩展驱动不会互相踩状态.注意: **AES / HASH 硬件是全芯片单实例**,若两个任务同时发起计算,仍需外层互斥锁(OS mutex 或裸机临界区)保护硬件访问.

### 依赖条件

在 MCU 的 `hc32f4xx_conf.h` 中确保开启对应的开关:

```c
#define LL_HASH_ENABLE      DDL_ON
#define LL_AES_ENABLE       DDL_ON
```

驱动已由 `drivers/CMakeLists.txt` 中的 `src/*.c` glob 自动纳入构建,无需额外添加源文件.

### hc32_ll_hash_ex —— 流式 SHA256 / HMAC-SHA256

头文件:`#include "hc32_ll_hash_ex.h"`

**SHA256 流式(数据零散到达/固件大于 RAM 时):**

```c
#include "hc32_ll_hash_ex.h"

stc_hash_ctx_t ctx;
uint8_t digest[32];

HASH_Init_Ex(&ctx);                   /* 重置硬件,初始化 ctx */

HASH_Update_Ex(&ctx, chunk1, len1);  /* 第1段,比如 Flash 前 1KB */
HASH_Update_Ex(&ctx, chunk2, len2);  /* 第2段 */
HASH_Update_Ex(&ctx, chunk3, len3);  /* ... 每次长度不限 */

HASH_Final_Ex(&ctx, digest);          /* 自动 padding,拿 32B 摘要 */
```

**HMAC-SHA256 流式:**

```c
stc_hmac_ctx_t ctx;
uint8_t key[] = "my-secret-key";
uint8_t hmac[32];

HMAC_Init_Ex(&ctx, key, sizeof(key) - 1); /* >64B 的 key 内部先 HASH */
HMAC_Update_Ex(&ctx, msg1, len1);
HMAC_Update_Ex(&ctx, msg2, len2);
HMAC_Final_Ex(&ctx, hmac);
```

> **原理**:HC32F4A0 的 HASH 硬件完成 SHA256 压缩运算,软件负责凑 64B 块和拼 FIPS 180-4 padding.HMAC 采用标准 RFC 2104 两次 SHA256 实现(`H(k0^opad || H(k0^ipad || msg))`).

### hc32_ll_aes_ex —— 流式 AES (ECB / CBC / CTR)

头文件:`#include "hc32_ll_aes_ex.h"`

**ECB 加密(简单块,无 IV,不推荐用于安全场景):**

```c
#include "hc32_ll_aes_ex.h"

stc_aes_ctx_t ctx;
uint8_t key[16] = { /* AES-128, 也可用 24/32 */ };
uint8_t cipher[256];
uint32_t out_len;

AES_Init_Ex(&ctx,
            AES_EX_MODE_ECB,
            AES_EX_DIR_ENCRYPT,
            key, AES_KEY_SIZE_16BYTE,
            NULL);                       /* ECB 不传 IV */

AES_Update_Ex(&ctx, plaintext, len, cipher, &out_len); /* 未满 16B 会暂存 */
AES_Final_Ex(&ctx, cipher + out_len, &out_len);         /* 自动 PKCS#7 补齐 1 块 */
```

**CBC 加密(带 IV):**

```c
stc_aes_ctx_t ctx;
uint8_t key[16] = { /* ... */ };
uint8_t iv[16]  = { /* 随机值,加解密必须同一 */ };

AES_Init_Ex(&ctx, AES_EX_MODE_CBC, AES_EX_DIR_ENCRYPT,
            key, AES_KEY_SIZE_16BYTE, iv);

AES_Update_Ex(&ctx, chunk1, len1, cipher,          &out1);
AES_Update_Ex(&ctx, chunk2, len2, cipher + out1,   &out2);
AES_Final_Ex(&ctx, cipher + out1 + out2,           &out3);
```

**CBC 解密(`Final` 同样是 3 参数,最后一个密文块由 `Update` 自动保留):**

```c
AES_Init_Ex(&ctx, AES_EX_MODE_CBC, AES_EX_DIR_DECRYPT,
            key, AES_KEY_SIZE_16BYTE, iv);  /* IV 和加密时相同 */

/* 每次 Update 必须输入 16B 整数倍,可多次调用,输出会延迟一块 */
AES_Update_Ex(&ctx, cipher + off, len, plain + out_off, &out);

/* Final 内部解密被保留的最后一块,校验并剥离 PKCS#7 */
AES_Final_Ex(&ctx, plain + out_off, &out);
```

> 解密时 `Update` 的输出比输入滞后一个 16B 块,因为最后一块可能带 padding,只能在 Final 校验.

**CTR(流密码,不需要 padding):**

```c
stc_aes_ctx_t ctx;
uint8_t nonce_ctr[16] = { /* 高 8B nonce + 低 8B 初始 counter(大端) */ };

AES_Init_Ex(&ctx, AES_EX_MODE_CTR, AES_EX_DIR_ENCRYPT, /* dir 被忽略 */
            key, AES_KEY_SIZE_16BYTE, nonce_ctr);

/* CTR 每次 Update 都立刻返回输出字节数,不需要 16B 对齐 */
AES_Update_Ex(&ctx, plaintext, len, cipher, &out);

AES_Final_Ex(&ctx, cipher + out, &out); /* CTR Final out=0,空操作 */
```

> **安全须知**:
> - ECB/CBC 解密时,每次 `Update` 的密文长度必须是 16B 整倍数,否则返回 `LL_ERR_INVD_PARAM`.
> - PKCS#7 在解密 Final 中做严格校验,padding 字节必须全部等于其值.
> - CTR 模式下同一 `(key, nonce)` 绝不能重复使用同一 counter 值,否则 keystream 会重复导致明文泄露.
> - 成功 Final 后 ctx 中的 key、buffer、ctr 均会自动清零,不会在 RAM 中残留密钥.

### 返回值

所有扩展 API 统一使用 `hc32_ll_def.h` 中的返回码:

| 返回值 | 含义 |
|---|---|
| `LL_OK` (0) | 成功 |
| `LL_ERR_INVD_PARAM` | 指针 NULL、参数越界、状态机非法或 padding 校验失败 |
| `LL_ERR_TIMEOUT` | 硬件 START 在超时窗口内未清零 |

### 多任务安全示例

```c
/* 任务 A:算固件 SHA256 */
stc_hash_ctx_t ctx_fw;
HASH_Init_Ex(&ctx_fw);
HASH_Update_Ex(&ctx_fw, fw_chunk, fw_len);
HASH_Final_Ex(&ctx_fw, fw_hash);

/* 任务 B:算通信包 HMAC */
stc_hmac_ctx_t ctx_comm;
HMAC_Init_Ex(&ctx_comm, key, key_len);
HMAC_Update_Ex(&ctx_comm, rx_pkt, rx_len);
HMAC_Final_Ex(&ctx_comm, mac);
```

两套 ctx 各自独立,但 **HASH/AES 硬件是共享的**.若两个任务并发调用,需在调用方外层加互斥保护:

```c
/* 以 RT-Thread 为例 */
#include <rtthread.h>

rt_mutex_t crypto_mutex = rt_mutex_create("crypto", RT_IPC_FLAG_PRIO);

rt_mutex_take(crypto_mutex, RT_WAITING_FOREVER);
HASH_Update_Ex(&ctx, data, len);
rt_mutex_release(crypto_mutex);
```

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
- cmake例子中, 添加了`FIRMWARE_VERSION_STR`和`FIRMWARE_VERSION` , 用来演示设置版本号例子, 添加了执行`clean`时也自动删除带版本号的map文件但是elf文件不带任何后缀,如果elf文件我名字一直变,对调试器非常不友好
- 添加了`PROJECT_VERSION_MAJOR`,`PROJECT_VERSION_MINOR`,`PROJECT_VERSION_PATCH` 宏定义, 下面是使用一个`uint32_t`来表示版本的示例
- cmake例子中, 添加了`option(USE_USB ON)`, 来默认开启USB支持, 打开和不打开USB支持, 对固件大小没有影响, 代码中真正使用了USB, 一定要打开支持, 并定义`usb_app_conf.h`, 没有这个头文件不会报错, 但是USB会无法使用, 因为缺少很多宏定义开关

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

- VsCode中调试工具:`Cortex-Debug`, 配置如下,`overrideGDBServerStartedRegex`这个是启动成功提示,如果有问题再加上

```json
 {
   // 使用 IntelliSense 了解相关属性。 
   // 悬停以查看现有属性的描述。
   // 欲了解更多信息，请访问: https://go.microsoft.com/fwlink/?linkid=830387
   "version": "0.2.0",
   "configurations": [

       {
           "name": "Cortex Debug",
           "cwd": "${workspaceFolder}",
           "executable": "./build/hc32f4a0-bootloader.elf",
           "request": "launch",
           "type": "cortex-debug",
           "runToEntryPoint": "main",
           "servertype": "pyocd",
           "targetId": "hc32f4a0xi",
           "gdbPath": "arm-none-eabi-gdb",
           //"overrideGDBServerStartedRegex": "GDB server listening on port"
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

set(FIRMWARE_VERSION_STR "1.0.0")
set(FIRMWARE_VERSION "")
find_program(GIT_EXECUTABLE git)
if(GIT_EXECUTABLE)
    # only hash
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE HASH_RESULT
        ERROR_QUIET
    )
    
    # check dirty
    execute_process(
        COMMAND ${GIT_EXECUTABLE} status --porcelain
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_STATUS
        ERROR_QUIET
    )
    
    if(HASH_RESULT EQUAL 0 AND GIT_HASH)
        set(FIRMWARE_VERSION "-${GIT_HASH}")
        if(GIT_STATUS)
            set(FIRMWARE_VERSION "-${FIRMWARE_VERSION}-dirty")
        endif()
    endif()
endif()
set(FIRMWARE_NAME "${CMAKE_PROJECT_NAME}-v${FIRMWARE_VERSION_STR}${FIRMWARE_VERSION}")
message(STATUS "Firmware name: ${FIRMWARE_NAME}")

# Driver function selection
set(MCU_TYPE "HC32F4A0xI" CACHE STRING "Set MCU Type")
set(BOOTLOADER OFF CACHE BOOL "Set to ON if bootloader is enabled")
set(APP OFF CACHE BOOL "Set to ON if app is enabled")
set(RT-THREAD OFF CACHE BOOL "Set to ON if RT-Thread is enabled")

# Enable printf and scanf float
option(ENABLE_PRINTF_FLOAT "printf report %f" OFF)
option(ENABLE_SCANF_FLOAT  "scanf report %f"  OFF)
# Include toolchain file
include("drivers/toolchain/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} VERSION ${FIRMWARE_VERSION_STR})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    OUTPUT_NAME ${CMAKE_PROJECT_NAME}
    ADDITIONAL_CLEAN_FILES "${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.map"
)

set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_CLEAN_FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${FIRMWARE_NAME}.map"
)
# "Generate hex and bin files and associate them with the clean target."
add_custom_command(
    TARGET ${CMAKE_PROJECT_NAME}
    POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${FIRMWARE_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${FIRMWARE_NAME}.bin
    BYPRODUCTS 
        ${FIRMWARE_NAME}.hex
        ${FIRMWARE_NAME}.bin
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

# USB support switch, enabled by default.
# Set it to ON only when the application uses the USB driver and provides
# its own usb_app_conf.h in one of the directories listed in HEADERS.

option(USE_USB "Enable USB support in the driver layer" ON)
if(USE_USB)
    # Expose application include directories to the driver layer so that
    # driver sources (e.g. hc32_ll_usb.c) can find application-provided
    # configuration headers such as usb_app_conf.h.
    target_include_directories(hc32f4a0_driver PUBLIC
        ${HEADERS}
    )
else()
    # Application include directories are only used by the application target.
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        ${HEADERS}
    )
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined libraries
)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    PROJECT_VERSION_MAJOR=${PROJECT_VERSION_MAJOR}
    PROJECT_VERSION_MINOR=${PROJECT_VERSION_MINOR}
    PROJECT_VERSION_PATCH=${PROJECT_VERSION_PATCH}
)
```
