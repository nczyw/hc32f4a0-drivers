# 小华单片机hc32f4a0系统cmake驱动
## 介绍
    - 支持配置为bootloader,app,normal三种模式
    - bootloader模式下，flash前64KB为bootloader，其余为app
## 使用前准备,[MinGW](https://github.com/niXman/mingw-builds-binaries/releases),[gcc-arm-none-eabi](https://developer.arm.com/downloads/-/gnu-rm)
    - 安装MinGW编译器
    - 安装gcc-arm-none-eabi,交叉编译工具
## 使用方法
    - topCmakeList.txt 文件，是顶层的cmake文件调用驱动的方法，设置toolchain很重要，toolchain位于驱动目录，顶层cmake要设置为绝对路径
    - 可以使用子模块的形式来管理驱动，也可以单独使用