# 小华单片机hc32f4a0系统cmake驱动
## 使用前准备
    - 安装MinGW编译器[下载地址](https://github.com/niXman/mingw-builds-binaries/releases)
    - 安装gcc-arm-none-eabi,交叉编译工具[下载地址](https://developer.arm.com/downloads/-/gnu-rm)
## 使用方法
    - topCmakeList.txt 文件，是顶层的cmake文件调用驱动的方法，设置toolchain很重要，toolchain位于驱动目录，顶层cmake要设置为绝对路径
    - 可以使用子模块的形式来管理驱动，也可以单独使用