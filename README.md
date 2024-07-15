# 配好调试环境的xv6-riscv
### 这个能提供什么
- 已经配置好的调试环境，可以直接使用 Visual Studio Code (VSCode) 进行调试。
- 支持代码跳转功能。
- 由其它库提供的xv6-riscv的中文文档.

## 调试代码
>首次调试会弹出窗口,点击“记住我的决定”,然后点击“继续DEBUG".  

使用 VSCode 的调试工具，可以利用已经配置好的调试步骤来启动调试会话。

## 代码跳转功能
- 更新代码后使用跳转功能时需要在 xv6-riscv 项目的根目录下执行 make 命令生成代码之间的关系.

# 使用方式
>环境配置及前提条件
>- 推荐使用 Debian 系统，其他 Linux 发行版可能需要自行解决库的安装问题。(可以搭建虚拟机,使用VSCode的ssh远程开发)
>- 需要使用 VSCode 作为编辑器。

### 一、 相关库的安装命令
>这个命令会安装必要的构建工具、RISC-V 架构的二进制工具、编译器、模拟器（QEMU）、调试器（GDB）和 clangd。
- 针对Debian系统:
```sh
    apt install make bear binutils-riscv64-unknown-elf gcc-riscv64-unknown-elf clang qemu-system-misc gdb-multiarch clangd -y
```

### 二、安装VSCODE插件
- clangd 插件用于代码自动跳转。
- Native Debug 插件用于代码调试。

### 三、加载中文文档
- 在 xv6-riscv 根目录下执行以下命令来加载中文文档:
```sh
    git submodule init
    git submodule update
```
