
# Mini-For-Developer

## 目录

- [Mini-For-Developer](#mini-for-developer)
  - [目录](#目录)
  - [项目简介](#项目简介)
  - [目录结构](#目录结构)
  - [安装和使用](#安装和使用)
    - [环境信息](#环境信息)
  - [更新日志](#更新日志)
    - [2025-12-03](#2025-12-03)
  - [贡献](#贡献)
  - [许可证](#许可证)

## 项目简介
此项目为，对于XGOMINI4的底层固件支持
## 目录结构
-  Mini-For-Developer:
    - components:示例程序
      - icm20948：传感器库
      - servo_driver：舵机驱动库
      - uart_comm：uart通用库
      - other：led等其他驱动库
    - config：UART和舵机的配置信息
    - main：主程序
## 安装和使用
### 环境信息
   1. win10
   2. IDF版本：4.4.8.1
   
1. 克隆本仓库：
    ```bash
    git clone https://github.com/Xgorobot/RaspberryPi-CM4-XGO-Dog.git
    ```

2. 进入项目目录：
    ```bash
    cd Mini-For-Developer
    ```

3. 编译并烧录

## 更新日志
### 2025-12-03
- **代码改进** 搭建了初步框架

## 贡献
欢迎贡献！我们欢迎任何建议、修复和功能增强。如果你有兴趣为这个项目贡献，可以按照以下步骤操作:  
1.Fork 本仓库  
2.创建你自己的分支 (git checkout -b feature-branch)  
3.提交你的修改 (git commit -m 'Add new feature')  
4.Push 到你的分支 (git push origin feature-branch)  
5.提交 Pull Request

## 许可证
此项目遵循 MIT 许可证。

