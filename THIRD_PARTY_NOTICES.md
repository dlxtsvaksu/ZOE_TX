# 第三方组件与许可声明 / Third-Party Notices

本仓库根目录的 `LICENSE`（MIT）只覆盖本项目作者编写的代码，主要包括：

- `Task_Schedule/`（调度、CRSF、PPM、混控、微调、Flash 存储、UI、USB HID 封装）
- `Core/` 里的 `USER CODE` 段内容（CubeMX 生成框架本身由 ST 提供）
- `cmake/`、`CMakeLists.txt`、`CMakePresets.json`、`.gitignore`、`.gitattributes`、根目录脚本与文档

以下目录/组件版权归 STMicroelectronics 或第三方所有，适用其各自的许可，**不受本项目 MIT 许可覆盖**：

| 组件 | 位置 | 许可 |
| --- | --- | --- |
| STM32F1 HAL 驱动 | `Drivers/STM32F1xx_HAL_Driver/` | BSD-3-Clause（见该目录 `LICENSE.txt`；随完整软件包分发时以 `Package_license` 为准） |
| CMSIS Core | `Drivers/CMSIS/` | Apache-2.0（见 `Drivers/CMSIS/LICENSE.txt`） |
| CMSIS Device STM32F1xx | `Drivers/CMSIS/Device/ST/STM32F1xx/` | Apache-2.0（见该目录 `LICENSE.txt`） |
| STM32 USB Device Library | `Middlewares/ST/STM32_USB_Device_Library/` | Ultimate Liberty license SLA0044（见各源文件头部注释，<https://www.st.com/SLA0044>） |
| STM32CubeMX 生成的初始化代码 | `Core/Src/*.c`、`Core/Inc/*.h` 中非 `USER CODE` 区域 | 随 STM32Cube 软件包条款 |

## OLED 驱动

`oled_multi/`（`oled.c` / `oled.h` / `font.c` / `font.h`）基于 **波特律动（keysking）STM32 教程** 配套 OLED 驱动库移植/修改而来，并针对本项目的 SH1106/SSD1306 屏、
中文显示与非阻塞分页刷新做了改动。本仓库未附带该上游库的许可文件；**若上游许可与本项目 MIT 不一致，以上游许可为准**。再分发前请自行核对上游仓库的授权条款。

## 商标

STM32、STM32Cube 是 STMicroelectronics International N.V. 的商标。CRSF / ELRS 相关名称归其各自权利人所有。
本项目与 STMicroelectronics 无隶属或背书关系。
