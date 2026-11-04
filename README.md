# CrowerFOC

基于 STM32C562CET6 的单轴无刷电机驱动器，包含 FOC 固件、PyQt 上位机和主板/输出轴编码器网表。

项目已完成初步实用验证：电流环和速度环已验证，进行了 **40 A、2 秒脉冲测试**和持续旋转测试等，USB 上位机已验证。串口和 CAN 通信尚未完成完整可靠性验证。完整标定使用约 5 A，电机可能转动；首次调试应固定电机、空载并准备切断母线。

**项目并非完整成品。** 硬件和上位机已具备基本可用性，不保证所有功能均已成熟或完成验证。

## 功能

- 25 kHz 中心对齐 PWM、电流采样和 d/q 电流环；支持电流、速度及位置控制、R/L 和编码器换相标定、d 轴闭环扫频。
- USB CDC 和 UART4 调试；CAN 使用 MIT 8 字节布局，但最后的控制字段和反馈为 **Iq 电流**，并非原版扭矩语义。
- PyQt 上位机提供曲线、参数、故障与诊断、CAN 报文编解码、MIDI 电机音乐；系统声音播放为可选实验功能。
- 可选第二编码器用于输出轴；该功能需接线、安装及独立验收。

## 待办

- [ ] 测试第二编码器 V2 的通信、角度、方向及输出轴反馈。
- [ ] 优化器件布局，并将 FPC 座子移至背面。
- [ ] 增加 Type-C 电源防倒灌保护。
- [ ] 完成位置环调试与验证，包括位置响应、边界和负载表现。

## 快速开始（Windows）

安装 Python 3.10 或更新版本，在仓库根目录的命令提示符中运行：

```bat
py -3 -m venv HostUI\.venv
HostUI\.venv\Scripts\python.exe -m pip install -r HostUI\requirements.txt
HostUI\run_gui.bat
```

在“电机音乐”页点击“载入《你》”，选择 `Piano RH` 旋律音轨。系统声音采集需安装 `HostUI/requirements-audio.txt`；播放会给电机施加电流。

固件使用 CMake 3.30+、Ninja 和 Arm GNU Toolchain 14.3.1（`arm-none-eabi-gcc` 在 PATH 中）：

```bat
cd Software\MotorTest_cmake
cmake --preset debug_GCC_STM32C562CET6
cmake --build --preset debug_GCC_STM32C562CET6
```

Windows 下可使用 `flash.bat list` 检查 Horco CMSIS-DAP，使用 `flash.bat` 构建、烧录并复位。pyOCD 和器件包准备方法见[调试手册](docs/CrowerFOC_在线测试与调试手册.md)，接线及操作见[快速使用指南](docs/CrowerFOC_快速使用指南.md)。

## 硬件与连接

| 项目 | 配置 |
|---|---|
| MCU | STM32C562CET6 |
| 相电流 | INA240A2、0.5 mΩ 分流器，ADC1 PA4/PA5/PA6 |
| 电机侧编码器 | TLE5012B，PB3 SCK、PB5 DATA、PB4 CS |
| 输出侧编码器 | 可选 TLE5012B，共用时钟/数据，PB6 CS |
| 母线采样 | PA0/ADC2_IN0，10kΩ/1kΩ 分压 |
| UART4 | PB8 RX、PB9 TX、3.3 V TTL、921600 bit/s |
| CAN | FDCAN1 PB12 RX、PB13 TX；详见 [MIT 布局协议](docs/CrowerFOC_MIT_CAN协议说明.md) |

`Hardware/CrowerFOC.epro2` 是立创 EDA 专业版工程，`Hardware/` 还包含主板及第二编码器网表。可直接查看[主板原理图第 1 页](Hardware/schematics/mainboard-page1.png)、[第 2 页](Hardware/schematics/mainboard-page2.png)和[第二编码器 V1 原理图](Hardware/schematics/encoder2-v1.png)。PCB 和生产资料请在立创 EDA 工程中核对；元件规格书请从厂家获取。

## 项目结构与测试

- `Software/MotorTest_cmake/`：MCU 固件、Cube 生成文件、第三方组件和烧录入口。
- `HostUI/`：上位机、通信协议、MIDI 播放与单元测试。
- `Hardware/`：主板及第二编码器网表。
- `docs/`：使用、调试、CAN 协议及历史扫频记录。
- `tools/tests/`：固件辅助函数的 ARM 仿真测试；`tools/` 保留可复用工具和历史报告。

```bat
HostUI\.venv\Scripts\python.exe -m unittest discover -s HostUI\tests -v
```

ARM 仿真测试还需要 `unicorn` 与 Arm GNU Toolchain；见[调试手册](docs/CrowerFOC_在线测试与调试手册.md)。

## 已知限制

CAN 已做 1 Mbps、低电流收发测试；断帧/恢复、第三方主控的可靠性尚未全量验收。第二编码器的输出轴测量未验收。1 kHz 是控制目标而非经过所有工况验证的带宽；历史扫频及有效点详见 [记录](docs/validation/current_loop_sweep_2026-10-14.png)。电机侧温度传感器模型基于两点拟合，保护配置和温度阈值必须按实际硬件检查。

## 许可

项目原创代码采用 [MIT License](LICENSE)；ST/CMSIS/USBX 等第三方代码及其生成文件按各自声明许可。随附 MIDI 由用户提供，不属于项目代码的 MIT 许可；音乐作品及其改编的使用权由相关权利人决定。详见 [第三方说明](THIRD_PARTY_NOTICES.md)。
