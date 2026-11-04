# CrowerFOC 在线测试与调试手册

本文供开发和实机测试使用，记录可复现命令、观测项、边界和当前连接限制。下文的 `<repo>` 代表仓库根目录。CAN 运控协议详见 [MIT CAN 协议说明](CrowerFOC_MIT_CAN协议说明.md)。

**当前验收状态（2026-10-27）：** 25kHz 电流环、USB 遥测和独立 d 轴扫频可用；最近扫频的 1.25–1.8kHz 点被判无效，附近可见增益峰，−3dB 带宽未能确定。不得把控制器的 1kHz 设计目标或单点通过旗标当作已经验证的带宽/稳定裕度。阶跃带宽分析已从固件和上位机移除。TEMP2 的外接 PTC 已按独立温度计数据 25℃/620Ω、48℃/848Ω 进行两点线性拟合；保护仍默认关闭，25–48℃之外的曲线与高温安全阈值尚未实测确认。CAN 已完成 PCAN-USB 1Mbps 收发及 1A 限流下 ±0.2A 电流命令验证；命令断帧看门狗、bus-off 恢复、第三方主控互操作和高电流能力仍待验收。第二编码器当前未连接。持续电流、40A能力和热可靠性均未认证。

当前固件已移除按实测转速触发的跳闸；速度指令限制仍约束速度模式命令，力矩模式下电机可加速至机械负载与反电势限制。`0x0402`仅为旧固件兼容参数，当前固件不执行该保护；电流、母线电压、编码器、驱动故障和通信看门狗保护保留。此项变化要求运行时确保机械结构和电机本体不超过各自额定转速。

## 1. 当前连接识别

设备管理器即插即用枚举（2026-10-13 本次检查）确认：

| 设备 | 识别信息 | 用途 |
|---|---|---|
| 电机板 USB CDC | `VID_0483&PID_5710`，当前 COM7 | 上位机协议和遥测 |
| Horco CMSIS-DAP v2 | `VID_FAED&PID_4870`，设备状态 Started，MI_00 绑定 WinUSB；组合设备另有无线串口 COM9 | SWD 调试/烧录；COM9 桥接目标 UART4 时使用 921600 bit/s |
| 板载 UART4 | Horco COM9 无线串口或外接 USB-UART 转接器 | 同一上位机协议；固件 921600 bit/s |

主板网表还确认板载 UART4 调试接口为 `CN2`：CN2-1=`UART4_RX`/PB8，CN2-2=`UART4_TX`/PB9，CN2-3=`GND`。UART4 是 3.3V TTL、8N1、921600 bit/s；上位机端选择对应 USB-UART 转接器 COM 口及 921600。连接时转接器 TX→CN2-1、RX→CN2-2、GND→CN2-3。不得向该接口输入 RS-232 或 5V TTL 电平。USB CDC 与 UART4 共用同一 CrowerFOC 帧协议和 HostUI 功能；一台主机只连接一条链路作为控制端。

Windows 看得到探针说明 USB 枚举和驱动已建立，不能单独证明调试传输已打开。复核命令：

```powershell
pnputil /enum-devices /connected |
  Select-String -Pattern 'Horco CMSIS-DAP v2|VID_FAED|VID_0483|COM7|COM9' -Context 2,4
```

工程隔离的 pyOCD 环境（首次使用需安装 pyOCD 和器件包）：

```powershell
cd <repo>
py -3 -m venv .tools\pyocd-venv
.\.tools\pyocd-venv\Scripts\python.exe -m pip install pyocd
.\.tools\pyocd-venv\Scripts\python.exe -m pyocd pack update
.\.tools\pyocd-venv\Scripts\python.exe -m pyocd pack install stm32c562cet6
.\.tools\pyocd-venv\Scripts\python.exe -m pyocd --version
.\.tools\pyocd-venv\Scripts\python.exe -m pyocd list -p
```

**探针可见性检查：** 设备管理器可见只说明 Windows 枚举到了设备，仍需由 pyOCD 成功打开 CMSIS-DAP 接口。请从 CMD 运行 `flash.bat list`；如果它报告未发现探针，再检查线缆、USB 口、Horco 驱动/接口状态和是否有调试软件占用。不要仅凭一次 pyOCD 枚举失败就更换驱动或用 Zadig 重绑接口。COM7 是电机板 CDC，与调试探针是不同 USB 设备。

最近一次 UART4 联调记录（2026-10-14）：COM9 可打开但无数据。pyOCD attach 读取 UART4_CR1=`0x2C`，发现 UE(bit 0) 未置位；临时置 UE 后 COM9 即收到遥测，`GET 0x0100` 成功返回极对数 21。原因是 UART HAL 配置后没有显式使能外设。已在 `phase_test_uart_init()` 补上 UE 使能；需要烧录该修复后确认复位启动仍能持续收发。探针识别和烧录均通过 `flash.bat` 操作；如果脚本识别不到探针，再检查 Windows 设备枚举、USB 连接、接口驱动和占用情况。

串口在 GUI 和脚本间只能由一个进程占用。USB CDC 在 HostUI 中使用 115200（CDC 实际吞吐不受该设置限制）；UART4（包括 Horco COM9 无线串口）使用 921600 bit/s，HostUI 选择对应端口后会默认选择该速率，也可在顶部波特率框修改。端口号应通过设备描述/VID/PID 识别，不应假定永远是 COM7。

## 2. 构建、主机测试与固件烧录

从仓库根目录运行主机逻辑回归，再构建 MCU 工程。首次构建需先运行 `cmake --preset debug_GCC_STM32C562CET6`：

```powershell
cd <repo>\HostUI
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

cd ..\Software\MotorTest_cmake
cmake --build --preset debug_GCC_STM32C562CET6
```

从 CMD 使用 `flash.bat` 烧录。无参数时会自动构建、识别 Horco 探针、尝试烧录并复位；也可先运行 `flash.bat list` 只检查探针：

```bat
cd /d <repo>\Software\MotorTest_cmake
flash.bat list
flash.bat
```

支持 `flash.bat uid 846703532218` 指定探针、`flash.bat skip-build` 使用已有 ELF、`flash.bat firmware "C:\path\MotorTest.elf"` 指定固件，以及 `flash.bat skip-verify` 跳过被动 UART 验证。成功判定应看到 `[flash] 烧录成功` 和 `[flash] 复位成功`；烧录后先检查状态/故障和参数，再决定是否转动。`flash_fast.ps1` 是批处理入口内部调用的实现文件，日常操作只需使用 `flash.bat`。

本次工作记录（2026-10-13）：`MotorTest.elf` SHA256 为 `5801DC9426DA2D9D8C84930225FD3B45D70517C2E879F5F78FA7EB848F9A7648`；对应固件已烧录并在线验收。构建通过（最终检查 `ninja: no work to do`），HostUI 单元测试 34/34 通过。完整回归报告为 `tools/axis_feature_results/20261013_083739_features.json`。最终状态回读：IDLE、fault=0、目标电流/速度为零，运行电流上限 40A、速度指令上限 230rpm、过速跳闸 253rpm、母线过压 32V，校准有效且配置已保存。

固件修复：校准最后阶段原先在 25kHz 控制中断内执行平方根、反正切和多次三角函数；实测 ISR 峰值约 6269 周期，超过 40µs预算并锁存 deadline 故障。现在 ISR 只安全停止 PWM 并提交待处理标志，校准数学和结果写回移到主循环。重测完整 5A 校准、Flash 保存与重启回读通过；最终 ISR 峰值 24.19µs。

## 3. 在线检查与有限动作验证

在上位机检查空闲状态、故障、母线、ENC1 和标定状态。调试端口号会变化，设备管理器枚举不代表串口未被占用。

只读故障快照可在仓库根目录执行：

```bat
HostUI\.venv\Scripts\python.exe tools\read_fault_snapshot.py
```

第二编码器使用 PB6 独立片选；通过 `0x0840` 启用，`0x0843` 读健康状态，`0x0844` 读累计失败次数，`0x0845/0x0846` 读角度/速度。未连接时保持关闭；速度模式使用电机侧反馈，位置模式需要有效输出轴反馈。接入后先检查角度连续性和方向，再验证传动比与控制。

有限动作检查应先确认机械可自由转动、母线和温度正常、ENC1 标定有效；用小电流短时点动并观察原始电流和故障。不同固件版本和硬件状态的历史结果不能替代本次验收。ENC1 低能量检查见第 7 节。

## 5. 电流采样和波形判读

- 电流环目标带宽参数 `0x0107` 在协议内部以 rad/s 存储、HostUI 以 Hz 显示；1kHz 对应约 6283rad/s。目标参数用于 PI 设计，按有效 R/L 计算，不等同于实测带宽。完整标定得到的 R/L 是逆变器与电机在线估算值，需结合独立仪表判断；R 用 mΩ、L 用 µH 显示。
- 独立扫频配置为 100–4000Hz、16点、d轴注入。每点须满足激励电流≥0.5A、统计窗口电压饱和≤1%、增益有限且<2倍，才参与有效带宽估算。任意无效频点会使当前保守算法无法计算完整−3dB交叉。2026-10-14 新扫频中100–1000Hz点有效，1.25/1.5/1.8kHz点无效并出现峰值，后续高频段下降；截图无法确认无效原因，需查看该三行的注入电流和饱和率。结果和 CSV 见 `docs/validation/current_loop_sweep_2026-10-14.*`。阶跃带宽分析已删除，100Hz USB 遥测不用于高频环路验收。
- PWM 当前为 25kHz 中心对齐；TIM1 触发 ADC 注入采样，参数 `0x0620=1` 表示硬件同步采样路径启用。
- 原始采样在控制滤波前仍用于过流保护；三点中值和 EMA 只用于控制输入/显示，不能把滤波后曲线当作原始峰值。
- `0x0600` 为原始大于约 6.45A 的累计计数，`0x0601` 为峰值计数；`0x0610` 为 ADC 异常；`0x0618` 为滤波前后相差较大的诊断计数；`0x0619/0x061A` 为 ADC/ISR 最长执行周期。
- USB 约 100Hz 遥测不能完整呈现 25kHz 开关纹波。原始短记录和已有比较结论见 `tools/CURRENT_SAMPLING_REPORT.md` 与 `tools/DRIVE_FULL_TEST_REPORT.md`。记录原始值、母线电压、转速/角度、控制模式、指令、PWM频率和固件 SHA256，避免只截取滤波曲线。
- 本次完整回归结束时 `0x0610` ADC 超时诊断计数为27、`fault=0`；它没有触发保护。该量在校准/点动期间累计，测试记录保留其值，后续应结合新增计数、ADC 状态和示波器触发波形判断，不能单凭非零累计值判定故障。
- 任何改变模拟 RC、ADC 触发或采样时序的改动都要重新验证零点、原始峰值、Iq 跟踪误差、三相和残差、ISR 时间与高调制度下有效采样窗口；不得仅以“曲线变平”判定改善。

## 6. 诊断结果与未完成的实物验证

GUI 的诊断模式会从 `Hardware\电调主板.net` 和 `Hardware\电调第二编码器.net` 读取器件清单，执行软件可观测项并为网表器件列出失效情景。它是检查表和逻辑模拟，不会物理注入器件短路、开路或过温，也不能代替示波器/万用表测量。

板载 NTC 和电机侧温度传感器已接入；但电机侧传感器阻值/类型曲线尚未确认，启用保护前需核对室温读数、断开测阻、温度变化方向和阈值配置。过温保护可在上位机单独启闭，关闭会使对应通道不参与降额/跳闸。母线过压阈值目标为32V，仍须核对分压量程、实际电源和器件耐压。40A 是配置允许的峰值限流，不是已验证的板级连续额定值。MIT CAN 已完成 PCAN-USB 1Mbps 收发和 1A 限流下 ±0.2A 指令的有限实机验证；超时断帧、bus-off 恢复和第三方主控互操作仍待验证，详见 `CrowerFOC_MIT_CAN协议说明.md`。第二编码器当前未连接，输出轴角度/速度/减速比及位置闭环尚未验收；空载小角度结果不能代表负载定位精度或长时间热可靠性。

历史记录：`tools/DRIVE_FULL_TEST_REPORT.md`、`tools/CURRENT_SAMPLING_REPORT.md`、`tools/SPEED_LIMIT_DIAGNOSIS_20261009.md`。CAN 协议与实机验证边界见 `docs/CrowerFOC_MIT_CAN协议说明.md`。

## 7. ENC1 换相映射的低能量实机检查

这项检查用于判断“编码器1机械角→电角度→Park角度”的符号、极对数和零偏是否一致；不能证明负载能力或代替完整标定。未标定、编码器状态不健康、母线/温度异常或轴被机构约束时不要开始。

1. 固定电机本体，拆除负载并清空轴周围空间；确保可立即断开母线。确认 ENC1 读数稳定、故障为 0，记录固件版本/ELF SHA、`pp`、`dir`、电角度零偏、映射质量及母线电压。不要为了本检查启动自动标定。
2. 空闲时把运行限流临时设为 **1A**，速度指令限值设为 **5rpm**，应用后读回确认。该设置不是电流模式下的实际转速保护，仍需人工监控。选用直接 Iq 电流模式；先记录空闲 2 秒基线。
3. 在闭环中依次给 `+0.2A` 250ms、`0A` 1秒、`−0.2A` 250ms、`0A` 1秒，最多重复 3 轮。若电流跟踪稳定且没有可见异常、但静摩擦使角度完全不动，可将幅值提高到 `±0.4A` 再做一次；全程不得超过 1A。
4. 每个脉冲同时记录时间戳、Iq 目标/实测、Id、三相电流（注明原始或处理后）、ENC1 原始角度和展开机械角、换相电角度、实际 Park 电角度、两者误差、速度 rpm、Vq/PWM 占空比、饱和标志、故障/编码器健康位。每轮后立刻发 0A；若速度超过 5rpm、250ms 内机械角变化超过 5°、电流明显跳变、异响/振动或出现故障，立即停止并断开母线，不再重试。
5. 检查关系 `elec = wrap(dir × pp × mech + offset)` 是否连续、方向一致；运动方向反转时，电流符号应随之反转，Park 角误差不应随角度持续漂移。完成后回到 IDLE，确认目标电流为 0，再恢复原运行限制并读回。保留 CAN Rx/Tx/drop 计数（若走 CAN）、USB/UART 日志和原始 CSV；没有这些同步字段的截图不能单独作为换相通过依据。

该步骤只提供低能量排查模板，当前没有新的实机采样记录，不能据此判定换相问题已经关闭。若 1A 限流下无法克服静摩擦，应先改善无负载/摩擦条件，不要直接提高电流。
