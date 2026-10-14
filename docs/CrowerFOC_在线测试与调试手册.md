# CrowerFOC 在线测试与调试手册

本文供开发和实机测试使用，记录可复现命令、观测项、边界和当前连接限制。路径以 `C:\Users\Crowe\Desktop\MyMotor_ws` 为工程根目录。CAN 运控协议详见 [MIT CAN 协议说明](CrowerFOC_MIT_CAN协议说明.md)。

**当前验收状态（2026-10-14）：** 25kHz 电流环、USB 遥测和独立 d 轴扫频可用；最近扫频的 1.25–1.8kHz 点被判无效，附近可见增益峰，−3dB 带宽未能确定。不得把控制器的 1kHz 设计目标或单点通过旗标当作已经验证的带宽/稳定裕度。阶跃带宽分析已从固件和上位机移除。TEMP2 的外接 PTC 已按独立温度计数据 25℃/620Ω、48℃/848Ω 进行两点线性拟合；保护仍默认关闭，25–48℃之外的曲线与高温安全阈值尚未实测确认。CAN 无物理适配器时只完成离线验证；第二编码器尚待安装验收。持续电流、40A能力和热可靠性均未认证。

当前固件已移除按实测转速触发的跳闸；速度指令限制仍约束速度模式命令，力矩模式下电机可加速至机械负载与反电势限制。`0x0402`仅为旧固件兼容参数，当前固件不执行该保护；电流、母线电压、编码器、驱动故障和通信看门狗保护保留。此项变化要求运行时确保机械结构和电机本体不超过各自额定转速。

## 1. 当前连接识别

设备管理器即插即用枚举（2026-10-13 本次检查）确认：

| 设备 | 识别信息 | 用途 |
|---|---|---|
| 电机板 USB CDC | `VID_0483&PID_5710`，当前 COM7 | 上位机协议和遥测 |
| Horco CMSIS-DAP v2 | `VID_FAED&PID_4870`，设备状态 Started，MI_00 绑定 WinUSB；组合设备另有 COM9 | SWD 调试/烧录；COM9 不是 CrowerFOC CDC |

Windows 看得到探针说明 USB 枚举和驱动已建立，不能单独证明调试传输已打开。复核命令：

```powershell
pnputil /enum-devices /connected |
  Select-String -Pattern 'Horco CMSIS-DAP v2|VID_FAED|VID_0483|COM7|COM9' -Context 2,4
```

工程隔离的 pyOCD 环境：

```powershell
cd C:\Users\Crowe\Desktop\MyMotor_ws
.\.tools\pyocd-venv\Scripts\python.exe -m pyocd --version
.\.tools\pyocd-venv\Scripts\python.exe -m pyocd list -p
```

**重要：设备管理器可见，不等于 Codex 受限进程能访问 USB 探针。** 本机 Horco CMSIS-DAP v2 使用 WinUSB（VID/PID `FAED:4870`）。在沙箱里运行 `pyocd list` / libusb 扫描可能得到 `No available debug probes are connected`，这是 USB 设备访问隔离导致的假阴性；它本身不能证明探针掉线，也不是让用户反复重插、换驱动或用 Zadig 重绑接口的理由。COM7 是电机板 CDC，和探针是不同 USB 设备。

**Codex 中烧录前检查顺序：** 先确认设备管理器仍列出 `Horco CMSIS-DAP v2`；如果仅沙箱内 pyOCD 找不到探针，直接把 `pyocd list -p` 以允许直接访问 USB 的方式运行（本环境即对 `exec_command` 使用 `require_escalated`），不要先重复普通沙箱扫描。出现探针 UID `846703532218` 后，再以同样的 USB 访问方式运行本节 `flash_fast.ps1`。只有直接访问 USB 的枚举仍失败，才继续查物理连接、驱动或占用。烧录脚本现已在尝试多个 SWD 频率前做探针预检，避免对“根本不可见”的设备逐频率空试。

最近一次复现记录（2026-10-14）：普通受限进程 `pyocd list` 看不到探针；允许直接 USB 访问后识别 Horco（UID `846703532218`），`flash_fast.ps1` 以 1MHz 烧录成功（9.9s），硬件复位成功，电机板 USB CDC 重新枚举为 COM7。不要为了扫描探针更换 WinUSB 驱动或用 Zadig 重绑接口。

串口在 GUI 和脚本间只能由一个进程占用。COM7 的 `PermissionError(13, Access denied)` 表示当前进程无法打开端口；关闭 GUI、串口助手和遗留测试进程后再检查。CDC 波特率由当前 HostUI/协议客户端打开为 115200；旧诊断脚本可能使用不同速率，应以脚本自身的 `serial.Serial(...)` 为准。端口号应通过 VID/PID 查找，不应假定永远是 COM7。

## 2. 构建、主机测试与固件烧录

先在工程根目录运行主机逻辑回归，再构建 MCU 工程：

```powershell
cd C:\Users\Crowe\Desktop\MyMotor_ws\HostUI
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

cd ..\Software\MotorTest_cmake
cmake --build --preset debug_GCC_STM32C562CET6
```

项目推荐烧录脚本从 `under-reset` 模式开始，首选 1MHz SWD，失败后依次尝试 500/250/100kHz，之后尝试硬件复位并等 USB CDC 重枚举：

```powershell
cd C:\Users\Crowe\Desktop\MyMotor_ws\Software\MotorTest_cmake
powershell -NoProfile -ExecutionPolicy Bypass -File .\flash_fast.ps1
```

成功判定必须有 `[flash] OK @ ...`，且脚本没有报告所有频率失败。最后的 USB 枚举只证明设备出现，不能替代协议读取。烧录后先检查状态/故障和参数，再决定是否转动。`flash.bat` 使用旧 `.exe` 启动器，不是当前推荐入口；采用 `python -m pyocd` 的 `flash_fast.ps1`。

本次工作记录（2026-10-13）：`MotorTest.elf` SHA256 为 `5801DC9426DA2D9D8C84930225FD3B45D70517C2E879F5F78FA7EB848F9A7648`；对应固件已烧录并在线验收。构建通过（最终检查 `ninja: no work to do`），HostUI 单元测试 34/34 通过。完整回归报告为 `tools/axis_feature_results/20261013_083739_features.json`。最终状态回读：IDLE、fault=0、目标电流/速度为零，运行电流上限 40A、速度指令上限 230rpm、过速跳闸 253rpm、母线过压 32V，校准有效且配置已保存。

固件修复：校准最后阶段原先在 25kHz 控制中断内执行平方根、反正切和多次三角函数；实测 ISR 峰值约 6269 周期，超过 40µs预算并锁存 deadline 故障。现在 ISR 只安全停止 PWM 并提交待处理标志，校准数学和结果写回移到主循环。重测完整 5A 校准、Flash 保存与重启回读通过；最终 ISR 峰值 24.19µs。

## 3. 只读在线检查

`tools/final_status_probe.py` 使用设备 USB 协议查询主要参数，并要求最终状态为 IDLE、无故障、零速度/力矩目标；它会发送心跳和 GET，不下发闭环/标定命令，并把结果写入 `tools/final_device_status.json`：

```powershell
cd C:\Users\Crowe\Desktop\MyMotor_ws
.\.tools\host-venv\Scripts\python.exe .\tools\final_status_probe.py
```

判定项：遥测持续更新，`state=1`、`fault=0`，目标速度和目标电流为零；母线电压与电源实测相符；标定标志、极对数和编码器状态有效。COM7 被占用时先关闭占用进程，不要重复启动多个 probe。

设备输出轴编码器（第二 TLE5012B）在固件中和电机轴编码器共用 SSC 时钟/数据，独立片选 PB6；主板原电机编码器片选 PB4、SCK PB3、DATA PB5。硬件网络名 `SPI3_CS_ENC2` 对应 PB6。HostUI 通过 `0x0840` 启用，`0x0843` 读健康状态，`0x0844` 读累计失败次数，`0x0845/0x0846` 读输出轴角度/速度。先保持关闭，机械手动转动验证读数连续、方向正确，再启用并保存。启用后控制位置/速度和反馈以输出轴编码器为准，不使用固定减速比换算。

## 4. 有限动作的硬件验收

`tools/axis_feature_probe.py --snapshot` 仅收集初始参数；`--accept` 会实际转动，不要在机构受限、带负载或手/工具靠近轴时运行：

```powershell
cd C:\Users\Crowe\Desktop\MyMotor_ws
.\.tools\host-venv\Scripts\python.exe .\tools\axis_feature_probe.py --snapshot
.\.tools\host-venv\Scripts\python.exe .\tools\axis_feature_probe.py --accept
```

`--accept` 的当前实现包含：参数边界拒绝测试；5A 完整标定（校准硬限流 6A，原始跳闸 8A，电机空载缓慢转动）；保存/重启/回读；输出编码器状态判定；USB 心跳超时停机；最后恢复并保存 40A、230rpm、253rpm 配置。输出编码器启用、健康且实测减速比有限时，才运行 ±30rpm 和当前位置附近 +20° 位置测试；本次编码器板未安装，回归验证固件拒绝速度闭环，并跳过位置测试。随后以1A运行限流做±0.2A短时力矩点动，速度边界20rpm、相电流边界1.5A。该实现避免了未安装输出反馈时错误测试位置/速度控制。

本次回归通过参数边界、校准、Flash 保存及重启、配置事务拒绝无效项、无输出编码器时安全拒绝速度闭环、±0.2A点动、USB心跳断线停机和最终配置恢复。点动遥测中的平均 Iq 分别为 +0.044A 与 −0.063A（目标±0.2A），速度峰值低于0.28rpm；这证明了低幅度正反向响应和安全停机，但电流跟踪精度需要单独继续检查。另一次独立短点动记录的跟踪更接近指令，见 `tools/current_loop_results/20261013_torque_smoke.json` 与配套 PNG/CSV，因此不能把单次均值当作精度验收。

最终回归记录 `tools/axis_feature_results/20261013_083739_features.json` 显示 ISR 峰值 24.19µs；回归结束时 ADC 超时诊断累计计数为27，没有锁存 ADC 故障。随后只读状态脚本多次查询温度/ADC 参数后，计数读到77，状态仍为 IDLE、fault=0。该计数是诊断量，不作为零值断言；累计增长原因尚未隔离，若运行时继续增长或故障位出现，应单独查同步注入和常规 ADC 转换的超时来源。最终状态快照见 `tools/final_device_status.json`。

顺序应为：先只读快照；确认空载和可自由转动；确认编码器校准有效、无故障、温度/母线条件可接受；再执行有限动作验收。`--accept` 当前会把软件运行限流临时设为1A，并在结束时恢复40A、230rpm、253rpm并保存。结束后仍需复查状态 IDLE、目标为零和保存序号。若中途异常，先断开控制输入并确认驱动已回到 IDLE，再保留失败 JSON；不要自动清故障后继续。

`tools/drive_full_test.py` 是早期测试流程，速度单位仍使用旧协议 rad/s；不建议把它作为当前 rpm/° 接口验收入口。当前公开单位映射和扩展协议回归优先用 `axis_feature_probe.py` 及 HostUI 单元测试。

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

板载 NTC 和电机侧温度传感器已接入；但电机侧传感器阻值/类型曲线尚未确认，启用保护前需核对室温读数、断开测阻、温度变化方向和阈值配置。过温保护可在上位机单独启闭，关闭会使对应通道不参与降额/跳闸。母线过压阈值目标为32V，仍须核对分压量程、实际电源和器件耐压。40A 是配置允许的峰值限流，不是已验证的板级连续额定值。MIT CAN 已实现离线编解码，真实总线仍需适配器和主控验收。第二编码器和位置闭环需等输出编码器安装后验收；空载小角度结果不能代表负载定位精度或长时间热可靠性。

相关记录：`tools/DRIVE_FULL_TEST_REPORT.md`、`tools/CURRENT_SAMPLING_REPORT.md`、`tools/SPEED_LIMIT_DIAGNOSIS_20261009.md`、`tools/axis_feature_results/20261013_083739_features.json`、其点动曲线/CSV，以及 `tools/current_loop_results/20261013_torque_smoke.{json,csv,png}`。
