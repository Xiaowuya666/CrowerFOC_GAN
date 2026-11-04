# CrowerFOC 上位机

Windows 图形上位机使用 PyQt5，可通过 USB CDC 或板载 UART4 与电机板通信。两种链路使用相同帧协议和调试功能；UART4 需 3.3V USB-UART 转接器，921600 bit/s。用户操作、控件状态、安全边界和当前验证限制统一见工程根目录 [快速使用指南](../docs/CrowerFOC_快速使用指南.md)；接线/烧录细节见 [在线测试与调试手册](../docs/CrowerFOC_在线测试与调试手册.md)。

## 启动

```bat
py -3 -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
run_gui.bat
```

USB CDC 默认使用 115200；选中普通 USB-UART 设备时波特率自动切换为 921600，也可在工具栏手动选择。脚本使用本地 `HostUI\.venv` Python 环境。电机音乐页可选择随附的《你》MIDI 或导入自己的文件。

## 测试

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

UI文件为 `crowerfoc_gui.py`；帧定义、通信线程及逻辑模块分别见 `crowerfoc_protocol.py`、`crowerfoc_transport.py`、`crowerfoc_client.py`。CAN 电流语义和帧布局见 [MIT-layout CAN 协议](../docs/CrowerFOC_MIT_CAN协议说明.md)。

“电机音乐”页中的“电脑声音 → 电机”是可选实验功能，依赖 Windows WASAPI 回环采集。安装：

```bat
.venv\Scripts\python.exe -m pip install -r requirements-audio.txt
```

该模式以 8 kHz 单声道 PCM 经 USB 传输，默认 q 轴峰值电流 1 A、上限 5 A；实际声学带宽仍受电流环和电机机械响应限制，不会注册为 Windows 音频输出设备。q 轴注入会产生转矩，必须在空载且可靠固定的台架上使用。
