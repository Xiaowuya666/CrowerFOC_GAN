# CrowerFOC 上位机

Windows 图形上位机使用 PyQt5，通过 USB CDC 与电机板通信。用户操作、控件状态、安全边界和当前验证限制统一见工程根目录 [快速使用指南](../docs/CrowerFOC_快速使用指南.md)；连接/烧录细节见 [在线测试与调试手册](../docs/CrowerFOC_在线测试与调试手册.md)。

## 启动

```bat
run_gui.bat
```

脚本使用本地 `HostUI\.venv` Python 环境。首次配置依赖请参见工程测试环境说明；环境目录不纳入版本控制。

## 测试

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v
```

UI文件为 `crowerfoc_gui.py`；帧定义、通信线程及逻辑模块分别见 `crowerfoc_protocol.py`、`crowerfoc_transport.py`、`crowerfoc_client.py`。CAN 电流语义和帧布局见 [MIT-layout CAN 协议](../docs/CrowerFOC_MIT_CAN协议说明.md)。
