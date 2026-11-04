# 第三方内容和许可

根目录 MIT 许可适用于 CrowerFOC 贡献者有权授权的原创代码，不覆盖以下内容：

- STM32Cube 生成的文件保留原有版权和许可头；ST 驱动和设备支持文件见 `Software/MotorTest_cmake/stm32c5xx_drivers/LICENSE.md`、`stm32c5xx_dfp/LICENSE.md` 及相应源码头。
- CMSIS、USBX 和系统调用实现的许可分别见 `Software/MotorTest_cmake/arch/cmsis/LICENSE.md`、`middleware/usbx/LICENSE.md` 和 `utilities/syscalls/LICENSE.md`；生成代码目录也有各组件的许可文件。
- Python 依赖由各自项目许可，包括 PyQt5 的 GPL/商业许可选项。分发应用时应自行核对所有依赖的许可义务。
- `HostUI/music/你_从第15小节.mid` 是用户提供的曲目文件，不按 CrowerFOC MIT 代码许可授权。作品和编曲的使用、分发权应向相应权利人确认。
- 元件厂家数据手册不随当前源码树分发，应从对应厂家获取。历史 Git 提交可能仍包含曾经提交的副本。
