# 20届智能车安徽基础光电组两轮平衡车

这是一个基于 STC32G12K128 和 Keil uVision 的两轮平衡智能车控制工程，面向基础光电组场景。代码包含电机控制、PID 控制、姿态角解算、循迹、数据采集和屏幕菜单等模块。

## 目录结构

- `Code/`: 用户功能模块代码，例如循迹、控制、采集、电机和界面逻辑。
- `User/`: 主程序入口和中断相关代码。
- `DAIMXA.uvproj`: Keil uVision 工程文件。

## 构建环境

- MCU: STC32G12K128
- IDE: Keil uVision
- Toolchain: MCS-251
- 工程依赖 DAIMXA 库，当前工程文件引用路径为：
  - `../DmxLibraries/DmxDevice`
  - `../DmxLibraries/DmxDriver`

克隆仓库后，需要将 DAIMXA 库放到工程上一级目录的 `DmxLibraries` 下，或按本机目录结构修改 `DAIMXA.uvproj` 中的包含路径和源文件路径。

## 开源注意事项

本工程中部分文件头部包含 DAIMXA/呆萌侠科技版权声明，相关库文件和模板代码的版权归原作者所有。发布前请确认 DAIMXA 库及模板代码的再分发许可；如果不能确认，建议只公开用户自写业务代码，并在 README 中说明外部依赖获取方式。

当前仓库未提交 Keil 编译输出、用户界面配置和本机临时文件，例如 `Objects/`、`Listings/`、`.uvgui.*`、`.uvopt` 和 `.vscode/`。
