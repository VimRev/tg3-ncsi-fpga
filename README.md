# TG3 / DHCP / NCSI FPGA 网络接口仿真

**单份共享源码 · Captain 75T / 100T · Windows 联网状态探测实验**

> 当前为 GitHub 私有预览，尚未公开；许可证及第三方再分发权限待确认。
> 本项目提供本地网络协议应答与 NCSI 状态仿真，**不提供真实 Internet 转发**。
> 66 个协议用例及 7 组模块仿真已通过；**用户已实测确认 NCSI 应答成功，Windows 已显示联网**。这是本次测试环境的用户反馈，不代表所有系统/网络环境的兼容性，也不等同于真实 Internet 转发。

## 1. 这是什么

这是基于 PCILeech FPGA 框架的 Artix-7 网络接口实验源码，包含 Broadcom TG3 风格的寄存器模型、收发描述符处理、本地 DHCP/DNS/ARP/ICMP 应答，以及 Windows 默认 NCSI HTTP 探测的有限实现。

75T 和 100T 使用同一份核心 RTL，保留各自顶层、约束与工程生成脚本。仓库不打包两套已生成工程，使用者按需要在本地生成。

Windows 的 NCSI（Network Connectivity Status Indicator）用于判断本地/Internet 连通性，不是服务器网卡管理协议 NC-SI。NCSI 有主动与被动探测，最终状态还受系统策略、路由、代理等因素影响。[Microsoft NCSI 概览](https://learn.microsoft.com/en-us/windows-server/networking/ncsi/ncsi-overview)。

### 适合做什么

- 研究自有 FPGA 网络接口的 DHCP 参数、网络应答格式和驱动接收元数据。
- 研究寄存器接口及 DMA 生命周期、复位、电源状态和背压处理。
- 在隔离实验环境中，验证 Windows 默认 IPv4 NCSI 探测的本地应答链。

### 不是什么

- 不是完整物理以太网卡、交换机、路由器或递归 DNS 服务。
- 不是完整 TCP/IP 协议栈，不支持任意应用连接或并发多客户端。
- 不保证“任何 Windows、任何网络环境都显示 Internet”。
- 不代表通过 WHQL、官方 Broadcom 驱动认证或全版本兼容性验证。

## 2. 源码布局：只有一套共享目录

```text
.
├── README.md                    # 功能、内部仿真摘要与 Windows 实测反馈
├── LICENSE_STATUS.md            # 许可状态说明
├── THIRD_PARTY_NOTICES.md        # 原作者与供应商权利声明
├── src/                         # 核心 RTL、顶层与约束
├── ip/                          # 合成需要的 XCI / COE / MEM 输入
├── pcie_7x/                     # 当前使用的 PCIe 核包装/HDL
├── tools/                       # 必要构建与配置脚本，不含测试脚本
├── generate_captain_75t_project.bat
├── generate_captain_100t_project.bat
├── vivado_generate_project_captain_75T.tcl
└── vivado_generate_project_captaindma_100t.tcl
```

**只发布源码及必要构建输入。不提交测试或验证资料。**

**不提交：** `tb/`、`tests/`、`docs/`、测试脚本、testbench、回归夹具、验证 JSON/日志、 `pcileech_enigma_x1/`、`pcileech_100t484_x1/`、`.xpr/.runs/.srcs`、综合实现缓存、固件 `.bit/.bin`、波形、原始抓包、驱动文件、编辑器配置和历史备份。`ip/100t/` 只是源配置，不是第二套 Vivado 工程。

## 3. 当前功能

| 模块 | 实现内容 | 重要限制 |
| --- | --- | --- |
| 设备身份 | FPGA DNA 派生身份，BAR/NVRAM/协议模板使用一致的地址信息 | 仿真使用固定 DNA 夹具，不代表每块实物板都完成验证 |
| DHCPv4 | DISCOVER/OFFER、REQUEST/ACK/NAK、续租校验、Option 50/54/61、可变长度与校验和 | 一个固定租约，不是通用多租户 DHCP 服务器；不处理 DHCP relay |
| VLAN | 单层标签保留，回复适配帧偏移与长度 | 66 个协议用例实际覆盖 802.1Q VLAN 100；不宣称 QinQ 验证 |
| DNS | 默认 NCSI 名称的 A 应答、AAAA NODATA、未知名称 NXDOMAIN | 不是递归解析器，不访问真实外部 DNS |
| ARP / ICMP | 本地网关 ARP 与 ICMP 模板 | 定向模板检查，不代表所有网络情况均被覆盖 |
| NCSI HTTP | 新旧默认 Host/路径、TCP 握手、分段请求、重复数据、FIN、拒绝错误 Host/路径 | IPv4 / TCP 80 / 单会话 / 请求缓冲有限；非任意 HTTP 服务 |
| DMA / RX | RX return ring、status block、字节序、长度/FCS、校验和标志 | 依赖实际驱动按预期消费；仿真不是完整 Windows 驱动测试 |
| 生命周期 | D3/D0、合格 FLR/hot-reset、BME/禁用、背压、迟到完成包隔离 | 不等于实际休眠恢复、热插拔与所有主板兼容性通过 |

### 当前默认是“本地模拟联网”配置

```systemverilog
parameter ADVERTISE_ROUTER_DNS = 1'b1;
parameter EMULATE_REMOTE_NCSI = 1'b1;
```

- DHCP 发布本地模拟网关和 DNS。
- 对**到达本接口**的 IPv4 DNS 和默认 NCSI HTTP 流量，可在 FPGA 内应答。
- 默认探测目的地址即使来自缓存的公网 IP，响应源 IP、校验和和会话匹配也使用对应服务地址。
- 没有向公网发送该流量，也没有提供真实数据转发。

这是一种受限实验模式。网关/DNS 广告可能影响主机路由及名称解析；请在隔离测试主机或受控虚拟环境使用。修改上述参数后必须重新仿真和构建，当前 66/66 结果不能自动适用于改参版本。

## 4. 做过哪些仿真

以下为最新 connected 配置的结果。断言数量、协议包数量和寄存器刺激数量属于不同计数，不能相加后宣称为同一种测试覆盖率。

| 仿真组 | 实际结果 | 检查重点 | 对应 Windows 预期（不代表模块逐项上机验证） |
| --- | --- | --- | --- |
| 独立协议事务 | **66/66** | DHCP/DNS/TCP/NCSI、单 VLAN、缓存服务 IP、输出校验和 | 正常租约和默认探测具备完成所需的应答条件 |
| T00 BAR 初始化重放 | **2272 条刺激通过** | 263 精确读、899 合法动态读、941 写、169 配置写 | 驱动初始化时读取/写入的模型行为一致；不是驱动加载成功证明 |
| T06 设备身份 | **28/28** | DNA、MAC、子网、BAR/NVRAM、DHCP/ICMP/DNS 模板 | 同一块设备的身份信息保持一致 |
| T09 网络模板 | **11/11** | NCSI SYN-ACK/正文、DNS、ARP、mDNS/LLMNR WPAD 模板 | 默认探测内容匹配；WPAD 模板不代表完整代理服务 |
| T13 RX 交付 | **27/27** | 描述符、status block、字节序、checksum flag、354B OFFER | 驱动可按约定看见收到的包及有效元数据 |
| T07 D3 电源门控 | **20/20** | PMCSR 毛刺过滤、稳定 D3 停止、D0 重装启动 | 避免短暂电源状态扰动误停；真实休眠恢复待验证 |
| T08 复位生命周期 | **35/35** | FLR/hot-reset 过滤、环清理、重装与再使能 | 合格复位后清理旧状态，并允许驱动重新初始化 |
| T14 禁用与背压 | **51/51** | 帧末接受、来源锁定、禁用排空、迟到完成隔离 | 降低禁用/重启时混帧和旧完成包污染风险 |

**发布范围说明：** 上述为已经完成的内部验证摘要。测试源、回归脚本、仿真刺激、验证报告和详细测试资料不随 GitHub 源码发布；本仓库不能直接重跑这些未提供的内部用例。Windows 对应表现与实测范围见下节。

## 5. Windows 实测反馈与预期表现

| 条件 | 预期表现 | 是否实机确认 |
| --- | --- | --- |
| 驱动已正常加载、接口启用 | 出现网络适配器，链路状态由驱动/寄存器模型决定 | 未确认 |
| Windows 使用 DHCPv4，包到达该接口 | 获得 DNA 派生子网中的固定地址、模拟网关及 DNS | 未确认；协议应答已仿真 |
| 本次测试环境的 NCSI 请求与应答 | NCSI 应答成功，Windows 显示联网 | **已确认：用户实测反馈** |
| 图标显示 Internet 后访问普通网站 | **本项目不提供实际 Internet 转发**，不能据图标判断上网成功 | 功能边界，不是上网测试结果 |
| 禁用设备、驱动异常、无 IPv4、探测走其他接口 | 此源码不能强制系统显示已连接 | 未覆盖 |
| 企业自定义探测、代理、HTTPS、IPv6-only | 不在当前实现/66 用例保障范围内 | 未覆盖 |

Microsoft 文档说明 NCSI 接收有效主动探测响应后可能认定 Internet 连通性，但这不替代真实应用的端到端联网测试。[NCSI 概览](https://learn.microsoft.com/en-us/windows-server/networking/ncsi/ncsi-overview)。

## 6. 本地生成与构建

源码构建需要适用的 Vivado/IP 许可；已验证工具版本为 Vivado 2024.2。按实际板卡二选一生成工程，不需要上传两套工程目录。

```powershell
# 75T
.\generate_captain_75t_project.bat
vivado -mode batch -source tools/build_captain_75t.tcl -notrace

# 或选择 100T，不需要同时构建两块板型
.\generate_captain_100t_project.bat
vivado -mode batch -source tools/build_captain_100t.tcl -notrace
```

生成的工程、固件与工具日志都由 `.gitignore` 排除。本次公开范围不包括测试入口，内部测试材料仍保留在本地，并没有删除原工程中的测试。

`ip/` 下的 `.coe/.mem` 为 RTL/存储器 IP 的初始化输入，不是 `tb/` 的测试夹具，不能为了去掉测试而误删这些构建依赖。

## 7. 构建与时序

共享 RTL 在打包前曾用 Vivado 2024.2 分别完成 75T / 100T 原生实现：

| 目标 | WNS（setup） | WHS（hold） | 说明 |
| --- | --- | --- | --- |
| 75T | +0.474 ns | +0.016 ns | 对应源版本的构建结果，不是上板结果 |
| 100T | +0.736 ns | +0.016 ns | 对应源版本的构建结果，不是上板结果 |

100T 已增加 PCIe user clock 16ns 约束；DHCP 校验和累加拆成寄存器流水，避免过长组合路径。构建脚本检查 user clock、负 setup/hold 路径，失败时不应交付产物。

源码发布副本仅对构建注释/导入目录名作可移植性适配；核心 RTL 与已验证修复版保持逐字节一致。移除的是发布包中的测试/报告资料，原本地验证仍保留。**源码精简目录尚未重新做完整芯片实现**，上述时序不能当作任何工具版本、任何参数的保证。

## 8. 修复摘要

1. DHCPREQUEST 按本服务器、请求地址和续租状态区分 ACK、NAK、忽略。
2. 原样回显存在的 Option 61；缺失时不凭空添加，支持最大 255 字节身份值。
3. 拒绝已覆盖的错误长度、分片与 BOOTP op；按实际选项长度生成正确报文。
4. 保留单 VLAN 回复标签，而不是只解析输入后把回复变成无标签。
5. 限定默认 NCSI DNS 名称与 HTTP Host/路径，避免所有流量都被误应答为探测成功。
6. 处理 TCP 分段、重复数据、FIN 及再连接；缓存服务 IP 参与响应源地址和会话匹配。
7. 修正/更新 DHCP 354B 固定夹具、RX 元数据检查和生命周期源码检查器。
8. 同步已导入 RTL/XDC、禁止使用旧增量检查点，并检查时序后再导出。

## 9. 来源、许可与公开发布状态

保留 PCILeech 原作者 Ulf Frisk 的现有版权注释；来源见 [PCILeech FPGA](https://github.com/ufrisk/pcileech-fpga)。第三方组件及生成 HDL 不会因为加上本 README 就成为本项目自有代码。

当前源副本没有足以覆盖整仓库的明确许可证文件，部分 AMD/Xilinx HDL 带独立权利/许可声明。因此预览不擅自写入 MIT/GPL/Apache 许可证，公开上传前需要确认权利和适用条款。详见 [LICENSE_STATUS.md](LICENSE_STATUS.md) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。GitHub 的公开仓库不自动等于具有开源授权的项目：[仓库许可说明](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository)。

用户已直接确认：“NCSI 应答成功已经验证，已经显示联网了”。本 README 据此记录本次环境的成功结果，不推定两块板型、所有 Windows 或所有网络环境均通过；没有虚构截图、抓包或未提供的系统版本。GitHub 私有预览仓库为 `VimRev/tg3-ncsi-fpga`，适用许可仍待确认；当前尚未公开。
