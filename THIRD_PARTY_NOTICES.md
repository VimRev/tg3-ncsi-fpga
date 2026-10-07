# 第三方来源与权利声明

| 范围 | 已观察来源 | 本预览处理 |
| --- | --- | --- |
| PCILeech 基础 HDL | 文件中现有 Ulf Frisk / PCILeech FPGA 注释；[上游](https://github.com/ufrisk/pcileech-fpga) | 保留原注释，不声称完全原创，不擅自再许可 |
| `pcie_7x/*.v` | 部分文件含 Xilinx / AMD 版权、免责声明及许可条件 | 本地预览保留现有依赖；公开再分发待核查 |
| `ip/*.xci`、`ip/100t/*.xci` | Vivado IP 配置 | 保留配置作为输入；供应商 IP 使用仍受适用条款约束 |
| COE/MEM 合成初始化输入 | 本地项目中的设备初始化模型 | 不发布测试夹具或原始抓包；构建所需初始化数据仍需核对标识与来源权限 |
| DHCP/NCSI 文档 | 项目实际 RTL/测试结果，NCSI 原理参照 Microsoft | 仿真事实与 Windows 条件性预期分别标注 |

本地的 `src/pcileech_com_e.v` 与 ILA HDL 未用于当前发布入口，已从预览范围排除。没有捆绑 Vivado、第三方驱动、下载网页或供应商安装程序。

本文是组件来源清单，不替代第三方许可证全文或供应商授权。准确的许可证与可再分发范围需在正式公开发布之前补齐。
