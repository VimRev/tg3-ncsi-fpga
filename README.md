# TG3 / DHCP / NCSI FPGA Network Interface Emulation

[![Join Discord](https://img.shields.io/badge/Discord-Join%20the%20server-5865F2?logo=discord&logoColor=white)](https://discord.gg/kzJuhn2BEv)

**One shared source tree - Captain 75T / 100T - Windows connectivity-probe experiments**

> This is a public, source-only repository. Licensing and third-party redistribution rights remain under review; public visibility does not grant additional reuse or redistribution rights.
> The design emulates local network responses and NCSI status. **It does not provide real Internet forwarding.**
> All 66 protocol cases and seven module simulation suites passed. **The project owner has confirmed successful NCSI responses and a connected status in Windows.** This is owner-reported feedback from the tested environment, not a universal compatibility claim or evidence of real Internet forwarding.

## 1. Overview

This project emulates a TG3-style network adapter on FPGA hardware for Windows. It models device registers, DMA transmit and receive rings, interrupt handling, reset behavior, and power-state transitions. Local protocol responders provide DHCPv4 leases, gateway ARP, ICMP replies, and DNS answers for default Windows connectivity probes. A limited TCP and HTTP implementation answers modern and legacy NCSI requests, allowing Windows to report a connected status in the tested environment. The repository includes one shared source tree for Captain 75T and 100T targets. It is a controlled connectivity-emulation experiment, not a complete physical Ethernet adapter or a real Internet forwarding solution.

Questions and project discussion: [join the Discord server](https://discord.gg/kzJuhn2BEv).

The 75T and 100T targets share the same core RTL while retaining their own top-level modules, constraints, and project-generation scripts. Two generated Vivado projects are not bundled; generate the required target locally.

Windows NCSI (Network Connectivity Status Indicator) assesses local and Internet connectivity. It is distinct from the server network-controller management protocol NC-SI. NCSI uses active and passive probes, and its final status can also depend on system policy, routing, and proxies. See the [Microsoft NCSI overview](https://learn.microsoft.com/en-us/windows-server/networking/ncsi/ncsi-overview).

### Intended uses

- Study DHCP parameters, network-response formats, and driver receive metadata on FPGA hardware you own or are authorized to test.
- Study the register interface and DMA lifecycle, including resets, power states, and backpressure.
- Validate local responses to the default Windows IPv4 NCSI probes in an isolated lab.

### What this project is not

- A complete physical Ethernet adapter, switch, router, or recursive DNS service.
- A complete TCP/IP stack supporting arbitrary applications or concurrent clients.
- A guarantee that every Windows version and network environment will report Internet connectivity.
- A WHQL-certified adapter or an officially certified Broadcom driver implementation.

## 2. Source layout

```text
.
|-- README.md                    # Features, internal simulation summary, and Windows feedback
|-- LICENSE_STATUS.md            # Licensing status
|-- THIRD_PARTY_NOTICES.md        # Upstream and vendor rights notices
|-- src/                         # Core RTL, top-level modules, and constraints
|-- ip/                          # XCI / COE / MEM synthesis inputs
|-- pcie_7x/                     # Active PCIe core wrappers and HDL
|-- tools/                       # Required build/configuration scripts; no test scripts
|-- generate_captain_75t_project.bat
|-- generate_captain_100t_project.bat
|-- vivado_generate_project_captain_75T.tcl
`-- vivado_generate_project_captaindma_100t.tcl
```

**Only source files and required build inputs are included. Tests and verification materials are not published.**

Excluded: `tb/`, `tests/`, `docs/`, test scripts, testbenches, regression fixtures, verification JSON/logs, `pcileech_enigma_x1/`, `pcileech_100t484_x1/`, generated `.xpr/.runs/.srcs` content, synthesis/implementation caches, `.bit/.bin` firmware, waveforms, raw packet captures, drivers, editor settings, and historical backups. `ip/100t/` contains source configurations, not a second generated Vivado project.

## 3. Implemented behavior

| Component | Implementation | Important limitations |
| --- | --- | --- |
| Device identity | FPGA DNA-derived identity with consistent BAR/NVRAM and protocol-template addressing | Simulations use a fixed DNA fixture; not every physical board has been verified |
| DHCPv4 | DISCOVER/OFFER, REQUEST/ACK/NAK, renewal validation, Options 50/54/61, variable lengths, and checksums | One fixed lease; not a general-purpose multi-client server; no DHCP relay |
| VLAN | Retains a single tag and adjusts reply offsets and lengths | The 66 protocol cases cover 802.1Q VLAN 100; no QinQ verification is claimed |
| DNS | A responses for default NCSI names, AAAA NODATA, and NXDOMAIN for unknown names | No recursive resolution or external DNS access |
| ARP / ICMP | Local gateway ARP and ICMP response templates | Targeted template checks, not exhaustive network coverage |
| NCSI HTTP | Modern/legacy default Host/path pairs, TCP handshake, segmented requests, duplicate data, FIN, and rejection of incorrect Host/path pairs | IPv4, TCP port 80, one session, finite request buffer; not a general HTTP server |
| DMA / RX | RX return ring, status block, byte order, length/FCS, and checksum flags | Depends on the actual driver's expected consumption behavior; not a complete Windows driver test |
| Lifecycle | D3/D0, qualified FLR/hot reset, BME/disable handling, backpressure, and late-completion isolation | Not proof of physical sleep/resume, hot-plug, or compatibility with every motherboard |

### Default local connectivity-emulation profile

```systemverilog
parameter ADVERTISE_ROUTER_DNS = 1'b1;
parameter EMULATE_REMOTE_NCSI = 1'b1;
```

- DHCP advertises an emulated local gateway and DNS server.
- IPv4 DNS and default NCSI HTTP traffic **that reaches this interface** can be answered inside the FPGA.
- If a default probe uses a cached public destination IP, the reply source IP, checksums, and session matching use the corresponding service address.
- No such traffic is forwarded to the public Internet.

This is a limited experimental mode. Advertising a gateway and DNS server may affect host routing and name resolution; use an isolated test host or a controlled virtual environment. Parameter changes require fresh simulation and builds. The current 66/66 result does not automatically apply to modified profiles.

## 4. Completed simulations

The following results apply to the latest connected profile. Assertions, protocol cases, and register stimuli are different units and must not be added together as one coverage metric.

| Suite | Result | Checks | Expected Windows behavior, not per-module hardware verification |
| --- | --- | --- | --- |
| Independent protocol transactions | **66/66** | DHCP/DNS/TCP/NCSI, single VLAN, cached service IPs, and output checksums | Provides the response conditions needed to complete normal leasing and default probes |
| T00 BAR initialization replay | **2,272 stimuli passed** | 263 exact reads, 899 valid dynamic reads, 941 writes, and 169 configuration writes | Consistent model behavior during driver initialization; not proof of successful driver loading |
| T06 device identity | **28/28** | DNA, MAC, subnet, BAR/NVRAM, and DHCP/ICMP/DNS templates | Consistent identity across the same device's interfaces |
| T09 network templates | **11/11** | NCSI SYN-ACK/body, DNS, ARP, and mDNS/LLMNR WPAD templates | Matching default-probe content; WPAD templates do not implement a complete proxy service |
| T13 RX delivery | **27/27** | Descriptors, status block, byte order, checksum flags, and a 354-byte OFFER | Received packets and metadata are exposed according to the modeled driver contract |
| T07 D3 power gating | **20/20** | PMCSR glitch filtering, stable D3 stop, and D0 reinitialization | Avoids false stops caused by transient power-state changes; physical sleep/resume remains unverified |
| T08 reset lifecycle | **35/35** | FLR/hot-reset filtering, ring cleanup, reinitialization, and re-enable | Clears stale state after qualified resets and permits driver reinitialization |
| T14 disable and backpressure | **51/51** | End-of-frame acceptance, source locking, disable draining, and late-completion isolation | Reduces mixed frames and stale-completion contamination during disable/restart |

**Publication scope:** These are summaries of completed internal verification. Test sources, regression scripts, simulation stimuli, verification reports, and detailed test materials are not included in this repository. The unpublished internal cases cannot be rerun directly from this source-only package. Windows expectations and reported hardware results are distinguished below.

## 5. Windows behavior and reported hardware results

| Condition | Expected behavior | Hardware confirmation |
| --- | --- | --- |
| Driver loaded normally and interface enabled | A network adapter appears; link status depends on the driver/register model | Not separately confirmed |
| Windows uses DHCPv4 and packets reach this interface | Receives the fixed address in the DNA-derived subnet, plus emulated gateway and DNS settings | Not separately confirmed; protocol responses were simulated |
| NCSI requests/responses in the owner's tested environment | Successful NCSI response and connected status in Windows | **Confirmed by the project owner's hardware-test feedback** |
| Visiting ordinary websites after an Internet-status icon appears | **No real Internet forwarding is provided**; the icon alone does not demonstrate usable Internet access | A functional boundary, not a successful browsing test |
| Device disabled, driver failure, no IPv4, or probes using another interface | This source cannot force the operating system to report a connected state | Not covered |
| Enterprise-custom probes, proxies, HTTPS, or IPv6-only networks | Outside the current implementation and 66-case coverage | Not covered |

Valid active-probe responses can cause NCSI to classify Internet connectivity, but this does not replace end-to-end application connectivity testing. See the [Microsoft NCSI overview](https://learn.microsoft.com/en-us/windows-server/networking/ncsi/ncsi-overview).

## 6. Generate and build locally

Building requires the applicable Vivado/IP licenses. The verified tool version was Vivado 2024.2. Select the target for your board; there is no need to upload or generate both project directories.

```powershell
# 75T
.\generate_captain_75t_project.bat
vivado -mode batch -source tools/build_captain_75t.tcl -notrace

# Alternatively, choose 100T; building both targets is not required
.\generate_captain_100t_project.bat
vivado -mode batch -source tools/build_captain_100t.tcl -notrace
```

Generated projects, firmware, and tool logs are excluded by `.gitignore`. Test entry points are not part of the publication package. Original local test materials are retained, not deleted from the original project.

The `.coe/.mem` files under `ip/` are RTL/memory-IP initialization inputs, not testbench fixtures. Removing them would remove required build dependencies.

## 7. Implementation and timing

Before packaging, the shared RTL completed native implementation for both targets in Vivado 2024.2:

| Target | WNS (setup) | WHS (hold) | Scope |
| --- | --- | --- | --- |
| 75T | +0.474 ns | +0.016 ns | Results for the corresponding source revision, not hardware-test results |
| 100T | +0.736 ns | +0.016 ns | Results for the corresponding source revision, not hardware-test results |

The 100T target includes a 16 ns PCIe user-clock constraint. DHCP checksum accumulation was split into registered pipeline stages to avoid an excessively long combinational path. Build scripts check the user clock and negative setup/hold paths; failed builds must not be treated as deliverable firmware.

The publication copy includes portability adjustments to build comments and imported-directory names. Core RTL remains byte-identical to the verified repaired source. Tests/reports were removed only from the publication package; original local verification materials remain available. **The reduced source-only directory has not undergone a fresh full-device implementation.** These timing numbers are not guarantees for other tool versions or parameters.

## 8. Repair summary

1. Distinguish DHCPREQUEST ACK, NAK, and ignore decisions using the selected server, requested address, and renewal state.
2. Echo Option 61 when present, do not invent it when absent, and support identifiers up to 255 bytes.
3. Reject covered malformed lengths, fragmented packets, and invalid BOOTP operations; generate replies using actual option lengths.
4. Preserve the single VLAN tag in replies rather than parsing tagged input and returning an untagged frame.
5. Restrict DNS names and HTTP Host/path pairs to default NCSI probes instead of treating arbitrary traffic as successful probes.
6. Handle segmented/duplicate TCP data, FIN, and reconnection; use cached service IPs for reply addressing and session matching.
7. Update the 354-byte DHCP fixture, RX metadata checks, and lifecycle source checkers in the retained internal test materials.
8. Synchronize imported RTL/XDC, disable stale incremental checkpoints, and check timing before exporting build outputs.

## 9. Attribution, licensing, and release status

Existing copyright comments crediting PCILeech author Ulf Frisk are preserved. See [PCILeech FPGA](https://github.com/ufrisk/pcileech-fpga). Third-party components and generated HDL do not become original project code merely by inclusion in this repository.

The selected source copy does not contain a clear license covering the entire repository, and some AMD/Xilinx HDL carries separate rights and licensing notices. This repository does not assign an assumed MIT/GPL/Apache license. Confirm applicable rights and terms before reusing or redistributing third-party components. See [LICENSE_STATUS.md](LICENSE_STATUS.md), [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md), and GitHub's [repository licensing documentation](https://docs.github.com/en/repositories/managing-your-repositorys-settings-and-features/customizing-your-repository/licensing-a-repository). Public visibility alone does not make a repository open-source licensed.

The project owner reported that NCSI responses succeeded and Windows displayed a connected state. This README records that result only for the tested environment; it does not imply that both board targets, every Windows version, or every network environment passed. No screenshots, captures, or unspecified system versions have been fabricated. `VimRev/tg3-ncsi-fpga` is publicly available as a source-only repository; applicable licensing remains under review.
