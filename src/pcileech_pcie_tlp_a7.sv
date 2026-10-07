//
// PCILeech FPGA.
//
// PCIe controller module - TLP handling for Artix-7.
//
// (c) Ulf Frisk, 2018-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_tlp_a7 #(
    parameter PRODUCTION = 0
)(
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    IfPCIeFifoTlp.mp_pcie   dfifo,
    
    // PCIe core receive/transmit data
    IfAXIS128.source        tlps_tx,
    IfAXIS128.sink_lite     tlps_rx,
    IfAXIS128.sink          tlps_static,
    IfShadow2Fifo.shadow    dshadow2fifo,
    input [15:0]            pcie_id,
    input                   bus_master_enable,
    input                   lifecycle_reset_req,
    input [5:0]             pl_ltssm_state,
    input                   power_state_d0,
    input                   dst64_idle,
    input                   tlp_tx_packet_done,
    input [2:0]             tlp_tx_packet_src,
    output                  data_plane_quiescent,
    input [127:0]           irq_debug,
    output                  int_enable,
    output                  msix_vaild,
    input                   msix_send_done,
    output [31:0]           msix_address,
    output [31:0]           msix_vector,
    input [31:0]            base_address_register,
    input [31:0]            base_address_register_1,
    input [31:0]            base_address_register_2,
    input [31:0]            base_address_register_3,
    input [31:0]            base_address_register_4,
    input [31:0]            base_address_register_5
    );
    
    IfAXIS128 tlps_bar_rsp();
    IfAXIS128 tlps_cfg_rsp();
    IfAXIS128 tlps_tg3_dma();
    IfAXIS128 tlps_rx_fifo();

    // 7 系列硬核 pl_ltssm_state 的 L0 编码（PG054 LTSSM 状态编码表）。
    localparam [5:0] LTSSM_L0 = 6'h16;

    // bar_en/alltlp_filter/cfgtlp_filter 来自 clk_sys 域的准静态配置，
    // 进入 clk_pcie 域前打两拍，消除多位异步翻转被采出不一致组合的窗口。
    (* ASYNC_REG = "TRUE" *) bit [2:0] cfgbits_sync1 = 0;
    (* ASYNC_REG = "TRUE" *) bit [2:0] cfgbits_sync2 = 0;
    always @ ( posedge clk_pcie ) begin
        cfgbits_sync1 <= {dshadow2fifo.bar_en, dshadow2fifo.alltlp_filter,
                          dshadow2fifo.cfgtlp_filter};
        cfgbits_sync2 <= cfgbits_sync1;
    end
    wire bar_en_sync        = cfgbits_sync2[2];
    wire alltlp_filter_sync = cfgbits_sync2[1];
    wire cfgtlp_filter_sync = cfgbits_sync2[0];
    wire [31:0] broadcom_cfg_68_value;
    wire [31:0] broadcom_cfg_6c_value;
    wire [31:0] broadcom_cfg_70_value;
    wire [63:0] broadcom_dma_tx_ring_addr;
    wire [15:0] broadcom_dma_tx_ring_size;
    wire [63:0] broadcom_dma_rx_std_ring_addr;
    wire [15:0] broadcom_dma_rx_std_ring_size;
    wire [63:0] broadcom_dma_rx_ret_ring_addr;
    wire [15:0] broadcom_dma_rx_ret_ring_size;
    wire [63:0] broadcom_dma_status_addr;
    wire [15:0] broadcom_dma_tx_prod_idx;
    wire [15:0] broadcom_dma_rx_std_prod_idx;
    wire [15:0] broadcom_dma_rx_ret_cons_idx;
    wire        broadcom_dma_hostcc_now;
    wire        broadcom_dma_link_event;
    wire        broadcom_dma_irq_request;
    wire [7:0]  broadcom_dma_status_tag;
    wire        broadcom_dma_cpl_pending;
    wire        broadcom_dma_cpl_claim;
    wire        broadcom_dma_read_lock;
    wire        broadcom_dma_reset_req;
    wire        broadcom_dma_quiescent;
    wire [703:0] broadcom_dma_debug;
    wire [447:0] broadcom_dma_debug_ext;
    wire         broadcom_dma_tx_stat_event;
    wire         broadcom_dma_rx_stat_event;
    wire [15:0]  broadcom_dma_tx_stat_bytes;
    wire [15:0]  broadcom_dma_rx_stat_bytes;
    wire [1:0]   broadcom_dma_tx_stat_class;
    wire [1:0]   broadcom_dma_rx_stat_class;
    wire         ident_valid;
    wire [31:0]  ident_mac_word_7c;
    wire [31:0]  ident_mac_word_80;
    wire [7:0]   ident_subnet_octet;
    wire [47:0]  ident_gw_mac;
    wire [7:0]   net_subnet_octet;
    wire [47:0]  net_gw_mac;

    pcileech_device_identity i_pcileech_device_identity(
        .clk             ( clk_pcie          ),
        .rst             ( rst               ),
        .identity_valid  ( ident_valid       ),
        .dna_value       (                   ),
        .nic_mac         (                   ),
        .mac_word_7c     ( ident_mac_word_7c ),
        .mac_word_80     ( ident_mac_word_80 ),
        .subnet_octet    ( ident_subnet_octet ),
        .gw_ipv4         (                   ),
        .lease_ipv4      (                   ),
        .gw_mac          ( ident_gw_mac      )
    );
    assign net_subnet_octet = ident_valid ? ident_subnet_octet : 8'h4D;
    assign net_gw_mac       = ident_valid ? ident_gw_mac : 48'h50C7BF000001;
    assign data_plane_quiescent =
        broadcom_dma_quiescent && dst64_idle && !tlps_tx.has_data;
    // Broadcom 的 0x7C/0x84 间接窗口属于网卡运行时配置，必须始终旁路捕获。
    // cfgtlp_en/cfgtlp_wren 只控制通用 shadow BRAM；控制端临时关闭 shadow
    // 写入时不能同时切断 TX/RX RCB 地址，否则 producer 已更新也不会启动 DMA。
    wire        broadcom_cfg_wr_valid =
                    tlps_rx.tvalid && tlps_rx.tuser[0] &&
                    (tlps_rx.tdata[31:25] == 7'b0100010);
    wire [9:0]  broadcom_cfg_wr_dwaddr = tlps_rx.tdata[75:66];
    wire [3:0]  broadcom_cfg_wr_be = tlps_rx.tdata[35:32];
    wire [31:0] broadcom_cfg_wr_data_raw = tlps_rx.tdata[127:96];
    // BAR0 共享状态采用主机字节序，和配置 shadow 的写入解码保持一致。
    wire [31:0] broadcom_cfg_wr_data = {
                    broadcom_cfg_wr_data_raw[7:0],
                    broadcom_cfg_wr_data_raw[15:8],
                    broadcom_cfg_wr_data_raw[23:16],
                    broadcom_cfg_wr_data_raw[31:24]
                };

    // 内部Broadcom DMA避开原始PCILeech通道常用的0x1F；Extended Tag未启用，保持在0x00--0x1F范围内。
    localparam [7:0] BROADCOM_DMA_TAG = 8'h1E;

    // 原始DMA软件和内部网卡共用同一个Requester ID及5位Tag空间。
    // 记录外部Tag 0x1F的生命周期；内部MRd只在该Tag空闲时启动。
    reg  external_dma_tag31_busy;
    reg [23:0] external_dma_tag31_timeout;
    wire external_mrd31_sent =
        // src_fifo 的 tready 是提前一拍的读授权，真正发送拍以 tvalid 表示。
        tlps_rx_fifo.tvalid &&
        tlps_rx_fifo.tuser[0] &&
        ((tlps_rx_fifo.tdata[31:25] == 7'b0000000) ||
         (tlps_rx_fifo.tdata[31:25] == 7'b0010000)) &&
        (tlps_rx_fifo.tdata[47:40] == BROADCOM_DMA_TAG);
    wire external_cpl31_first =
        tlps_rx.tvalid && tlps_rx.tuser[0] &&
        ((tlps_rx.tdata[31:25] == 7'b0000101) ||
         (tlps_rx.tdata[31:25] == 7'b0100101)) &&
        (tlps_rx.tdata[95:80] == {pcie_id[7:0], pcie_id[15:8]}) &&
        (tlps_rx.tdata[79:72] == BROADCOM_DMA_TAG);
    wire external_cpl31_has_data =
        (tlps_rx.tdata[31:25] == 7'b0100101);
    wire [12:0] external_cpl31_payload_bytes =
        {1'b0, tlps_rx.tdata[9:0], 2'b00} -
        {11'h000, tlps_rx.tdata[65:64]};
    wire external_cpl31_last = external_cpl31_first &&
        (!external_cpl31_has_data ||
         ({1'b0, tlps_rx.tdata[43:32]} <=
          external_cpl31_payload_bytes));
    wire external_dma_tag31_unavailable =
        external_dma_tag31_busy || external_mrd31_sent;

    always @(posedge clk_pcie) begin
        if (rst) begin
            external_dma_tag31_busy <= 1'b0;
            external_dma_tag31_timeout <= 0;
        end
        else if (external_mrd31_sent) begin
            // 同拍完成旧请求并发出新请求时，新请求仍拥有Tag。
            external_dma_tag31_busy <= 1'b1;
            external_dma_tag31_timeout <= 0;
        end
        else if (external_cpl31_last) begin
            external_dma_tag31_busy <= 1'b0;
            external_dma_tag31_timeout <= 0;
        end
        else if (external_dma_tag31_busy) begin
            if (&external_dma_tag31_timeout) begin
                // 外部请求异常丢失时允许内部数据面最终自恢复。
                external_dma_tag31_busy <= 1'b0;
                external_dma_tag31_timeout <= 0;
            end
            else begin
                external_dma_tag31_timeout <=
                    external_dma_tag31_timeout + 1'b1;
            end
        end
        else begin
            external_dma_tag31_timeout <= 0;
        end
    end
    
    // ------------------------------------------------------------------------
    // Convert received TLPs from PCIe core and transmit onwards:
    // ------------------------------------------------------------------------
    IfAXIS128 tlps_filtered();
    
    pcileech_tlps128_bar_controller #(
        .PRODUCTION                 ( PRODUCTION                )
    ) i_pcileech_tlps128_bar_controller(
        .rst                        ( rst                       ),
        .clk                        ( clk_pcie                  ),
        .bar_en                     ( bar_en_sync               ),
        .pcie_id                    ( pcie_id                   ),
        .tlps_in                    ( tlps_rx                   ),
        .tlps_out                   ( tlps_bar_rsp.source       ),
        .lifecycle_reset_req        ( lifecycle_reset_req       ),
        .bus_master_enable          ( bus_master_enable         ),
        .power_state_d0             ( power_state_d0            ),
        .broadcom_dma_reset_req     ( broadcom_dma_reset_req    ),
        .broadcom_dma_quiescent     ( broadcom_dma_quiescent    ),
        .int_enable                 ( int_enable                ),
        .msix_vaild                 ( msix_vaild                ),
        .msix_address               ( msix_address              ),
        .msix_vector                ( msix_vector               ),
        .msix_send_done             ( msix_send_done            ),
        .broadcom_cfg_68_value      ( broadcom_cfg_68_value     ),
        .broadcom_cfg_6c_value      ( broadcom_cfg_6c_value     ),
        .broadcom_cfg_70_value      ( broadcom_cfg_70_value     ),
        .broadcom_dma_tx_ring_addr  ( broadcom_dma_tx_ring_addr ),
        .broadcom_dma_tx_ring_size  ( broadcom_dma_tx_ring_size ),
        .broadcom_dma_rx_std_ring_addr ( broadcom_dma_rx_std_ring_addr ),
        .broadcom_dma_rx_std_ring_size ( broadcom_dma_rx_std_ring_size ),
        .broadcom_dma_rx_ret_ring_addr ( broadcom_dma_rx_ret_ring_addr ),
        .broadcom_dma_rx_ret_ring_size ( broadcom_dma_rx_ret_ring_size ),
        .broadcom_dma_status_addr   ( broadcom_dma_status_addr  ),
        .broadcom_dma_tx_prod_idx   ( broadcom_dma_tx_prod_idx  ),
        .broadcom_dma_rx_std_prod_idx ( broadcom_dma_rx_std_prod_idx ),
        .broadcom_dma_rx_ret_cons_idx ( broadcom_dma_rx_ret_cons_idx ),
        .broadcom_dma_hostcc_now    ( broadcom_dma_hostcc_now   ),
        .broadcom_dma_link_event    ( broadcom_dma_link_event   ),
        .broadcom_dma_irq_request   ( broadcom_dma_irq_request  ),
        .broadcom_dma_status_tag    ( broadcom_dma_status_tag   ),
        .broadcom_dma_debug         ( broadcom_dma_debug        ),
        .broadcom_dma_debug_ext     ( broadcom_dma_debug_ext    ),
        .broadcom_dma_tx_stat_event ( broadcom_dma_tx_stat_event ),
        .broadcom_dma_rx_stat_event ( broadcom_dma_rx_stat_event ),
        .broadcom_dma_tx_stat_bytes ( broadcom_dma_tx_stat_bytes ),
        .broadcom_dma_rx_stat_bytes ( broadcom_dma_rx_stat_bytes ),
        .broadcom_dma_tx_stat_class ( broadcom_dma_tx_stat_class ),
        .broadcom_dma_rx_stat_class ( broadcom_dma_rx_stat_class ),
        .broadcom_irq_debug         ( irq_debug                  ),
        .broadcom_cfg_wr_valid      ( broadcom_cfg_wr_valid     ),
        .broadcom_cfg_wr_dwaddr     ( broadcom_cfg_wr_dwaddr    ),
        .broadcom_cfg_wr_be         ( broadcom_cfg_wr_be        ),
        .broadcom_cfg_wr_data       ( broadcom_cfg_wr_data      ),
        .base_address_register      ( base_address_register     ),  // bar0 base register 基地址
        .base_address_register_1    ( base_address_register_1   ),  // bar1 base register 基地址
        .base_address_register_2    ( base_address_register_2   ),  // bar2 base register 基地址
        .base_address_register_3    ( base_address_register_3   ),  // bar3 base register 基地址
        .base_address_register_4    ( base_address_register_4   ),  // bar4 base register 基地址
        .base_address_register_5    ( base_address_register_5   ),  // bar5 base register 基地址
        .nvram_mac_override_valid   ( ident_valid               ),
        .nvram_mac_word_7c          ( ident_mac_word_7c         ),
        .nvram_mac_word_80          ( ident_mac_word_80         )
    );

    // 驱动写 HOSTCC_MODE.NOW 后先写回有效状态块，再发出第一个中断。
    // 私有 0x3FF8 脉冲仍保留，供不重新加载驱动时手动重新武装。
    pcileech_tg3_dma #(
        .PRODUCTION             ( PRODUCTION                    )
    ) i_pcileech_tg3_dma(
        .rst                    ( rst                           ),
        .clk                    ( clk_pcie                      ),
        .pcie_id                ( pcie_id                       ),
        .bus_master_enable      ( bus_master_enable             ),
        .tx_packet_done         ( tlp_tx_packet_done            ),
        .tx_packet_src          ( tlp_tx_packet_src             ),
        .dma_tx_idle            ( dst64_idle                    ),
        .tx_ring_addr           ( broadcom_dma_tx_ring_addr     ),
        .tx_ring_size           ( broadcom_dma_tx_ring_size     ),
        .rx_std_ring_addr       ( broadcom_dma_rx_std_ring_addr ),
        .rx_std_ring_size       ( broadcom_dma_rx_std_ring_size ),
        .rx_ret_ring_addr       ( broadcom_dma_rx_ret_ring_addr ),
        .rx_ret_ring_size       ( broadcom_dma_rx_ret_ring_size ),
        .status_addr            ( broadcom_dma_status_addr      ),
        .tx_prod_idx            ( broadcom_dma_tx_prod_idx      ),
        .rx_std_prod_idx        ( broadcom_dma_rx_std_prod_idx  ),
        .rx_ret_cons_idx        ( broadcom_dma_rx_ret_cons_idx  ),
        .hostcc_now             ( broadcom_dma_hostcc_now       ),
        .link_event             ( broadcom_dma_link_event       ),
        .lifecycle_reset_req    ( broadcom_dma_reset_req        ),
        .link_in_l0             ( pl_ltssm_state == LTSSM_L0    ),
        .power_state_d0         ( power_state_d0                ),
        .external_dma_tag_busy  ( external_dma_tag31_unavailable ),
        .tlps_cpl_in            ( tlps_rx                       ),
        .tlps_out               ( tlps_tg3_dma.source           ),
        .irq_request            ( broadcom_dma_irq_request      ),
        .status_tag_value       ( broadcom_dma_status_tag       ),
        .completion_pending     ( broadcom_dma_cpl_pending      ),
        .completion_claim       ( broadcom_dma_cpl_claim        ),
        .read_channel_lock      ( broadcom_dma_read_lock        ),
        .dma_quiescent          ( broadcom_dma_quiescent        ),
        .debug_snapshot         ( broadcom_dma_debug            ),
        .debug_ext              ( broadcom_dma_debug_ext        ),
        .tx_stat_event          ( broadcom_dma_tx_stat_event    ),
        .rx_stat_event          ( broadcom_dma_rx_stat_event    ),
        .tx_stat_bytes          ( broadcom_dma_tx_stat_bytes    ),
        .rx_stat_bytes          ( broadcom_dma_rx_stat_bytes    ),
        .tx_stat_class          ( broadcom_dma_tx_stat_class    ),
        .rx_stat_class          ( broadcom_dma_rx_stat_class    ),
        .subnet_octet           ( net_subnet_octet              ),
        .gw_mac                 ( net_gw_mac                    )
    );
    
    pcileech_tlps128_cfgspace_shadow i_pcileech_tlps128_cfgspace_shadow(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .clk_sys        ( clk_sys                       ),
        .tlps_in        ( tlps_rx                       ),
        .pcie_id        ( pcie_id                       ),
        .broadcom_cfg_68_value ( broadcom_cfg_68_value          ),
        .broadcom_cfg_6c_value ( broadcom_cfg_6c_value          ),
        .broadcom_cfg_70_value ( broadcom_cfg_70_value          ),
        .dshadow2fifo   ( dshadow2fifo                  ),
        .tlps_cfg_rsp   ( tlps_cfg_rsp.source           )
    );
    
    pcileech_tlps128_filter i_pcileech_tlps128_filter(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .alltlp_filter  ( alltlp_filter_sync          ),
        .cfgtlp_filter  ( cfgtlp_filter_sync          ),
        .drop_tlp       ( broadcom_dma_cpl_claim        ),
        .tlps_in        ( tlps_rx                       ),
        .tlps_out       ( tlps_filtered.source_lite     )
    );
    
    pcileech_tlps128_dst_fifo i_pcileech_tlps128_dst_fifo(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .clk_sys        ( clk_sys                       ),
        .tlps_in        ( tlps_filtered.sink_lite       ),
        .dfifo          ( dfifo                         )
    );
    
    // ------------------------------------------------------------------------
    // TX data received from FIFO
    // ------------------------------------------------------------------------
    pcileech_tlps128_src_fifo i_pcileech_tlps128_src_fifo(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .clk_sys        ( clk_sys                       ),
        .dfifo_tx_data  ( dfifo.tx_data                 ),
        .dfifo_tx_last  ( dfifo.tx_last                 ),
        .dfifo_tx_valid ( dfifo.tx_valid                ),
        .tlps_out       ( tlps_rx_fifo.source           )
    );
    
    pcileech_tlps128_sink_mux1 i_pcileech_tlps128_sink_mux1(
        .rst            ( rst                           ),
        .clk_pcie       ( clk_pcie                      ),
        .tlps_out       ( tlps_tx                       ),
        .tlps_in1       ( tlps_cfg_rsp.sink             ),
        .tlps_in2       ( tlps_bar_rsp.sink             ),
        .tlps_in3       ( tlps_rx_fifo.sink             ),
        .tlps_in4       ( tlps_tg3_dma.sink             ),
        .hold_in3       ( broadcom_dma_read_lock        )
    );

    // 当前框架的 static TLP 发生器固定关闭，槽位供手动数据面使用。
    assign tlps_static.tready = 1'b0;

endmodule


// ------------------------------------------------------------------------
// Broadcom tg3 最小离线数据面。
//
// 功能边界：
// - 单个固定 Tag、单 Outstanding 的 64 位 MRd；
// - 128 字节以内分段的 64 位 MWr；
// - 消费 TX 描述符并完成 TX consumer；
// - 向标准 RX 缓冲区注入 DHCP OFFER/ACK 和 ARP Reply；
// - 最后写回 tg3 状态块，再由现有 MSI/INTx 控制器发中断。
//
// 本模块不连接真实网络，模拟 192.168.{subnet_octet}.0/24：
// 服务器 192.168.x.1，客户端租约 192.168.x.2。x 由 Device DNA 派生。
// ------------------------------------------------------------------------
module pcileech_tg3_dma #(
    parameter PRODUCTION = 0,
    // Connected-display emulation: probes are answered locally, not forwarded to WAN.
    parameter ADVERTISE_ROUTER_DNS = 1'b1,
    parameter EMULATE_REMOTE_NCSI = 1'b1
)(
    input                   rst,
    input                   clk,
    input [15:0]            pcie_id,
    input                   bus_master_enable,
    input                   tx_packet_done,
    input [2:0]             tx_packet_src,
    input                   dma_tx_idle,

    input [63:0]            tx_ring_addr,
    input [15:0]            tx_ring_size,
    input [63:0]            rx_std_ring_addr,
    input [15:0]            rx_std_ring_size,
    input [63:0]            rx_ret_ring_addr,
    input [15:0]            rx_ret_ring_size,
    input [63:0]            status_addr,
    input [15:0]            tx_prod_idx,
    input [15:0]            rx_std_prod_idx,
    input [15:0]            rx_ret_cons_idx,
    input                   hostcc_now,
    input                   link_event,
    input                   lifecycle_reset_req,
    input                   link_in_l0,
    input                   power_state_d0,
    input                   external_dma_tag_busy,

    IfAXIS128.sink_lite     tlps_cpl_in,
    IfAXIS128.source        tlps_out,
    output bit              irq_request,
    output wire [7:0]       status_tag_value,
    output wire             completion_pending,
    output wire             completion_claim,
    output wire             read_channel_lock,
    output wire             dma_quiescent,
    output wire [703:0]     debug_snapshot,
    output wire [447:0]     debug_ext,
    output bit              tx_stat_event,
    output bit              rx_stat_event,
    output bit [15:0]       tx_stat_bytes,
    output bit [15:0]       rx_stat_bytes,
    output bit [1:0]        tx_stat_class,
    output bit [1:0]        rx_stat_class,
    input [7:0]             subnet_octet,
    input [47:0]            gw_mac
);

    wire [31:0] gw_ipv4    = {16'hC0A8, subnet_octet, 8'h01};
    wire [31:0] lease_ipv4 = {16'hC0A8, subnet_octet, 8'h02};
    wire [31:0] bcast_ipv4 = {16'hC0A8, subnet_octet, 8'hFF};

    // Device Control 未启用 Extended Tag，因此固定Tag必须位于0x00--0x1F。
    localparam [7:0] DMA_TAG = 8'h1E;

    localparam [1:0] RD_TX_DESC = 2'd0;
    localparam [1:0] RD_TX_DATA = 2'd1;
    localparam [1:0] RD_RX_DESC = 2'd2;

    localparam [1:0] WR_REPLY   = 2'd0;
    localparam [1:0] WR_RX_DESC = 2'd1;
    localparam [1:0] WR_STATUS  = 2'd2;

    localparam [3:0] REPLY_NONE       = 4'd0;
    localparam [3:0] REPLY_DHCP_OFFER = 4'd1;
    localparam [3:0] REPLY_DHCP_ACK   = 4'd2;
    localparam [3:0] REPLY_ARP        = 4'd3;
    localparam [3:0] REPLY_ICMP       = 4'd4;
    localparam [3:0] REPLY_DNS        = 4'd5;
    localparam [3:0] REPLY_BG_ARP     = 4'd6;
    localparam [3:0] REPLY_HTTP       = 4'd7;
    localparam [3:0] REPLY_TCP_SYNACK = 4'd8;
    localparam [3:0] REPLY_BG_MDNS    = 4'd9;
    localparam [3:0] REPLY_BG_LLMNR   = 4'd10;
    localparam [3:0] REPLY_TCP_ACK    = 4'd11;
    localparam [3:0] REPLY_TCP_RST    = 4'd12;
    localparam [3:0] REPLY_DHCP_NAK   = 4'd13;
    localparam [3:0] REPLY_TCP_FIN    = 4'd14;
    // 75T 基线 LUT 57% / BRAM 66%，TCP/HTTP 走 LUT 不占 BRAM，直接开 NCSI。
    localparam       ENABLE_TCP_HTTP  = 1'b1;
    localparam       ENABLE_BG_MDNS_LLMNR = 1'b1;
    // Windows SuspectDnsProbe：dns.msftncsi.com 必须含 131.107.255.255。
    // 该特判优先于万能解析；www.msftconnecttest.com 仍回网关，HTTP 已闭环。
    localparam [31:0] NCSI_DNS_A0 = 32'h836BFFFF;
    localparam [31:0] NCSI_DNS_A1 = 32'h0D6BCE22;
    localparam [31:0] NCSI_DNS_A2 = 32'h96AB0A22;
    localparam [31:0] NCSI_DNS_A3 = 32'h96AB1022;
    localparam [31:0] NCSI_DNS_A4 = 32'h0D6BDE22;
    // The x1 PCIe user clock is 62.5 MHz. Jitter produces roughly 50--183 ms gaps.
    localparam       BG_ENABLE = 1'b1;
    localparam [23:0] BG_RATE_MIN_CYCLES = 24'd3_125_000;
    localparam [15:0] BG_LFSR_SEED = 16'h1ACE;
    localparam integer ICMP_CAPTURE_BYTES = 128;
    // 覆盖完整标准/VLAN 以太帧，并给驱动填充留余量；512 字节上限会吞掉
    // 普通大帧但不计数，导致 Windows 的 NDIS 字节/速率统计和状态块脱节。
    localparam [16:0] MAX_TX_FRAME_BYTES = 17'd2048;
    localparam [31:0] TCP_INITIAL_SEQUENCE = 32'h5A00_0001;

    // tg3 的 TX consumer 和 RX return/std 环是独立资源；RX 配置缺口不能阻塞
    // TX 描述符读取和发送完成状态，否则驱动只会看到 producer 前进但 consumer 不动。
    // D3 等非 D0 电源状态视同未使能：不发起 DMA、不接受武装，杜绝向已被
    // 电源管理的内存区域读写。
    wire tx_config_ready =
        bus_master_enable && power_state_d0 &&
        (tx_ring_addr != 0) && (tx_ring_addr[1:0] == 0) &&
        (tx_ring_size >= 2) && (tx_prod_idx < tx_ring_size) &&
        (status_addr != 0) && (status_addr[1:0] == 0);

    wire rx_config_ready =
        (rx_std_ring_addr != 0) && (rx_std_ring_addr[1:0] == 0) &&
        (rx_std_ring_size >= 2) &&
        (rx_std_prod_idx < rx_std_ring_size) &&
        (rx_ret_ring_addr != 0) && (rx_ret_ring_addr[1:0] == 0) &&
        (rx_ret_ring_size >= 2) &&
        (rx_ret_cons_idx < rx_ret_ring_size);

    wire dma_config_ready = tx_config_ready && rx_config_ready;

    // 组合地址检查只能说明“当前值看起来完整”，不能证明驱动已经结束本轮初始化。
    // 以 HOSTCC.NOW 作为代际门闩，避免冷启动或暖复位期间用新旧 RCB 混合值发起 DMA。
    reg dma_runtime_armed;
    reg hostcc_pending;
    // 会话结束沿检测（BME/D0 1→0）：跨会话的挂起 NOW 只在此时丢弃，
    // 会话内 tx_config_ready 未就绪的窗口里必须保留（否则先写 NOW 后配
    // ring 的正常初始化顺序会把武装请求丢掉，数据面永远无法武装）。
    reg bme_prev, d0_prev;
    wire dma_session_end = (bme_prev && !bus_master_enable) ||
                           (d0_prev && !power_state_d0);
    wire dma_runtime_ready = tx_config_ready && dma_runtime_armed;

    localparam [7:0] S_WAIT_CONFIG       = 8'd0;
    localparam [7:0] S_IDLE              = 8'd1;
    localparam [7:0] S_MRD_QUEUE         = 8'd2;
    localparam [7:0] S_MRD_WAIT_SENT     = 8'd3;
    localparam [7:0] S_MRD_WAIT_CPL      = 8'd4;
    localparam [7:0] S_TX_DESC_READY     = 8'd5;
    localparam [7:0] S_TX_DATA_PREP      = 8'd6;
    localparam [7:0] S_TX_DATA_CHUNK_DONE= 8'd7;
    localparam [7:0] S_TX_SEG_PROGRESS   = 8'd8;
    localparam [7:0] S_PARSE_CLASSIFY    = 8'd9;
    localparam [7:0] S_DHCP_OPTIONS      = 8'd10;
    localparam [7:0] S_WAIT_RX_BUFFER    = 8'd11;
    localparam [7:0] S_RX_DESC_READY     = 8'd12;
    localparam [7:0] S_MWR_PREP          = 8'd13;
    localparam [7:0] S_MWR_HDR_WAIT      = 8'd14;
    localparam [7:0] S_MWR_DATA_QUEUE    = 8'd15;
    localparam [7:0] S_MWR_DATA_WAIT     = 8'd16;
    localparam [7:0] S_MWR_CHUNK_DONE    = 8'd17;
    localparam [7:0] S_REPLY_WRITTEN     = 8'd18;
    localparam [7:0] S_RX_DESC_WRITE     = 8'd19;
    localparam [7:0] S_RX_DESC_WRITTEN   = 8'd20;
    localparam [7:0] S_STATUS_PREP       = 8'd21;
    localparam [7:0] S_STATUS_WRITE      = 8'd22;
    localparam [7:0] S_STATUS_WRITTEN    = 8'd23;
    localparam [7:0] S_IRQ               = 8'd24;
    localparam [7:0] S_TX_DESC_REQUEST   = 8'd25;
    localparam [7:0] S_RX_DESC_REQUEST   = 8'd26;
    localparam [7:0] S_TX_DROP_PROGRESS  = 8'd27;
    localparam [7:0] S_MRD_WAIT_TX_DONE  = 8'd28;
    localparam [7:0] S_MWR_WAIT_TX_DONE  = 8'd29;
    localparam [7:0] S_TX_CHAIN_WAIT     = 8'd30;
    localparam [7:0] S_TX_DATA_DRAIN     = 8'd31;
    localparam [7:0] S_DHCP_CSUM_INIT    = 8'd32;
    localparam [7:0] S_DHCP_CSUM         = 8'd33;
    localparam [7:0] S_TCP_REQUEST_SCAN  = 8'd34;
    localparam [7:0] S_DHCP_CSUM_BYTE    = 8'd35;
    localparam [7:0] S_DHCP_CSUM_FOLD    = 8'd36;

    function automatic [31:0] byte_swap32(input [31:0] value);
        begin
            byte_swap32 = {
                value[7:0], value[15:8], value[23:16], value[31:24]
            };
        end
    endfunction

    function automatic [3:0] first_be_for_range(
        input [1:0]  byte_offset,
        input [15:0] byte_count
    );
        integer lane;
        begin
            first_be_for_range = 4'h0;
            for (lane = 0; lane < 4; lane = lane + 1)
                if ((lane >= byte_offset) &&
                    ((lane - byte_offset) < byte_count))
                    first_be_for_range[lane] = 1'b1;
        end
    endfunction

    function automatic [3:0] last_be_for_range(
        input [1:0]  byte_offset,
        input [15:0] byte_count
    );
        reg [17:0] span_bytes;
        begin
            span_bytes = {16'h0000, byte_offset} + byte_count;
            if (span_bytes <= 4)
                last_be_for_range = 4'h0;
            else begin
                case (span_bytes[1:0])
                    2'd1: last_be_for_range = 4'b0001;
                    2'd2: last_be_for_range = 4'b0011;
                    2'd3: last_be_for_range = 4'b0111;
                    default: last_be_for_range = 4'b1111;
                endcase
            end
        end
    endfunction

    function automatic [15:0] ring_next(
        input [15:0] index_value,
        input [15:0] ring_count
    );
        begin
            if ((ring_count < 2) || ((index_value + 1'b1) >= ring_count))
                ring_next = 16'h0000;
            else
                ring_next = index_value + 1'b1;
        end
    endfunction

    reg [31:0] desc_words [0:7];
    reg [31:0] tx_data_words [0:31];

    // 只锁存分类和诊断真正使用的字节，避免多读口 LUTRAM 在综合后
    // 复制出大量 RAM32M，且避免其写入/读出语义与 RTL 仿真不一致。
    reg [15:0] frame_ethertype;
    reg [7:0]  frame_ip_protocol;
    reg [15:0] frame_arp_opcode;
    reg [47:0] frame_arp_sender_mac;
    reg [31:0] frame_arp_sender_ip;
    reg [31:0] frame_arp_target_ip;
    reg [31:0] frame_udp_ports;
    reg [31:0] frame_dhcp_xid;
    reg [15:0] frame_dhcp_flags;
    reg [47:0] frame_dhcp_chaddr;
    reg [31:0] frame_dhcp_cookie;
    reg [23:0] frame_dhcp_message_option;
    reg [47:0] frame_dst_mac;
    reg [15:0] frame_outer_ethertype;
    reg        frame_vlan_tagged;
    reg        frame_ipv4_header_valid;
    reg [10:0] frame_l3_offset;
    reg [10:0] frame_l4_offset;
    reg [10:0] frame_bootp_offset;
    reg [10:0] frame_dhcp_options_offset;
    reg [7:0]  frame_dhcp_message_type;
    reg        frame_dhcp_message_type_valid;
    reg [31:0] frame_dhcp_requested_ip;
    reg [31:0] frame_dhcp_server_id;
    reg        frame_dhcp_requested_ip_valid;
    reg        frame_dhcp_server_id_valid;
    reg [15:0] frame_vlan_tci;
    reg [15:0] frame_vlan_tpid;
    reg [7:0] frame_bootp_op, frame_bootp_htype, frame_bootp_hlen;
    reg [31:0] frame_bootp_ciaddr, frame_bootp_giaddr;
    reg frame_dhcp_malformed, frame_dhcp_end_seen;
    reg frame_dhcp_client_id_valid;
    reg [7:0] frame_dhcp_client_id_length;
    (* ram_style = "distributed" *) reg [7:0] dhcp_client_id_bytes [0:254];
    reg frame_dns_web_ok, frame_dns_legacy_ok;
    reg frame_dns_web, frame_dns_legacy;
    reg [47:0] frame_src_mac;
    reg [31:0] frame_ipv4_src;
    reg [31:0] frame_ipv4_dst;
    reg [7:0]  frame_ipv4_tos;
    reg [15:0] frame_ipv4_total_length;
    reg [15:0] frame_ipv4_identification;
    reg [15:0] frame_ipv4_flags_fragment;
    reg [15:0] frame_udp_length;
    reg [15:0] frame_dns_flags;
    reg [15:0] frame_dns_qdcount;
    reg [15:0] frame_dns_qtype;
    reg [15:0] frame_dns_qclass;
    reg [10:0] frame_dns_question_end;
    reg        frame_dns_qname_done;
    reg        frame_dns_ncsi_ok;
    reg        frame_dns_ncsi;
    reg [31:0] frame_tcp_seq;
    reg [31:0] frame_tcp_ack;
    reg [3:0]  frame_tcp_data_offset;
    reg [7:0]  frame_tcp_flags;
    reg [7:0]  frame_icmp_type;
    reg [7:0]  frame_icmp_code;
    reg [23:0] frame_icmp_even_byte_sum;
    reg [23:0] frame_icmp_odd_byte_sum;
    reg [1:0]  dhcp_scan_state;
    reg [7:0]  dhcp_scan_code;
    reg [7:0]  dhcp_scan_length;
    reg [7:0]  dhcp_scan_remaining;
    reg [7:0]  dhcp_scan_index;

    localparam [1:0] DHCP_SCAN_CODE  = 2'd0;
    localparam [1:0] DHCP_SCAN_LEN   = 2'd1;
    localparam [1:0] DHCP_SCAN_VALUE = 2'd2;
    localparam [1:0] DHCP_SCAN_END   = 2'd3;

    localparam [1:0] STAT_UNICAST   = 2'd0;
    localparam [1:0] STAT_MULTICAST = 2'd1;
    localparam [1:0] STAT_BROADCAST = 2'd2;

    reg [15:0] dbg_tx_desc_count;
    reg [15:0] dbg_tx_end_count;
    reg [7:0]  dbg_classified_count;
    reg [7:0]  dbg_dhcp_discover_count;
    reg [7:0]  dbg_dhcp_request_count;
    reg [7:0]  dbg_arp_count;
    reg [15:0] dbg_rx_accept_count;
    reg [15:0] dbg_rx_reject_count;
    reg [15:0] dbg_reply_commit_count;
    reg [15:0] dbg_status_commit_count;
    reg [7:0]  dbg_last_drop_reason;
    reg [1:0]  dbg_last_tx_addr_low;
    reg [1:0]  dbg_last_rx_addr_low;
    reg [7:0]  dbg_packet_bytes [0:55];
    reg [15:0] dbg_packet_length;
    reg [15:0] dbg_packet_flags;
    reg [15:0] dbg_packet_tx_cons_idx;
    reg [7:0]  dbg_packet_capture_count;
    reg        dbg_packet_valid;
    reg        dbg_packet_candidate_frozen;
    reg        dbg_packet_overflow;
    reg        dbg_offer_pending;
    reg        dbg_offer_frozen;
    reg        dbg_offer_bytes_done;
    reg        dbg_offer_irq;
    reg [63:0] dbg_offer_addr;
    reg [31:0] dbg_offer_ret_addr;
    reg [31:0] dbg_offer_opaque;
    reg [31:0] dbg_offer_idx_len;
    reg [15:0] dbg_offer_prod;
    reg [7:0]  dbg_offer_tag;
    reg [15:0] dbg_hostcc_accept_count;
    reg [15:0] dbg_hostcc_deferred_count;
    reg [15:0] dbg_mrd_tx_timeout_count;
    reg [15:0] dbg_mwr_tx_timeout_count;
    reg [15:0] dbg_cpl_seen_count;
    reg [15:0] dbg_cpl_match_count;
    reg [15:0] dbg_cpl_reqid_mismatch_count;
    reg [15:0] dbg_cpl_tag_mismatch_count;
    reg [15:0] dbg_cpl_status_error_count;
    reg [15:0] dbg_cpl_nodata_error_count;
    reg [15:0] dbg_cpl_timeout_count;
    reg [15:0] dbg_cpl_malformed_count;
    reg [7:0]  dbg_last_cpl_tag;
    reg [2:0]  dbg_last_cpl_status;
    reg [15:0] dbg_tx_packet_total;
    reg [15:0] dbg_rx_packet_total;
    integer    dbg_packet_reset_index;

    reg [47:0] client_mac;
    reg [31:0] client_ipv4;
    reg [31:0] dhcp_xid;
    reg [15:0] dhcp_flags;
    reg [3:0]  reply_type;
    reg [15:0] reply_frame_length;
    reg [9:0]  icmp_l3_offset;
    reg [9:0]  icmp_l4_offset;
    reg [15:0] icmp_reply_ip_checksum;
    reg [15:0] icmp_reply_message_checksum;
    reg [15:0] icmp_request_length;
    // 单写异步读缓存强制使用 LUTRAM，避免展开成 1024 位触发器和多级读选择器。
    (* ram_style = "distributed" *)
    reg [7:0]  icmp_request_bytes [0:ICMP_CAPTURE_BYTES-1];
    reg [9:0]  dns_question_end;
    reg        dns_ncsi;
    reg [15:0] reply_ipv4_identification;
    reg reply_vlan;
    reg [15:0] reply_vlan_tci, reply_vlan_tpid;
    wire [15:0] reply_vlan_bytes = reply_vlan ? 16'd4 : 16'd0;
    wire [15:0] reply_core_length = reply_frame_length - reply_vlan_bytes;
    reg [31:0] service_ipv4;
    reg [31:0] tcp_peer_service_ipv4;
    reg [3:0] dns_answer_count;
    reg [3:0] dns_rcode;
    reg [31:0] dhcp_ciaddr;
    reg dhcp_client_id_valid;
    reg [7:0] dhcp_client_id_length;
    reg [15:0] dhcp_reply_udp_checksum;
    reg [31:0] dhcp_checksum_sum;
    reg [9:0] dhcp_checksum_index;
    reg dhcp_checksum_running;
    reg [7:0] dhcp_checksum_byte_q;
    localparam integer TCP_REQUEST_MAX_BYTES = 512;
    (* ram_style = "distributed" *) reg [7:0] tcp_frame_bytes[0:TCP_REQUEST_MAX_BYTES-1];
    reg tcp_session_active, tcp_session_established, tcp_fin_sent, tcp_client_fin_seen;
    reg [47:0] tcp_peer_mac;
    reg [31:0] tcp_peer_ip, tcp_client_next_seq, tcp_server_next_seq;
    reg [15:0] tcp_peer_port;
    reg tcp_peer_vlan;
    reg [15:0] tcp_peer_tci, tcp_peer_tpid;
    reg [31:0] tcp_session_age, tcp_retry_age;
    reg [1:0] tcp_retry_count;
    reg tcp_response_pending;
    reg [15:0] tcp_scan_index;
    reg [15:0] tcp_http_bytes, tcp_line_index, tcp_host_index, tcp_header_index;
    reg [31:0] tcp_header_tail;
    reg tcp_first_line, tcp_modern_match, tcp_legacy_match;
    reg tcp_path_modern, tcp_path_legacy, tcp_http_10;
    reg tcp_host_prefix_match, tcp_host_started, tcp_host_tail;
    reg tcp_host_modern_match, tcp_host_legacy_match;
    reg tcp_host_modern_seen, tcp_host_legacy_seen;
    reg http_legacy_reply;
    wire [15:0] frame_tcp_payload_length = frame_ipv4_total_length - 16'd20 - {10'h0,frame_tcp_data_offset,2'b00};
    wire tcp_peer_tuple_match = frame_src_mac==tcp_peer_mac && frame_ipv4_dst==tcp_peer_service_ipv4 &&
        frame_ipv4_src==tcp_peer_ip && frame_udp_ports[31:16]==tcp_peer_port &&
        frame_vlan_tagged==tcp_peer_vlan && (!tcp_peer_vlan ||
        (frame_vlan_tci==tcp_peer_tci && frame_vlan_tpid==tcp_peer_tpid));
    wire tcp_peer_match = tcp_session_active && tcp_peer_tuple_match;
    wire tcp_retransmit_due = tcp_response_pending && tcp_retry_age>=32'd62500000 && tcp_retry_count<3;
    wire [7:0] tcp_scan_byte=tcp_frame_bytes[tcp_scan_index[8:0]];
    wire [15:0] http_reply_bytes = http_legacy_reply ? 16'd72 : 16'd80;
    wire [7:0] tcp_service_flags = (reply_type==REPLY_HTTP) ? 8'h19 :
        ((reply_type==REPLY_TCP_FIN) ? 8'h11 : ((reply_type==REPLY_TCP_ACK) ? 8'h10 : ((reply_type==REPLY_TCP_RST) ? 8'h14 : 8'h12)));
    reg [15:0] tcp_client_port;
    reg [31:0] tcp_reply_sequence;
    reg [31:0] tcp_reply_acknowledgment;

    // Low-priority background traffic timer and per-frame identity.
    reg [23:0] bg_tick_count;
    reg [15:0] bg_lfsr;
    reg [1:0]  bg_source_slot;
    reg [7:0]  bg_target_host;
    wire [23:0] bg_interval_cycles =
        BG_RATE_MIN_CYCLES + {1'b0, bg_lfsr[6:0], 16'b0};
    wire [15:0] bg_frame_length =
        16'd60 + {8'h00, bg_lfsr[5:0], 2'b00};
    // DORA 完成前不灌 ARP/mDNS/LLMNR。上板窗口里 RX 两千多帧几乎全是
    // 背景流量，Offer 被挤在后面，Windows 可能在重试窗口内看不到。
    reg dhcp_acked;
    wire bg_due = BG_ENABLE && dhcp_acked && dma_runtime_ready &&
                  (bg_tick_count >= bg_interval_cycles);
    wire bg_lfsr_feedback =
        bg_lfsr[15] ^ bg_lfsr[13] ^ bg_lfsr[12] ^ bg_lfsr[10];

    // TCP Data Offset is expressed in 32-bit words.  Windows normally places
    // MSS/SACK/window-scale options on SYN and may retain timestamp options on
    // data packets, so treating every TCP header as exactly 20 bytes rejects
    // otherwise valid NCSI traffic and advances ACK by the wrong amount.
    wire [15:0] frame_tcp_header_bytes =
        {10'h000, frame_tcp_data_offset, 2'b00};

    function automatic [7:0] gw_mac_byte(input integer byte_index);
        begin
            case (byte_index[2:0])
                3'd0: gw_mac_byte = gw_mac[47:40];
                3'd1: gw_mac_byte = gw_mac[39:32];
                3'd2: gw_mac_byte = gw_mac[31:24];
                3'd3: gw_mac_byte = gw_mac[23:16];
                3'd4: gw_mac_byte = gw_mac[15:8];
                default: gw_mac_byte = gw_mac[7:0];
            endcase
        end
    endfunction

    function automatic [7:0] gw_ipv4_byte(input [1:0] byte_index);
        begin
            case (byte_index)
                2'd0: gw_ipv4_byte = gw_ipv4[31:24];
                2'd1: gw_ipv4_byte = gw_ipv4[23:16];
                2'd2: gw_ipv4_byte = gw_ipv4[15:8];
                default: gw_ipv4_byte = gw_ipv4[7:0];
            endcase
        end
    endfunction

    function automatic [7:0] dns_ascii_lower(input [7:0] value);
        begin
            if ((value >= 8'h41) && (value <= 8'h5A))
                dns_ascii_lower = value + 8'h20;
            else
                dns_ascii_lower = value;
        end
    endfunction

    // Wire-format QNAME for "dns.msftncsi.com" including the root label.
    function automatic [7:0] ncsi_dns_qname_byte(input [4:0] idx);
        begin
            case (idx)
                5'd0:  ncsi_dns_qname_byte = 8'h03;
                5'd1:  ncsi_dns_qname_byte = 8'h64;
                5'd2:  ncsi_dns_qname_byte = 8'h6e;
                5'd3:  ncsi_dns_qname_byte = 8'h73;
                5'd4:  ncsi_dns_qname_byte = 8'h08;
                5'd5:  ncsi_dns_qname_byte = 8'h6d;
                5'd6:  ncsi_dns_qname_byte = 8'h73;
                5'd7:  ncsi_dns_qname_byte = 8'h66;
                5'd8:  ncsi_dns_qname_byte = 8'h74;
                5'd9:  ncsi_dns_qname_byte = 8'h6e;
                5'd10: ncsi_dns_qname_byte = 8'h63;
                5'd11: ncsi_dns_qname_byte = 8'h73;
                5'd12: ncsi_dns_qname_byte = 8'h69;
                5'd13: ncsi_dns_qname_byte = 8'h03;
                5'd14: ncsi_dns_qname_byte = 8'h63;
                5'd15: ncsi_dns_qname_byte = 8'h6f;
                5'd16: ncsi_dns_qname_byte = 8'h6d;
                5'd17: ncsi_dns_qname_byte = 8'h00;
                default: ncsi_dns_qname_byte = 8'hFF;
            endcase
        end
    endfunction

    function automatic [7:0] web_dns_qname_byte(input [5:0] idx);
        case (idx)
            6'd0: web_dns_qname_byte = 8'h03;
            6'd1: web_dns_qname_byte = 8'h77;
            6'd2: web_dns_qname_byte = 8'h77;
            6'd3: web_dns_qname_byte = 8'h77;
            6'd4: web_dns_qname_byte = 8'h0F;
            6'd5: web_dns_qname_byte = 8'h6D;
            6'd6: web_dns_qname_byte = 8'h73;
            6'd7: web_dns_qname_byte = 8'h66;
            6'd8: web_dns_qname_byte = 8'h74;
            6'd9: web_dns_qname_byte = 8'h63;
            6'd10: web_dns_qname_byte = 8'h6F;
            6'd11: web_dns_qname_byte = 8'h6E;
            6'd12: web_dns_qname_byte = 8'h6E;
            6'd13: web_dns_qname_byte = 8'h65;
            6'd14: web_dns_qname_byte = 8'h63;
            6'd15: web_dns_qname_byte = 8'h74;
            6'd16: web_dns_qname_byte = 8'h74;
            6'd17: web_dns_qname_byte = 8'h65;
            6'd18: web_dns_qname_byte = 8'h73;
            6'd19: web_dns_qname_byte = 8'h74;
            6'd20: web_dns_qname_byte = 8'h03;
            6'd21: web_dns_qname_byte = 8'h63;
            6'd22: web_dns_qname_byte = 8'h6F;
            6'd23: web_dns_qname_byte = 8'h6D;
            6'd24: web_dns_qname_byte = 8'h00;
            default: web_dns_qname_byte = 8'h00;
        endcase
    endfunction
    function automatic [7:0] legacy_dns_qname_byte(input [5:0] idx);
        case (idx)
            6'd0: legacy_dns_qname_byte = 8'h03;
            6'd1: legacy_dns_qname_byte = 8'h77;
            6'd2: legacy_dns_qname_byte = 8'h77;
            6'd3: legacy_dns_qname_byte = 8'h77;
            6'd4: legacy_dns_qname_byte = 8'h08;
            6'd5: legacy_dns_qname_byte = 8'h6D;
            6'd6: legacy_dns_qname_byte = 8'h73;
            6'd7: legacy_dns_qname_byte = 8'h66;
            6'd8: legacy_dns_qname_byte = 8'h74;
            6'd9: legacy_dns_qname_byte = 8'h6E;
            6'd10: legacy_dns_qname_byte = 8'h63;
            6'd11: legacy_dns_qname_byte = 8'h73;
            6'd12: legacy_dns_qname_byte = 8'h69;
            6'd13: legacy_dns_qname_byte = 8'h03;
            6'd14: legacy_dns_qname_byte = 8'h63;
            6'd15: legacy_dns_qname_byte = 8'h6F;
            6'd16: legacy_dns_qname_byte = 8'h6D;
            6'd17: legacy_dns_qname_byte = 8'h00;
            default: legacy_dns_qname_byte = 8'h00;
        endcase
    endfunction

    // Default DNS A is still the gateway. Only dns.msftncsi.com uses the
    // Microsoft probe addresses; first RR is the required 131.107.255.255.
    function automatic [31:0] dns_answer_ipv4(input [3:0] rr_index);
        begin
            if (!dns_ncsi)
                dns_answer_ipv4 = gw_ipv4;
            else begin
                case (rr_index)
                    4'd0: dns_answer_ipv4 = NCSI_DNS_A0;
                    4'd1: dns_answer_ipv4 = NCSI_DNS_A1;
                    4'd2: dns_answer_ipv4 = NCSI_DNS_A2;
                    4'd3: dns_answer_ipv4 = NCSI_DNS_A3;
                    4'd4: dns_answer_ipv4 = NCSI_DNS_A4;
                    default: dns_answer_ipv4 = 32'h00000000;
                endcase
            end
        end
    endfunction

    function automatic [7:0] client_mac_byte(input [2:0] byte_index);
        begin
            case (byte_index)
                3'd0: client_mac_byte = client_mac[47:40];
                3'd1: client_mac_byte = client_mac[39:32];
                3'd2: client_mac_byte = client_mac[31:24];
                3'd3: client_mac_byte = client_mac[23:16];
                3'd4: client_mac_byte = client_mac[15:8];
                default: client_mac_byte = client_mac[7:0];
            endcase
        end
    endfunction

    function automatic [7:0] client_ipv4_byte(input [1:0] byte_index);
        begin
            case (byte_index)
                2'd0: client_ipv4_byte = client_ipv4[31:24];
                2'd1: client_ipv4_byte = client_ipv4[23:16];
                2'd2: client_ipv4_byte = client_ipv4[15:8];
                default: client_ipv4_byte = client_ipv4[7:0];
            endcase
        end
    endfunction

    function automatic [7:0] bg_sender_mac_byte(input [2:0] byte_index);
        begin
            case (byte_index)
                3'd0: bg_sender_mac_byte = gw_mac[47:40];
                3'd1: bg_sender_mac_byte = gw_mac[39:32];
                3'd2: bg_sender_mac_byte = gw_mac[31:24];
                3'd3: bg_sender_mac_byte = subnet_octet;
                3'd4: bg_sender_mac_byte =
                          8'h64 + {6'h00, bg_source_slot};
                default: bg_sender_mac_byte = 8'h01;
            endcase
        end
    endfunction

    function automatic [7:0] bg_sender_ipv4_byte(input [1:0] byte_index);
        begin
            case (byte_index)
                2'd0: bg_sender_ipv4_byte = 8'hC0;
                2'd1: bg_sender_ipv4_byte = 8'hA8;
                2'd2: bg_sender_ipv4_byte = subnet_octet;
                default: bg_sender_ipv4_byte =
                             8'd100 + {6'h00, bg_source_slot};
            endcase
        end
    endfunction

    function automatic [7:0] xid_byte(input [1:0] byte_index);
        begin
            case (byte_index)
                2'd0: xid_byte = dhcp_xid[31:24];
                2'd1: xid_byte = dhcp_xid[23:16];
                2'd2: xid_byte = dhcp_xid[15:8];
                default: xid_byte = dhcp_xid[7:0];
            endcase
        end
    endfunction

    function automatic [15:0] icmp_checksum_from_byte_sums(
        input [23:0] even_byte_sum,
        input [23:0] odd_byte_sum
    );
        reg [31:0] checksum_sum;
        begin
            checksum_sum =
                ({8'h00, even_byte_sum} << 8) +
                {8'h00, odd_byte_sum};
            checksum_sum =
                {16'h0000, checksum_sum[15:0]} +
                {16'h0000, checksum_sum[31:16]};
            checksum_sum =
                {16'h0000, checksum_sum[15:0]} +
                {31'h00000000, checksum_sum[16]};
            icmp_checksum_from_byte_sums = ~checksum_sum[15:0];
        end
    endfunction

    // Windows可能把IPv4头校验和卸载给网卡；回复不能直接复用尚未补齐的值。
    function automatic [15:0] ipv4_reply_header_checksum(
        input [15:0] version_tos,
        input [15:0] total_length,
        input [15:0] identification,
        input [15:0] flags_fragment,
        input [15:0] ttl_protocol,
        input [31:0] destination_ipv4
    );
        reg [19:0] checksum_sum;
        reg [16:0] checksum_fold_1;
        reg [16:0] checksum_fold_2;
        begin
            checksum_sum =
                {4'h0, version_tos} +
                {4'h0, total_length} +
                {4'h0, identification} +
                {4'h0, flags_fragment} +
                {4'h0, ttl_protocol} +
                {4'h0, gw_ipv4[31:16]} +
                {4'h0, gw_ipv4[15:0]} +
                {4'h0, destination_ipv4[31:16]} +
                {4'h0, destination_ipv4[15:0]};
            checksum_fold_1 =
                {1'b0, checksum_sum[15:0]} +
                {13'h0000, checksum_sum[19:16]};
            checksum_fold_2 =
                {1'b0, checksum_fold_1[15:0]} +
                checksum_fold_1[16];
            ipv4_reply_header_checksum = ~checksum_fold_2[15:0];
        end
    endfunction

    function automatic [7:0] dhcp_option_byte(input [9:0] idx);
        reg [9:0] pos, cid_size;
        reg nak;
        begin
            nak = (reply_type == REPLY_DHCP_NAK);
            cid_size = dhcp_client_id_valid ? (10'd2 + dhcp_client_id_length) : 10'd0;
            dhcp_option_byte = 0;
            if (idx < 9) begin
                case (idx)
                    0: dhcp_option_byte=53; 1: dhcp_option_byte=1;
                    2: dhcp_option_byte=nak ? 6 : ((reply_type == REPLY_DHCP_ACK) ? 5 : 2);
                    3: dhcp_option_byte=54; 4: dhcp_option_byte=4;
                    default: dhcp_option_byte=gw_ipv4_byte(idx-5);
                endcase
            end else if (!nak && idx < 27) begin
                case (idx)
                    9: dhcp_option_byte=51; 10: dhcp_option_byte=4;
                    13: dhcp_option_byte=8'hA8; 14: dhcp_option_byte=8'hC0;
                    15: dhcp_option_byte=1; 16: dhcp_option_byte=4;
                    17,18,19: dhcp_option_byte=255;
                    21: dhcp_option_byte=28; 22: dhcp_option_byte=4;
                    23: dhcp_option_byte=8'hC0; 24: dhcp_option_byte=8'hA8;
                    25: dhcp_option_byte=subnet_octet; 26: dhcp_option_byte=255;
                    default: dhcp_option_byte=0;
                endcase
            end else begin
                pos = idx - (nak ? 10'd9 : 10'd27);
                if (pos < cid_size) begin
                    if (pos==0) dhcp_option_byte=61;
                    else if (pos==1) dhcp_option_byte=dhcp_client_id_length;
                    else dhcp_option_byte=dhcp_client_id_bytes[pos-2];
                end else begin
                    pos=pos-cid_size;
                    if (ADVERTISE_ROUTER_DNS && !nak && pos<12) begin
                        case (pos)
                            0: dhcp_option_byte=3; 1,7: dhcp_option_byte=4;
                            6: dhcp_option_byte=6;
                            default: dhcp_option_byte=gw_ipv4_byte((pos<6) ? pos-2 : pos-8);
                        endcase
                    end else if (pos == ((ADVERTISE_ROUTER_DNS && !nak) ? 12 : 0))
                        dhcp_option_byte=255;
                end
            end
        end
    endfunction

    function automatic [15:0] dhcp_frame_bytes(input nak, input cid_valid, input [7:0] cid_length);
        reg [15:0] bootp_length;
        begin
            bootp_length = 16'd240 + (nak ? 16'd9 : 16'd27) +
                (cid_valid ? 16'd2+cid_length : 16'd0) +
                ((ADVERTISE_ROUTER_DNS && !nak) ? 16'd12 : 16'd0) + 16'd1;
            if (bootp_length < 312) bootp_length=312;
            dhcp_frame_bytes=16'd42+bootp_length;
        end
    endfunction

    function automatic [15:0] ipv4_header_checksum_pair(
        input [15:0] version_tos,
        input [15:0] total_length,
        input [15:0] identification,
        input [15:0] flags_fragment,
        input [15:0] ttl_protocol,
        input [31:0] source_ipv4,
        input [31:0] destination_ipv4
    );
        reg [19:0] checksum_sum;
        reg [16:0] checksum_fold_1;
        reg [16:0] checksum_fold_2;
        begin
            checksum_sum =
                {4'h0, version_tos} +
                {4'h0, total_length} +
                {4'h0, identification} +
                {4'h0, flags_fragment} +
                {4'h0, ttl_protocol} +
                {4'h0, source_ipv4[31:16]} +
                {4'h0, source_ipv4[15:0]} +
                {4'h0, destination_ipv4[31:16]} +
                {4'h0, destination_ipv4[15:0]};
            checksum_fold_1 =
                {1'b0, checksum_sum[15:0]} +
                {13'h0000, checksum_sum[19:16]};
            checksum_fold_2 =
                {1'b0, checksum_fold_1[15:0]} +
                checksum_fold_1[16];
            ipv4_header_checksum_pair = ~checksum_fold_2[15:0];
        end
    endfunction

    function automatic [15:0] internet_checksum(
        input [31:0] checksum_sum
    );
        reg [31:0] checksum_fold_1;
        reg [31:0] checksum_fold_2;
        begin
            checksum_fold_1 =
                {16'h0000, checksum_sum[15:0]} +
                {16'h0000, checksum_sum[31:16]};
            checksum_fold_2 =
                {16'h0000, checksum_fold_1[15:0]} +
                {31'h00000000, checksum_fold_1[16]};
            internet_checksum = ~checksum_fold_2[15:0];
        end
    endfunction

    function automatic [7:0] http_modern_line_byte(input [9:0] idx);
        case(idx)
            10'd0: http_modern_line_byte=8'h47;
            10'd1: http_modern_line_byte=8'h45;
            10'd2: http_modern_line_byte=8'h54;
            10'd3: http_modern_line_byte=8'h20;
            10'd4: http_modern_line_byte=8'h2F;
            10'd5: http_modern_line_byte=8'h63;
            10'd6: http_modern_line_byte=8'h6F;
            10'd7: http_modern_line_byte=8'h6E;
            10'd8: http_modern_line_byte=8'h6E;
            10'd9: http_modern_line_byte=8'h65;
            10'd10: http_modern_line_byte=8'h63;
            10'd11: http_modern_line_byte=8'h74;
            10'd12: http_modern_line_byte=8'h74;
            10'd13: http_modern_line_byte=8'h65;
            10'd14: http_modern_line_byte=8'h73;
            10'd15: http_modern_line_byte=8'h74;
            10'd16: http_modern_line_byte=8'h2E;
            10'd17: http_modern_line_byte=8'h74;
            10'd18: http_modern_line_byte=8'h78;
            10'd19: http_modern_line_byte=8'h74;
            10'd20: http_modern_line_byte=8'h20;
            10'd21: http_modern_line_byte=8'h48;
            10'd22: http_modern_line_byte=8'h54;
            10'd23: http_modern_line_byte=8'h54;
            10'd24: http_modern_line_byte=8'h50;
            10'd25: http_modern_line_byte=8'h2F;
            10'd26: http_modern_line_byte=8'h31;
            10'd27: http_modern_line_byte=8'h2E;
            10'd28: http_modern_line_byte=8'h31;
            10'd29: http_modern_line_byte=8'h0D;
            10'd30: http_modern_line_byte=8'h0A;
            default: http_modern_line_byte=0;
        endcase
    endfunction
    function automatic [7:0] http_legacy_line_byte(input [9:0] idx);
        case(idx)
            10'd0: http_legacy_line_byte=8'h47;
            10'd1: http_legacy_line_byte=8'h45;
            10'd2: http_legacy_line_byte=8'h54;
            10'd3: http_legacy_line_byte=8'h20;
            10'd4: http_legacy_line_byte=8'h2F;
            10'd5: http_legacy_line_byte=8'h6E;
            10'd6: http_legacy_line_byte=8'h63;
            10'd7: http_legacy_line_byte=8'h73;
            10'd8: http_legacy_line_byte=8'h69;
            10'd9: http_legacy_line_byte=8'h2E;
            10'd10: http_legacy_line_byte=8'h74;
            10'd11: http_legacy_line_byte=8'h78;
            10'd12: http_legacy_line_byte=8'h74;
            10'd13: http_legacy_line_byte=8'h20;
            10'd14: http_legacy_line_byte=8'h48;
            10'd15: http_legacy_line_byte=8'h54;
            10'd16: http_legacy_line_byte=8'h54;
            10'd17: http_legacy_line_byte=8'h50;
            10'd18: http_legacy_line_byte=8'h2F;
            10'd19: http_legacy_line_byte=8'h31;
            10'd20: http_legacy_line_byte=8'h2E;
            10'd21: http_legacy_line_byte=8'h31;
            10'd22: http_legacy_line_byte=8'h0D;
            10'd23: http_legacy_line_byte=8'h0A;
            default: http_legacy_line_byte=0;
        endcase
    endfunction
    function automatic [7:0] http_host_prefix_byte(input [9:0] idx);
        case(idx)
            10'd0: http_host_prefix_byte=8'h68;
            10'd1: http_host_prefix_byte=8'h6F;
            10'd2: http_host_prefix_byte=8'h73;
            10'd3: http_host_prefix_byte=8'h74;
            10'd4: http_host_prefix_byte=8'h3A;
            default: http_host_prefix_byte=0;
        endcase
    endfunction
    function automatic [7:0] http_modern_host_byte(input [9:0] idx);
        case(idx)
            10'd0: http_modern_host_byte=8'h77;
            10'd1: http_modern_host_byte=8'h77;
            10'd2: http_modern_host_byte=8'h77;
            10'd3: http_modern_host_byte=8'h2E;
            10'd4: http_modern_host_byte=8'h6D;
            10'd5: http_modern_host_byte=8'h73;
            10'd6: http_modern_host_byte=8'h66;
            10'd7: http_modern_host_byte=8'h74;
            10'd8: http_modern_host_byte=8'h63;
            10'd9: http_modern_host_byte=8'h6F;
            10'd10: http_modern_host_byte=8'h6E;
            10'd11: http_modern_host_byte=8'h6E;
            10'd12: http_modern_host_byte=8'h65;
            10'd13: http_modern_host_byte=8'h63;
            10'd14: http_modern_host_byte=8'h74;
            10'd15: http_modern_host_byte=8'h74;
            10'd16: http_modern_host_byte=8'h65;
            10'd17: http_modern_host_byte=8'h73;
            10'd18: http_modern_host_byte=8'h74;
            10'd19: http_modern_host_byte=8'h2E;
            10'd20: http_modern_host_byte=8'h63;
            10'd21: http_modern_host_byte=8'h6F;
            10'd22: http_modern_host_byte=8'h6D;
            10'd23: http_modern_host_byte=8'h3A;
            10'd24: http_modern_host_byte=8'h38;
            10'd25: http_modern_host_byte=8'h30;
            default: http_modern_host_byte=0;
        endcase
    endfunction
    function automatic [7:0] http_legacy_host_byte(input [9:0] idx);
        case(idx)
            10'd0: http_legacy_host_byte=8'h77;
            10'd1: http_legacy_host_byte=8'h77;
            10'd2: http_legacy_host_byte=8'h77;
            10'd3: http_legacy_host_byte=8'h2E;
            10'd4: http_legacy_host_byte=8'h6D;
            10'd5: http_legacy_host_byte=8'h73;
            10'd6: http_legacy_host_byte=8'h66;
            10'd7: http_legacy_host_byte=8'h74;
            10'd8: http_legacy_host_byte=8'h6E;
            10'd9: http_legacy_host_byte=8'h63;
            10'd10: http_legacy_host_byte=8'h73;
            10'd11: http_legacy_host_byte=8'h69;
            10'd12: http_legacy_host_byte=8'h2E;
            10'd13: http_legacy_host_byte=8'h63;
            10'd14: http_legacy_host_byte=8'h6F;
            10'd15: http_legacy_host_byte=8'h6D;
            10'd16: http_legacy_host_byte=8'h3A;
            10'd17: http_legacy_host_byte=8'h38;
            10'd18: http_legacy_host_byte=8'h30;
            default: http_legacy_host_byte=0;
        endcase
    endfunction
    function automatic [7:0] http_legacy_payload_byte(input [9:0] idx);
        case(idx)
            10'd0: http_legacy_payload_byte=8'h48;
            10'd1: http_legacy_payload_byte=8'h54;
            10'd2: http_legacy_payload_byte=8'h54;
            10'd3: http_legacy_payload_byte=8'h50;
            10'd4: http_legacy_payload_byte=8'h2F;
            10'd5: http_legacy_payload_byte=8'h31;
            10'd6: http_legacy_payload_byte=8'h2E;
            10'd7: http_legacy_payload_byte=8'h31;
            10'd8: http_legacy_payload_byte=8'h20;
            10'd9: http_legacy_payload_byte=8'h32;
            10'd10: http_legacy_payload_byte=8'h30;
            10'd11: http_legacy_payload_byte=8'h30;
            10'd12: http_legacy_payload_byte=8'h20;
            10'd13: http_legacy_payload_byte=8'h4F;
            10'd14: http_legacy_payload_byte=8'h4B;
            10'd15: http_legacy_payload_byte=8'h0D;
            10'd16: http_legacy_payload_byte=8'h0A;
            10'd17: http_legacy_payload_byte=8'h43;
            10'd18: http_legacy_payload_byte=8'h6F;
            10'd19: http_legacy_payload_byte=8'h6E;
            10'd20: http_legacy_payload_byte=8'h74;
            10'd21: http_legacy_payload_byte=8'h65;
            10'd22: http_legacy_payload_byte=8'h6E;
            10'd23: http_legacy_payload_byte=8'h74;
            10'd24: http_legacy_payload_byte=8'h2D;
            10'd25: http_legacy_payload_byte=8'h4C;
            10'd26: http_legacy_payload_byte=8'h65;
            10'd27: http_legacy_payload_byte=8'h6E;
            10'd28: http_legacy_payload_byte=8'h67;
            10'd29: http_legacy_payload_byte=8'h74;
            10'd30: http_legacy_payload_byte=8'h68;
            10'd31: http_legacy_payload_byte=8'h3A;
            10'd32: http_legacy_payload_byte=8'h20;
            10'd33: http_legacy_payload_byte=8'h31;
            10'd34: http_legacy_payload_byte=8'h34;
            10'd35: http_legacy_payload_byte=8'h0D;
            10'd36: http_legacy_payload_byte=8'h0A;
            10'd37: http_legacy_payload_byte=8'h43;
            10'd38: http_legacy_payload_byte=8'h6F;
            10'd39: http_legacy_payload_byte=8'h6E;
            10'd40: http_legacy_payload_byte=8'h6E;
            10'd41: http_legacy_payload_byte=8'h65;
            10'd42: http_legacy_payload_byte=8'h63;
            10'd43: http_legacy_payload_byte=8'h74;
            10'd44: http_legacy_payload_byte=8'h69;
            10'd45: http_legacy_payload_byte=8'h6F;
            10'd46: http_legacy_payload_byte=8'h6E;
            10'd47: http_legacy_payload_byte=8'h3A;
            10'd48: http_legacy_payload_byte=8'h20;
            10'd49: http_legacy_payload_byte=8'h63;
            10'd50: http_legacy_payload_byte=8'h6C;
            10'd51: http_legacy_payload_byte=8'h6F;
            10'd52: http_legacy_payload_byte=8'h73;
            10'd53: http_legacy_payload_byte=8'h65;
            10'd54: http_legacy_payload_byte=8'h0D;
            10'd55: http_legacy_payload_byte=8'h0A;
            10'd56: http_legacy_payload_byte=8'h0D;
            10'd57: http_legacy_payload_byte=8'h0A;
            10'd58: http_legacy_payload_byte=8'h4D;
            10'd59: http_legacy_payload_byte=8'h69;
            10'd60: http_legacy_payload_byte=8'h63;
            10'd61: http_legacy_payload_byte=8'h72;
            10'd62: http_legacy_payload_byte=8'h6F;
            10'd63: http_legacy_payload_byte=8'h73;
            10'd64: http_legacy_payload_byte=8'h6F;
            10'd65: http_legacy_payload_byte=8'h66;
            10'd66: http_legacy_payload_byte=8'h74;
            10'd67: http_legacy_payload_byte=8'h20;
            10'd68: http_legacy_payload_byte=8'h4E;
            10'd69: http_legacy_payload_byte=8'h43;
            10'd70: http_legacy_payload_byte=8'h53;
            10'd71: http_legacy_payload_byte=8'h49;
            default: http_legacy_payload_byte=0;
        endcase
    endfunction

    // Windows NCSI active probe expects the response body to be exactly
    // "Microsoft Connect Test".  Returning a generic HTTP 200/"OK" keeps the
    // link up but leaves Network List Manager in the "No Internet" state.
    localparam integer NCSI_HTTP_PAYLOAD_BYTES = 80;

    function automatic [7:0] http_payload_byte(
        input [6:0] payload_offset
    );
        begin
            case (payload_offset)
                6'd0:  http_payload_byte = 8'h48;
                6'd1:  http_payload_byte = 8'h54;
                6'd2:  http_payload_byte = 8'h54;
                6'd3:  http_payload_byte = 8'h50;
                6'd4:  http_payload_byte = 8'h2F;
                6'd5:  http_payload_byte = 8'h31;
                6'd6:  http_payload_byte = 8'h2E;
                6'd7:  http_payload_byte = 8'h31;
                6'd8:  http_payload_byte = 8'h20;
                6'd9:  http_payload_byte = 8'h32;
                6'd10: http_payload_byte = 8'h30;
                6'd11: http_payload_byte = 8'h30;
                6'd12: http_payload_byte = 8'h20;
                6'd13: http_payload_byte = 8'h4F;
                6'd14: http_payload_byte = 8'h4B;
                6'd15: http_payload_byte = 8'h0D;
                6'd16: http_payload_byte = 8'h0A;
                6'd17: http_payload_byte = 8'h43;
                6'd18: http_payload_byte = 8'h6F;
                6'd19: http_payload_byte = 8'h6E;
                6'd20: http_payload_byte = 8'h74;
                6'd21: http_payload_byte = 8'h65;
                6'd22: http_payload_byte = 8'h6E;
                6'd23: http_payload_byte = 8'h74;
                6'd24: http_payload_byte = 8'h2D;
                6'd25: http_payload_byte = 8'h4C;
                6'd26: http_payload_byte = 8'h65;
                6'd27: http_payload_byte = 8'h6E;
                6'd28: http_payload_byte = 8'h67;
                6'd29: http_payload_byte = 8'h74;
                6'd30: http_payload_byte = 8'h68;
                6'd31: http_payload_byte = 8'h3A;
                6'd32: http_payload_byte = 8'h20;
                6'd33: http_payload_byte = 8'h32;
                6'd34: http_payload_byte = 8'h32;
                6'd35: http_payload_byte = 8'h0D;
                6'd36: http_payload_byte = 8'h0A;
                6'd37: http_payload_byte = 8'h43;
                6'd38: http_payload_byte = 8'h6F;
                6'd39: http_payload_byte = 8'h6E;
                6'd40: http_payload_byte = 8'h6E;
                6'd41: http_payload_byte = 8'h65;
                6'd42: http_payload_byte = 8'h63;
                6'd43: http_payload_byte = 8'h74;
                6'd44: http_payload_byte = 8'h69;
                6'd45: http_payload_byte = 8'h6F;
                6'd46: http_payload_byte = 8'h6E;
                6'd47: http_payload_byte = 8'h3A;
                6'd48: http_payload_byte = 8'h20;
                6'd49: http_payload_byte = 8'h63;
                6'd50: http_payload_byte = 8'h6C;
                6'd51: http_payload_byte = 8'h6F;
                6'd52: http_payload_byte = 8'h73;
                6'd53: http_payload_byte = 8'h65;
                6'd54: http_payload_byte = 8'h0D;
                6'd55: http_payload_byte = 8'h0A;
                6'd56: http_payload_byte = 8'h0D;
                6'd57: http_payload_byte = 8'h0A;
                6'd58: http_payload_byte = 8'h4D;
                6'd59: http_payload_byte = 8'h69;
                6'd60: http_payload_byte = 8'h63;
                6'd61: http_payload_byte = 8'h72;
                6'd62: http_payload_byte = 8'h6F;
                6'd63: http_payload_byte = 8'h73;
                7'd64: http_payload_byte = 8'h6F;
                7'd65: http_payload_byte = 8'h66;
                7'd66: http_payload_byte = 8'h74;
                7'd67: http_payload_byte = 8'h20;
                7'd68: http_payload_byte = 8'h43;
                7'd69: http_payload_byte = 8'h6F;
                7'd70: http_payload_byte = 8'h6E;
                7'd71: http_payload_byte = 8'h6E;
                7'd72: http_payload_byte = 8'h65;
                7'd73: http_payload_byte = 8'h63;
                7'd74: http_payload_byte = 8'h74;
                7'd75: http_payload_byte = 8'h20;
                7'd76: http_payload_byte = 8'h54;
                7'd77: http_payload_byte = 8'h65;
                7'd78: http_payload_byte = 8'h73;
                7'd79: http_payload_byte = 8'h74;
                default: http_payload_byte = 8'h00;
            endcase
        end
    endfunction

    function automatic [15:0] tcp_service_checksum(
        input [31:0] destination_ipv4,
        input [15:0] destination_port,
        input [31:0] sequence_number,
        input [31:0] acknowledgment_number,
        input        include_http_payload
    );
        reg [31:0] checksum_sum;
        reg [15:0] tcp_length;
        reg [7:0]  tcp_flags;
        begin
            tcp_length = include_http_payload ?
                         (16'd20 + http_reply_bytes) : 16'd20;
            tcp_flags = tcp_service_flags;
            checksum_sum =
                {16'h0000, service_ipv4[31:16]} + {16'h0000, service_ipv4[15:0]} +
                {16'h0000, destination_ipv4[31:16]} +
                {16'h0000, destination_ipv4[15:0]} +
                32'h00000006 + {16'h0000, tcp_length} +
                32'h00000050 + {16'h0000, destination_port} +
                {16'h0000, sequence_number[31:16]} +
                {16'h0000, sequence_number[15:0]} +
                {16'h0000, acknowledgment_number[31:16]} +
                {16'h0000, acknowledgment_number[15:0]} +
                {16'h0000, 8'h50, tcp_flags} +
                ((reply_type == REPLY_TCP_RST) ? 32'h0 : 32'h00004000);
            if (include_http_payload)
                // Sum of all big-endian 16-bit words in the 80-byte NCSI
                // HTTP response above.
                checksum_sum = checksum_sum + (http_legacy_reply ? 32'h000AABD2 : 32'h000C5B80);
            tcp_service_checksum = internet_checksum(checksum_sum);
        end
    endfunction

    // CRC accumulation never traverses the IPv4/TCP checksum template cloud.
    function automatic [7:0] dhcp_udp_payload_byte(input [9:0] offset);
        reg [15:0] flags;
        begin
            flags=(reply_type==REPLY_DHCP_NAK) ? 16'h8000 : {dhcp_flags[15],15'h0};
            dhcp_udp_payload_byte=0;
            case(offset)
                35:dhcp_udp_payload_byte=67; 37:dhcp_udp_payload_byte=68;
                38:dhcp_udp_payload_byte=(reply_core_length-34)>>8;
                39:dhcp_udp_payload_byte=reply_core_length-34;
                42:dhcp_udp_payload_byte=2; 43:dhcp_udp_payload_byte=1; 44:dhcp_udp_payload_byte=6;
                46:dhcp_udp_payload_byte=xid_byte(0); 47:dhcp_udp_payload_byte=xid_byte(1);
                48:dhcp_udp_payload_byte=xid_byte(2); 49:dhcp_udp_payload_byte=xid_byte(3);
                52:dhcp_udp_payload_byte=flags[15:8]; 53:dhcp_udp_payload_byte=flags[7:0];
                54:dhcp_udp_payload_byte=dhcp_ciaddr[31:24]; 55:dhcp_udp_payload_byte=dhcp_ciaddr[23:16];
                56:dhcp_udp_payload_byte=dhcp_ciaddr[15:8]; 57:dhcp_udp_payload_byte=dhcp_ciaddr[7:0];
                58:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : lease_ipv4[31:24];
                59:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : lease_ipv4[23:16];
                60:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : lease_ipv4[15:8];
                61:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : lease_ipv4[7:0];
                62:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(0); 63:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(1);
                64:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(2); 65:dhcp_udp_payload_byte=(reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(3);
                70:dhcp_udp_payload_byte=client_mac_byte(0);71:dhcp_udp_payload_byte=client_mac_byte(1);
                72:dhcp_udp_payload_byte=client_mac_byte(2);73:dhcp_udp_payload_byte=client_mac_byte(3);
                74:dhcp_udp_payload_byte=client_mac_byte(4);75:dhcp_udp_payload_byte=client_mac_byte(5);
                278:dhcp_udp_payload_byte=8'h63;279:dhcp_udp_payload_byte=8'h82;
                280:dhcp_udp_payload_byte=8'h53;281:dhcp_udp_payload_byte=8'h63;
                default:if(offset>=282) dhcp_udp_payload_byte=dhcp_option_byte(offset-282);
            endcase
        end
    endfunction

    // DHCP uses a bounded variable-length Client Identifier; FCS is not stored.
    // DHCP 帧至少354字节；FCS不写入主机缓冲区，
    // 但返回描述符长度会按真实 tg3 行为额外包含 4 字节。
    function automatic [7:0] service_ipv4_byte(input [1:0] idx);
        case(idx)
            0:service_ipv4_byte=service_ipv4[31:24];
            1:service_ipv4_byte=service_ipv4[23:16];
            2:service_ipv4_byte=service_ipv4[15:8];
            3:service_ipv4_byte=service_ipv4[7:0];
        endcase
    endfunction

    function automatic [7:0] reply_core_byte(input [9:0] byte_offset);
        reg [7:0] dhcp_message_type;
        reg [15:0] service_ip_total_length;
        reg [15:0] service_ip_checksum;
        reg [15:0] service_udp_length;
        reg [15:0] service_tcp_checksum;
        reg [7:0]  service_tcp_flags;
        reg [15:0] dhcp_ip_checksum;
        reg [15:0] dhcp_udp_csum;
        reg [15:0] dhcp_bootp_flags;
        reg [31:0] dhcp_dest_ipv4;
        reg [15:0] bg_mcast_ip_checksum;
        reg [31:0] bg_src_ipv4;
        reg [9:0]  dns_ans_off;
        reg [31:0] dns_a_ipv4;
        begin
            reply_core_byte = 8'h00;
            dns_ans_off = 10'd0;
            dns_a_ipv4 = 32'h0;
            bg_src_ipv4 = {
                16'hC0A8, subnet_octet,
                8'd100 + {6'h00, bg_source_slot}
            };
            if (reply_type == REPLY_BG_MDNS)
                bg_mcast_ip_checksum = ipv4_header_checksum_pair(
                    16'h4500, 16'd56, bg_lfsr, 16'h0000, 16'hFF11,
                    bg_src_ipv4, 32'hE00000FB);
            else if (reply_type == REPLY_BG_LLMNR)
                bg_mcast_ip_checksum = ipv4_header_checksum_pair(
                    16'h4500, 16'd50, bg_lfsr, 16'h0000, 16'h0111,
                    bg_src_ipv4, 32'hE00000FC);
            else
                bg_mcast_ip_checksum = 16'h0000;
            dhcp_message_type =
                (reply_type == REPLY_DHCP_ACK) ? 8'h05 : 8'h02;
            service_ip_total_length =
                ((reply_type == REPLY_TCP_SYNACK) || (reply_type == REPLY_TCP_FIN) || (reply_type == REPLY_TCP_ACK) || (reply_type == REPLY_TCP_RST)) ?
                    16'd40 : (reply_core_length - 16'd14);
            service_udp_length = reply_core_length - 16'd34;
            service_ip_checksum = ipv4_header_checksum_pair(
                16'h4500, service_ip_total_length, reply_ipv4_identification,
                16'h4000, (reply_type == REPLY_DNS) ? 16'h4011 : 16'h4006,
                service_ipv4, client_ipv4);
            // 三层跟 Discover broadcast flag：置位则 L2/L3/flags 全广播，
            // 否则 L2=chaddr、L3=yiaddr、flags=0。禁止混合。
            dhcp_bootp_flags = (reply_type == REPLY_DHCP_NAK) ? 16'h8000 : {dhcp_flags[15],15'h0};
            dhcp_dest_ipv4 = (reply_type == REPLY_DHCP_NAK) ? 32'hFFFFFFFF :
                ((dhcp_ciaddr != 0) ? dhcp_ciaddr : (dhcp_flags[15] ? 32'hFFFFFFFF : lease_ipv4));
            dhcp_ip_checksum = ipv4_reply_header_checksum(
                16'h4500,
                reply_core_length - 16'd14,
                16'h0000,
                16'h4000,
                16'h4011,
                dhcp_dest_ipv4);
            dhcp_udp_csum = dhcp_checksum_running ? 16'h0000 : dhcp_reply_udp_checksum;
            service_tcp_flags =
                tcp_service_flags;
            if (ENABLE_TCP_HTTP)
                service_tcp_checksum = tcp_service_checksum(
                    client_ipv4,
                    tcp_client_port,
                    tcp_reply_sequence,
                    tcp_reply_acknowledgment,
                    (reply_type == REPLY_HTTP));
            else
                service_tcp_checksum = 16'h0000;

            if (reply_type == REPLY_ICMP) begin
                if ((byte_offset < icmp_request_length) &&
                    (byte_offset < ICMP_CAPTURE_BYTES))
                    reply_core_byte = icmp_request_bytes[(byte_offset + reply_vlan_bytes) & 7'h7F];

                if (byte_offset < 6)
                    reply_core_byte = client_mac_byte(byte_offset[2:0]);
                else if ((byte_offset >= 6) && (byte_offset < 12)) begin
                    reply_core_byte = gw_mac_byte(byte_offset - 10'd6);
                end

                if (byte_offset == icmp_l3_offset)
                    reply_core_byte = 8'h45;
                if (byte_offset == (icmp_l3_offset + 10'd1))
                    reply_core_byte = frame_ipv4_tos;
                if (byte_offset == (icmp_l3_offset + 10'd2))
                    reply_core_byte = frame_ipv4_total_length[15:8];
                if (byte_offset == (icmp_l3_offset + 10'd3))
                    reply_core_byte = frame_ipv4_total_length[7:0];
                if (byte_offset == (icmp_l3_offset + 10'd4))
                    reply_core_byte = frame_ipv4_identification[15:8];
                if (byte_offset == (icmp_l3_offset + 10'd5))
                    reply_core_byte = frame_ipv4_identification[7:0];
                if (byte_offset == (icmp_l3_offset + 10'd6))
                    reply_core_byte = frame_ipv4_flags_fragment[15:8];
                if (byte_offset == (icmp_l3_offset + 10'd7))
                    reply_core_byte = frame_ipv4_flags_fragment[7:0];
                if (byte_offset == (icmp_l3_offset + 10'd8))
                    reply_core_byte = 8'h40;
                if (byte_offset == (icmp_l3_offset + 10'd9))
                    reply_core_byte = 8'h01;
                if (byte_offset == (icmp_l3_offset + 10'd12))
                    reply_core_byte = gw_ipv4_byte(0);
                if (byte_offset == (icmp_l3_offset + 10'd13))
                    reply_core_byte = gw_ipv4_byte(1);
                if (byte_offset == (icmp_l3_offset + 10'd14))
                    reply_core_byte = gw_ipv4_byte(2);
                if (byte_offset == (icmp_l3_offset + 10'd15))
                    reply_core_byte = gw_ipv4_byte(3);
                if (byte_offset == (icmp_l3_offset + 10'd16))
                    reply_core_byte = client_ipv4_byte(0);
                if (byte_offset == (icmp_l3_offset + 10'd17))
                    reply_core_byte = client_ipv4_byte(1);
                if (byte_offset == (icmp_l3_offset + 10'd18))
                    reply_core_byte = client_ipv4_byte(2);
                if (byte_offset == (icmp_l3_offset + 10'd19))
                    reply_core_byte = client_ipv4_byte(3);
                if (byte_offset == (icmp_l3_offset + 10'd10))
                    reply_core_byte = icmp_reply_ip_checksum[15:8];
                if (byte_offset == (icmp_l3_offset + 10'd11))
                    reply_core_byte = icmp_reply_ip_checksum[7:0];
                if (byte_offset == icmp_l4_offset)
                    reply_core_byte = 8'h00;
                if (byte_offset == (icmp_l4_offset + 10'd2))
                    reply_core_byte = icmp_reply_message_checksum[15:8];
                if (byte_offset == (icmp_l4_offset + 10'd3))
                    reply_core_byte = icmp_reply_message_checksum[7:0];
            end
            else if (reply_type == REPLY_BG_ARP) begin
                case (byte_offset)
                    10'd0, 10'd1, 10'd2, 10'd3, 10'd4, 10'd5:
                        reply_core_byte = 8'hFF;
                    10'd6:  reply_core_byte = bg_sender_mac_byte(0);
                    10'd7:  reply_core_byte = bg_sender_mac_byte(1);
                    10'd8:  reply_core_byte = bg_sender_mac_byte(2);
                    10'd9:  reply_core_byte = bg_sender_mac_byte(3);
                    10'd10: reply_core_byte = bg_sender_mac_byte(4);
                    10'd11: reply_core_byte = bg_sender_mac_byte(5);
                    10'd12: reply_core_byte = 8'h08;
                    10'd13: reply_core_byte = 8'h06;
                    10'd14: reply_core_byte = 8'h00;
                    10'd15: reply_core_byte = 8'h01;
                    10'd16: reply_core_byte = 8'h08;
                    10'd17: reply_core_byte = 8'h00;
                    10'd18: reply_core_byte = 8'h06;
                    10'd19: reply_core_byte = 8'h04;
                    10'd20: reply_core_byte = 8'h00;
                    10'd21: reply_core_byte = 8'h01;
                    10'd22: reply_core_byte = bg_sender_mac_byte(0);
                    10'd23: reply_core_byte = bg_sender_mac_byte(1);
                    10'd24: reply_core_byte = bg_sender_mac_byte(2);
                    10'd25: reply_core_byte = bg_sender_mac_byte(3);
                    10'd26: reply_core_byte = bg_sender_mac_byte(4);
                    10'd27: reply_core_byte = bg_sender_mac_byte(5);
                    10'd28: reply_core_byte = bg_sender_ipv4_byte(0);
                    10'd29: reply_core_byte = bg_sender_ipv4_byte(1);
                    10'd30: reply_core_byte = bg_sender_ipv4_byte(2);
                    10'd31: reply_core_byte = bg_sender_ipv4_byte(3);
                    10'd38: reply_core_byte = 8'hC0;
                    10'd39: reply_core_byte = 8'hA8;
                    10'd40: reply_core_byte = subnet_octet;
                    10'd41: reply_core_byte = bg_target_host;
                    default: reply_core_byte = 8'h00;
                endcase
            end
            else if (ENABLE_BG_MDNS_LLMNR &&
                     ((reply_type == REPLY_BG_MDNS) ||
                      (reply_type == REPLY_BG_LLMNR))) begin
                case (byte_offset)
                    10'd0:  reply_core_byte = 8'h01;
                    10'd1:  reply_core_byte = 8'h00;
                    10'd2:  reply_core_byte = 8'h5E;
                    10'd3:  reply_core_byte = 8'h00;
                    10'd4:  reply_core_byte = 8'h00;
                    10'd5:  reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'hFB : 8'hFC;
                    10'd6:  reply_core_byte = bg_sender_mac_byte(0);
                    10'd7:  reply_core_byte = bg_sender_mac_byte(1);
                    10'd8:  reply_core_byte = bg_sender_mac_byte(2);
                    10'd9:  reply_core_byte = bg_sender_mac_byte(3);
                    10'd10: reply_core_byte = bg_sender_mac_byte(4);
                    10'd11: reply_core_byte = bg_sender_mac_byte(5);
                    10'd12: reply_core_byte = 8'h08;
                    10'd13: reply_core_byte = 8'h00;
                    10'd14: reply_core_byte = 8'h45;
                    10'd15: reply_core_byte = 8'h00;
                    10'd16: reply_core_byte = 8'h00;
                    10'd17: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'h38 : 8'h32;
                    10'd18: reply_core_byte = bg_lfsr[15:8];
                    10'd19: reply_core_byte = bg_lfsr[7:0];
                    10'd20, 10'd21: reply_core_byte = 8'h00;
                    10'd22: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'hFF : 8'h01;
                    10'd23: reply_core_byte = 8'h11;
                    10'd24: reply_core_byte = bg_mcast_ip_checksum[15:8];
                    10'd25: reply_core_byte = bg_mcast_ip_checksum[7:0];
                    10'd26: reply_core_byte = bg_sender_ipv4_byte(0);
                    10'd27: reply_core_byte = bg_sender_ipv4_byte(1);
                    10'd28: reply_core_byte = bg_sender_ipv4_byte(2);
                    10'd29: reply_core_byte = bg_sender_ipv4_byte(3);
                    10'd30: reply_core_byte = 8'hE0;
                    10'd31, 10'd32: reply_core_byte = 8'h00;
                    10'd33: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'hFB : 8'hFC;
                    10'd34: reply_core_byte = 8'h14;
                    10'd35: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'hE9 : 8'hEB;
                    10'd36: reply_core_byte = 8'h14;
                    10'd37: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'hE9 : 8'hEB;
                    10'd38: reply_core_byte = 8'h00;
                    10'd39: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'h24 : 8'h1E;
                    10'd40, 10'd41: reply_core_byte = 8'h00;
                    10'd42: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'h00 : bg_lfsr[15:8];
                    10'd43: reply_core_byte = (reply_type == REPLY_BG_MDNS) ?
                                         8'h00 : bg_lfsr[7:0];
                    10'd44, 10'd45: reply_core_byte = 8'h00;
                    10'd46: reply_core_byte = 8'h00;
                    10'd47: reply_core_byte = 8'h01;
                    10'd48, 10'd49, 10'd50, 10'd51, 10'd52, 10'd53:
                        reply_core_byte = 8'h00;
                    10'd54: reply_core_byte = 8'h04;
                    10'd55: reply_core_byte = 8'h77;
                    10'd56: reply_core_byte = 8'h70;
                    10'd57: reply_core_byte = 8'h61;
                    10'd58: reply_core_byte = 8'h64;
                    default: begin
                        if (reply_type == REPLY_BG_MDNS) begin
                            case (byte_offset)
                                10'd59: reply_core_byte = 8'h05;
                                10'd60: reply_core_byte = 8'h6C;
                                10'd61: reply_core_byte = 8'h6F;
                                10'd62: reply_core_byte = 8'h63;
                                10'd63: reply_core_byte = 8'h61;
                                10'd64: reply_core_byte = 8'h6C;
                                10'd65: reply_core_byte = 8'h00;
                                10'd66: reply_core_byte = 8'h00;
                                10'd67: reply_core_byte = 8'h01;
                                10'd68: reply_core_byte = 8'h00;
                                10'd69: reply_core_byte = 8'h01;
                                default: reply_core_byte = 8'h00;
                            endcase
                        end
                        else begin
                            case (byte_offset)
                                10'd59: reply_core_byte = 8'h00;
                                10'd60: reply_core_byte = 8'h00;
                                10'd61: reply_core_byte = 8'h01;
                                10'd62: reply_core_byte = 8'h00;
                                10'd63: reply_core_byte = 8'h01;
                                default: reply_core_byte = 8'h00;
                            endcase
                        end
                    end
                endcase
            end
            else if (reply_type == REPLY_ARP) begin
                case (byte_offset)
                    10'd0:  reply_core_byte = client_mac_byte(0);
                    10'd1:  reply_core_byte = client_mac_byte(1);
                    10'd2:  reply_core_byte = client_mac_byte(2);
                    10'd3:  reply_core_byte = client_mac_byte(3);
                    10'd4:  reply_core_byte = client_mac_byte(4);
                    10'd5:  reply_core_byte = client_mac_byte(5);
                    10'd6:  reply_core_byte = gw_mac_byte(0);
                    10'd7:  reply_core_byte = gw_mac_byte(1);
                    10'd8:  reply_core_byte = gw_mac_byte(2);
                    10'd9:  reply_core_byte = gw_mac_byte(3);
                    10'd10: reply_core_byte = gw_mac_byte(4);
                    10'd11: reply_core_byte = gw_mac_byte(5);
                    10'd12: reply_core_byte = 8'h08;
                    10'd13: reply_core_byte = 8'h06;
                    10'd14: reply_core_byte = 8'h00;
                    10'd15: reply_core_byte = 8'h01;
                    10'd16: reply_core_byte = 8'h08;
                    10'd17: reply_core_byte = 8'h00;
                    10'd18: reply_core_byte = 8'h06;
                    10'd19: reply_core_byte = 8'h04;
                    10'd20: reply_core_byte = 8'h00;
                    10'd21: reply_core_byte = 8'h02;
                    10'd22: reply_core_byte = gw_mac_byte(0);
                    10'd23: reply_core_byte = gw_mac_byte(1);
                    10'd24: reply_core_byte = gw_mac_byte(2);
                    10'd25: reply_core_byte = gw_mac_byte(3);
                    10'd26: reply_core_byte = gw_mac_byte(4);
                    10'd27: reply_core_byte = gw_mac_byte(5);
                    10'd28: reply_core_byte = gw_ipv4_byte(0);
                    10'd29: reply_core_byte = gw_ipv4_byte(1);
                    10'd30: reply_core_byte = gw_ipv4_byte(2);
                    10'd31: reply_core_byte = gw_ipv4_byte(3);
                    10'd32: reply_core_byte = client_mac_byte(0);
                    10'd33: reply_core_byte = client_mac_byte(1);
                    10'd34: reply_core_byte = client_mac_byte(2);
                    10'd35: reply_core_byte = client_mac_byte(3);
                    10'd36: reply_core_byte = client_mac_byte(4);
                    10'd37: reply_core_byte = client_mac_byte(5);
                    10'd38: reply_core_byte = client_ipv4_byte(0);
                    10'd39: reply_core_byte = client_ipv4_byte(1);
                    10'd40: reply_core_byte = client_ipv4_byte(2);
                    10'd41: reply_core_byte = client_ipv4_byte(3);
                    default: reply_core_byte = 8'h00;
                endcase
            end
            else if (reply_type == REPLY_DNS) begin
                case (byte_offset)
                    10'd0:  reply_core_byte = client_mac_byte(0);
                    10'd1:  reply_core_byte = client_mac_byte(1);
                    10'd2:  reply_core_byte = client_mac_byte(2);
                    10'd3:  reply_core_byte = client_mac_byte(3);
                    10'd4:  reply_core_byte = client_mac_byte(4);
                    10'd5:  reply_core_byte = client_mac_byte(5);
                    10'd6:  reply_core_byte = gw_mac_byte(0);
                    10'd7:  reply_core_byte = gw_mac_byte(1);
                    10'd8:  reply_core_byte = gw_mac_byte(2);
                    10'd9, 10'd10: reply_core_byte = gw_mac_byte((byte_offset == 10'd9) ? 3'd3 : 3'd4);
                    10'd11: reply_core_byte = gw_mac_byte(5);
                    10'd12: reply_core_byte = 8'h08;
                    10'd13: reply_core_byte = 8'h00;

                    // IPv4/UDP：虚拟 DNS 服务端 192.168.x.1:53。
                    10'd14: reply_core_byte = 8'h45;
                    10'd15: reply_core_byte = 8'h00;
                    10'd16: reply_core_byte = service_ip_total_length[15:8];
                    10'd17: reply_core_byte = service_ip_total_length[7:0];
                    10'd18: reply_core_byte = reply_ipv4_identification[15:8];
                    10'd19: reply_core_byte = reply_ipv4_identification[7:0];
                    10'd20: reply_core_byte = 8'h40;
                    10'd21: reply_core_byte = 8'h00;
                    10'd22: reply_core_byte = 8'h40;
                    10'd23: reply_core_byte = 8'h11;
                    10'd24: reply_core_byte = service_ip_checksum[15:8];
                    10'd25: reply_core_byte = service_ip_checksum[7:0];
                    10'd26: reply_core_byte = service_ipv4_byte(0);
                    10'd27: reply_core_byte = service_ipv4_byte(1);
                    10'd28: reply_core_byte = service_ipv4_byte(2);
                    10'd29: reply_core_byte = service_ipv4_byte(3);
                    10'd30: reply_core_byte = client_ipv4_byte(0);
                    10'd31: reply_core_byte = client_ipv4_byte(1);
                    10'd32: reply_core_byte = client_ipv4_byte(2);
                    10'd33: reply_core_byte = client_ipv4_byte(3);
                    10'd34: reply_core_byte = 8'h00;
                    10'd35: reply_core_byte = 8'h35;
                    10'd36: reply_core_byte = frame_udp_ports[31:24];
                    10'd37: reply_core_byte = frame_udp_ports[23:16];
                    10'd38: reply_core_byte = service_udp_length[15:8];
                    10'd39: reply_core_byte = service_udp_length[7:0];
                    10'd40, 10'd41: reply_core_byte = 8'h00;

                    // DNS header：复制事务号，标准递归应答。万能解析 1 条 A=网关；
                    // dns.msftncsi.com 为 5 条 A，首条 131.107.255.255。
                    10'd42, 10'd43:
                        reply_core_byte =
                            icmp_request_bytes[(byte_offset + reply_vlan_bytes) & 7'h7F];
                    10'd44: reply_core_byte = {7'b1000000,frame_dns_flags[8]};
                    10'd45: reply_core_byte = {4'h0,dns_rcode};
                    10'd46: reply_core_byte = 8'h00;
                    10'd47: reply_core_byte = 8'h01;
                    10'd48: reply_core_byte = 8'h00;
                    10'd49: reply_core_byte = {4'h0,dns_answer_count};
                    10'd50, 10'd51, 10'd52, 10'd53:
                        reply_core_byte = 8'h00;
                    default: begin
                        if (byte_offset < dns_question_end)
                            reply_core_byte =
                                icmp_request_bytes[(byte_offset + reply_vlan_bytes) & 7'h7F];
                        else begin
                            dns_ans_off = byte_offset - dns_question_end;
                            dns_a_ipv4 = dns_answer_ipv4(dns_ans_off[7:4]);
                            case (dns_ans_off[3:0])
                                4'd0:  reply_core_byte = 8'hC0;
                                4'd1:  reply_core_byte = 8'h0C;
                                4'd2:  reply_core_byte = 8'h00;
                                4'd3:  reply_core_byte = 8'h01;
                                4'd4:  reply_core_byte = 8'h00;
                                4'd5:  reply_core_byte = 8'h01;
                                4'd6, 4'd7, 4'd8:
                                    reply_core_byte = 8'h00;
                                4'd9:  reply_core_byte = 8'h3C;
                                4'd10: reply_core_byte = 8'h00;
                                4'd11: reply_core_byte = 8'h04;
                                4'd12: reply_core_byte = dns_a_ipv4[31:24];
                                4'd13: reply_core_byte = dns_a_ipv4[23:16];
                                4'd14: reply_core_byte = dns_a_ipv4[15:8];
                                4'd15: reply_core_byte = dns_a_ipv4[7:0];
                                default: reply_core_byte = 8'h00;
                            endcase
                        end
                    end
                endcase
            end
            else if (ENABLE_TCP_HTTP &&
                     ((reply_type == REPLY_TCP_SYNACK) ||
                       (reply_type == REPLY_HTTP) || (reply_type == REPLY_TCP_FIN) || (reply_type == REPLY_TCP_ACK) || (reply_type == REPLY_TCP_RST))) begin
                case (byte_offset)
                    10'd0:  reply_core_byte = client_mac_byte(0);
                    10'd1:  reply_core_byte = client_mac_byte(1);
                    10'd2:  reply_core_byte = client_mac_byte(2);
                    10'd3:  reply_core_byte = client_mac_byte(3);
                    10'd4:  reply_core_byte = client_mac_byte(4);
                    10'd5:  reply_core_byte = client_mac_byte(5);
                    10'd6:  reply_core_byte = gw_mac_byte(0);
                    10'd7:  reply_core_byte = gw_mac_byte(1);
                    10'd8:  reply_core_byte = gw_mac_byte(2);
                    10'd9, 10'd10: reply_core_byte = gw_mac_byte((byte_offset == 10'd9) ? 3'd3 : 3'd4);
                    10'd11: reply_core_byte = gw_mac_byte(5);
                    10'd12: reply_core_byte = 8'h08;
                    10'd13: reply_core_byte = 8'h00;

                    // IPv4/TCP：只对主机实际提交的 TCP/80 请求产生应答。
                    10'd14: reply_core_byte = 8'h45;
                    10'd15: reply_core_byte = 8'h00;
                    10'd16: reply_core_byte = service_ip_total_length[15:8];
                    10'd17: reply_core_byte = service_ip_total_length[7:0];
                    10'd18: reply_core_byte = reply_ipv4_identification[15:8];
                    10'd19: reply_core_byte = reply_ipv4_identification[7:0];
                    10'd20: reply_core_byte = 8'h40;
                    10'd21: reply_core_byte = 8'h00;
                    10'd22: reply_core_byte = 8'h40;
                    10'd23: reply_core_byte = 8'h06;
                    10'd24: reply_core_byte = service_ip_checksum[15:8];
                    10'd25: reply_core_byte = service_ip_checksum[7:0];
                    10'd26: reply_core_byte = service_ipv4_byte(0);
                    10'd27: reply_core_byte = service_ipv4_byte(1);
                    10'd28: reply_core_byte = service_ipv4_byte(2);
                    10'd29: reply_core_byte = service_ipv4_byte(3);
                    10'd30: reply_core_byte = client_ipv4_byte(0);
                    10'd31: reply_core_byte = client_ipv4_byte(1);
                    10'd32: reply_core_byte = client_ipv4_byte(2);
                    10'd33: reply_core_byte = client_ipv4_byte(3);

                    // TCP：SYN-ACK 或携带 NCSI HTTP 负载的 FIN/PSH/ACK。
                    10'd34: reply_core_byte = 8'h00;
                    10'd35: reply_core_byte = 8'h50;
                    10'd36: reply_core_byte = tcp_client_port[15:8];
                    10'd37: reply_core_byte = tcp_client_port[7:0];
                    10'd38: reply_core_byte = tcp_reply_sequence[31:24];
                    10'd39: reply_core_byte = tcp_reply_sequence[23:16];
                    10'd40: reply_core_byte = tcp_reply_sequence[15:8];
                    10'd41: reply_core_byte = tcp_reply_sequence[7:0];
                    10'd42: reply_core_byte = tcp_reply_acknowledgment[31:24];
                    10'd43: reply_core_byte = tcp_reply_acknowledgment[23:16];
                    10'd44: reply_core_byte = tcp_reply_acknowledgment[15:8];
                    10'd45: reply_core_byte = tcp_reply_acknowledgment[7:0];
                    10'd46: reply_core_byte = 8'h50;
                    10'd47: reply_core_byte = service_tcp_flags;
                    10'd48: reply_core_byte = (reply_type == REPLY_TCP_RST) ? 8'h00 : 8'h40;
                    10'd49: reply_core_byte = 8'h00;
                    10'd50: reply_core_byte = service_tcp_checksum[15:8];
                    10'd51: reply_core_byte = service_tcp_checksum[7:0];
                    10'd52, 10'd53: reply_core_byte = 8'h00;
                    default: begin
                        if ((reply_type == REPLY_HTTP) &&
                            (byte_offset >= 10'd54) &&
                            (byte_offset <
                             (10'd54 + http_reply_bytes)))
                            reply_core_byte = http_legacy_reply ? http_legacy_payload_byte(byte_offset-10'd54) :
                                http_payload_byte(byte_offset-10'd54);
                    end
                endcase
            end
            else begin
                case (byte_offset)
                    // L2 跟 Discover flag：广播则全 F，否则 chaddr。
                    10'd0, 10'd1, 10'd2, 10'd3, 10'd4, 10'd5:
                        reply_core_byte = ((reply_type == REPLY_DHCP_NAK) || ((dhcp_ciaddr == 0) && dhcp_flags[15])) ?
                            8'hFF : client_mac_byte(byte_offset[2:0]);
                    10'd6:  reply_core_byte = gw_mac_byte(0);
                    10'd7:  reply_core_byte = gw_mac_byte(1);
                    10'd8:  reply_core_byte = gw_mac_byte(2);
                    10'd9:  reply_core_byte = gw_mac_byte(3);
                    10'd10: reply_core_byte = gw_mac_byte(4);
                    10'd11: reply_core_byte = gw_mac_byte(5);
                    10'd12: reply_core_byte = 8'h08;
                    10'd13: reply_core_byte = 8'h00;

                    // IPv4：总长 0x0154（340），DF，源.1。
                    10'd14: reply_core_byte = 8'h45;
                    10'd15: reply_core_byte = 8'h00;
                    10'd16: reply_core_byte = (reply_core_length-16'd14) >> 8;
                    10'd17: reply_core_byte = reply_core_length-16'd14;
                    10'd18, 10'd19: reply_core_byte = 8'h00;
                    10'd20: reply_core_byte = 8'h40;
                    10'd21: reply_core_byte = 8'h00;
                    10'd22: reply_core_byte = 8'h40;
                    10'd23: reply_core_byte = 8'h11;
                    10'd24: reply_core_byte = dhcp_ip_checksum[15:8];
                    10'd25: reply_core_byte = dhcp_ip_checksum[7:0];
                    10'd26: reply_core_byte = gw_ipv4_byte(0);
                    10'd27: reply_core_byte = gw_ipv4_byte(1);
                    10'd28: reply_core_byte = gw_ipv4_byte(2);
                    10'd29: reply_core_byte = gw_ipv4_byte(3);
                    10'd30: reply_core_byte = dhcp_dest_ipv4[31:24];
                    10'd31: reply_core_byte = dhcp_dest_ipv4[23:16];
                    10'd32: reply_core_byte = dhcp_dest_ipv4[15:8];
                    10'd33: reply_core_byte = dhcp_dest_ipv4[7:0];

                    // UDP：67 -> 68，长度 0x0140（320）。
                    10'd34: reply_core_byte = 8'h00;
                    10'd35: reply_core_byte = 8'h43;
                    10'd36: reply_core_byte = 8'h00;
                    10'd37: reply_core_byte = 8'h44;
                    10'd38: reply_core_byte = (reply_core_length-16'd34) >> 8;
                    10'd39: reply_core_byte = reply_core_length-16'd34;
                    10'd40: reply_core_byte = dhcp_udp_csum[15:8];
                    10'd41: reply_core_byte = dhcp_udp_csum[7:0];

                    10'd42: reply_core_byte = 8'h02;
                    10'd43: reply_core_byte = 8'h01;
                    10'd44: reply_core_byte = 8'h06;
                    10'd45: reply_core_byte = 8'h00;
                    10'd46: reply_core_byte = xid_byte(0);
                    10'd47: reply_core_byte = xid_byte(1);
                    10'd48: reply_core_byte = xid_byte(2);
                    10'd49: reply_core_byte = xid_byte(3);
                    10'd50, 10'd51:
                        reply_core_byte = 8'h00;
                    10'd52: reply_core_byte = dhcp_bootp_flags[15:8];
                    10'd53: reply_core_byte = dhcp_bootp_flags[7:0];
                    10'd54: reply_core_byte=dhcp_ciaddr[31:24];
                    10'd55: reply_core_byte=dhcp_ciaddr[23:16];
                    10'd56: reply_core_byte=dhcp_ciaddr[15:8];
                    10'd57: reply_core_byte=dhcp_ciaddr[7:0];
                    10'd58: reply_core_byte = (reply_type == REPLY_DHCP_NAK) ? 0 : lease_ipv4[31:24];
                    10'd59: reply_core_byte = (reply_type == REPLY_DHCP_NAK) ? 0 : lease_ipv4[23:16];
                    10'd60: reply_core_byte = (reply_type == REPLY_DHCP_NAK) ? 0 : lease_ipv4[15:8];
                    10'd61: reply_core_byte = (reply_type == REPLY_DHCP_NAK) ? 0 : lease_ipv4[7:0];
                    // 6801/Marvell：siaddr 填网关，不留 0。
                    10'd62: reply_core_byte = (reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(0);
                    10'd63: reply_core_byte = (reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(1);
                    10'd64: reply_core_byte = (reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(2);
                    10'd65: reply_core_byte = (reply_type==REPLY_DHCP_NAK) ? 0 : gw_ipv4_byte(3);
                    10'd66, 10'd67, 10'd68, 10'd69:
                        reply_core_byte = 8'h00;
                    10'd70: reply_core_byte = client_mac_byte(0);
                    10'd71: reply_core_byte = client_mac_byte(1);
                    10'd72: reply_core_byte = client_mac_byte(2);
                    10'd73: reply_core_byte = client_mac_byte(3);
                    10'd74: reply_core_byte = client_mac_byte(4);
                    10'd75: reply_core_byte = client_mac_byte(5);

                    // DHCP magic cookie。
                    10'd278: reply_core_byte = 8'h63;
                    10'd279: reply_core_byte = 8'h82;
                    10'd280: reply_core_byte = 8'h53;
                    10'd281: reply_core_byte = 8'h63;

                    default: begin
                        if (byte_offset >= 10'd282)
                            reply_core_byte=dhcp_option_byte(byte_offset-10'd282);
                    end
                endcase
            end
        end
    endfunction

    // Preserve tags on replies. No silent stripping without RX VLAN metadata.
    function automatic [7:0] reply_byte(input [9:0] byte_offset);
        begin
            if (!reply_vlan || byte_offset < 12)
                reply_byte=reply_core_byte(byte_offset);
            else case (byte_offset)
                12: reply_byte=reply_vlan_tpid[15:8];
                13: reply_byte=reply_vlan_tpid[7:0];
                14: reply_byte=reply_vlan_tci[15:8];
                15: reply_byte=reply_vlan_tci[7:0];
                default: reply_byte=reply_core_byte(byte_offset-10'd4);
            endcase
        end
    endfunction

    wire [31:0] dhcp_checksum_next_sum = dhcp_checksum_sum +
        (dhcp_checksum_index[0] ? {24'h0,dhcp_checksum_byte_q} :
         {16'h0,dhcp_checksum_byte_q,8'h0});
    reg [63:0] rx_buffer_addr;
    reg [15:0] rx_buffer_capacity;
    reg [15:0] rx_buffer_index;
    reg [31:0] rx_buffer_opaque;
    reg [15:0] tx_cons_idx;
    reg [15:0] rx_std_cons_idx;
    reg [15:0] rx_ret_prod_idx;
    reg [7:0]  status_tag;
    reg [7:0]  committed_status_tag;
    reg        link_status_pending;
    reg        status_link_change;
    // BAR 中断邮箱只能与主机已经可见的状态 tag 比较。
    // 正在构造或发送的状态块尚未提交，不能提前触发 tagged rearm。
    assign status_tag_value = committed_status_tag;

    // Linux tg3.h + b57nd60a 0x14005405e/0x140056ea0：
    // flags 在 type_flags[15:0]；IP 在 ip_tcp_csum[31:16]（主机 LE +0x12），
    // TCP/UDP 在 [15:0]（+0x10）。成功值是 0xFFFF。
    // 5751 只在 RXD_FLAG_IP_CSUM 置位时把 BD+0x12 抄到软描述符 +0x40；
    // 随后 mapper：offload 开且 +0x40!=0xFFFF → NDIS IpChecksumFailed=0x4。
    // 旧 RTL 不置 0x1000、csum 写 0，PktMon 因此报 0x4。自产帧校验已算对，
    // IPv4 必须置 0x1000 并写 IP=0xFFFF；L4 再置 0x2000 并写 0xFFFF。
    localparam [15:0] RXD_FLAG_END         = 16'h0004;
    localparam [15:0] RXD_FLAG_IP_CSUM     = 16'h1000;
    localparam [15:0] RXD_FLAG_TCPUDP_CSUM = 16'h2000;
    wire rxd_ipv4 =
        (reply_type == REPLY_DHCP_OFFER) ||
        (reply_type == REPLY_DHCP_ACK) ||
        (reply_type == REPLY_DHCP_NAK) ||
        (reply_type == REPLY_ICMP) ||
        (reply_type == REPLY_DNS) ||
        (reply_type == REPLY_HTTP) ||
        (reply_type == REPLY_TCP_SYNACK) ||
        (reply_type == REPLY_TCP_FIN) || (reply_type == REPLY_TCP_ACK) || (reply_type == REPLY_TCP_RST) ||
        (reply_type == REPLY_BG_MDNS) ||
        (reply_type == REPLY_BG_LLMNR);
    wire rxd_l4 =
        (reply_type == REPLY_DHCP_OFFER) ||
        (reply_type == REPLY_DHCP_ACK) ||
        (reply_type == REPLY_DHCP_NAK) ||
        (reply_type == REPLY_DNS) ||
        (reply_type == REPLY_HTTP) ||
        (reply_type == REPLY_TCP_SYNACK) ||
        (reply_type == REPLY_TCP_FIN) || (reply_type == REPLY_TCP_ACK) || (reply_type == REPLY_TCP_RST) ||
        (reply_type == REPLY_BG_MDNS) ||
        (reply_type == REPLY_BG_LLMNR);
    wire [15:0] rxd_flags =
        RXD_FLAG_END |
        (rxd_ipv4 ? RXD_FLAG_IP_CSUM : 16'h0000) |
        (rxd_l4 ? RXD_FLAG_TCPUDP_CSUM : 16'h0000);
    wire [31:0] rxd_ip_tcp_csum =
        rxd_l4 ? 32'hFFFFFFFF :
        (rxd_ipv4 ? 32'hFFFF0000 : 32'h00000000);

    function automatic [31:0] mwr_host_dword(
        input [1:0] write_kind,
        input [9:0] word_index
    );
        begin
            mwr_host_dword = 32'h00000000;
            case (write_kind)
                // WR_REPLY 在 mwr_source_byte 中直接按字节生成，避免复制第二套回复路径。
                WR_RX_DESC: begin
                    case (word_index[2:0])
                        3'd0: mwr_host_dword = rx_buffer_addr[63:32];
                        3'd1: mwr_host_dword = rx_buffer_addr[31:0];
                        // Broadcom 571X PG：Index 由 producer 原样传到 return
                        // ring；Opaque 只在 dword7。不得用 opaque[15:0] 顶替。
                        3'd2: mwr_host_dword = {
                            rx_buffer_index,
                            reply_frame_length + 16'd4
                        };
                        3'd3: mwr_host_dword = {16'h0000, rxd_flags};
                        3'd4: mwr_host_dword = rxd_ip_tcp_csum;
                        3'd5: mwr_host_dword = 32'h00000000;
                        3'd6: mwr_host_dword = 32'h00000000;
                        default: mwr_host_dword = rx_buffer_opaque;
                    endcase
                end
                WR_STATUS: begin
                    case (word_index[2:0])
                        // bit0=UPDATED，bit1=LINK_CHG。
                        3'd0: mwr_host_dword = {
                            30'h00000000, status_link_change, 1'b1
                        };
                        3'd1: mwr_host_dword = {24'h000000, status_tag};
                        // Broadcom 571X PG Table 14：+0x08[31:16] 是 Standard
                        // RX Producer Consumer；+0x08[15:0] 保留 0。
                        3'd2: mwr_host_dword = {
                            rx_std_cons_idx,
                            16'h0000
                        };
                        3'd3: mwr_host_dword = 32'h00000000;
                        3'd4: mwr_host_dword = {tx_cons_idx, rx_ret_prod_idx};
                        default: mwr_host_dword = 32'h00000000;
                    endcase
                end
                default: mwr_host_dword = 32'h00000000;
            endcase
        end
    endfunction

    // --------------------------------------------------------------------
    // 非标准 AXIS 脉冲源。
    // 128->64 适配器要求先以 has_data 取得 tready，再只脉冲一次 tvalid。
    // --------------------------------------------------------------------
    reg [127:0] queued_tdata;
    reg [3:0]   queued_tkeepdw;
    reg         queued_tlast;
    reg [8:0]   queued_tuser;
    reg         queued_starts_mwr;
    reg         out_pending;
    reg         out_sent;
    reg         mrd_tlp_open;
    reg         mwr_tlp_open;
    reg         mrd_tx_done_latched;
    reg         mwr_tx_done_latched;

    reg [127:0] out_tdata;
    reg [3:0]   out_tkeepdw;
    reg         out_tlast;
    reg [8:0]   out_tuser;
    reg         out_tvalid;

    assign tlps_out.tdata    = out_tdata;
    assign tlps_out.tkeepdw  = out_tkeepdw;
    assign tlps_out.tlast    = out_tlast;
    assign tlps_out.tuser    = out_tuser;
    assign tlps_out.tvalid   = out_tvalid;
    // 三种发送生命周期：
    //   queued   : out_pending=1, 尚未拿到 tready, *_tlp_open=0
    //   granted  : 本拍已被 mux 用 tready 取走
    //   open     : 该 TLP 已上 mux，必须发到本源 tlast
    // 禁用后只允许 open 包继续对 mux 声明 has_data；queued 由下面的
    // discard 丢掉，否则 mux 永远不给 tready、teardown 又等 pending=0。
    wire tlp_open = mrd_tlp_open || mwr_tlp_open;
    wire queued_ungranted =
        out_pending && !tlp_open && !out_tvalid;
    assign tlps_out.has_data =
        out_pending &&
        (dma_runtime_ready || tlp_open);

    // --------------------------------------------------------------------
    // 单 Outstanding Completion 接收器。
    // --------------------------------------------------------------------
    reg [1:0]   mrd_kind;
    reg [10:0]  mrd_store_base_byte;
    reg [9:0]   mrd_expected_words;
    reg [15:0]  mrd_requested_bytes;
    reg [1:0]   mrd_leading_bytes;
    reg [63:0]  mrd_addr;
    reg [1:0]   mrd_pending_kind;
    reg [10:0]  mrd_pending_store_base_byte;
    reg [9:0]   mrd_pending_expected_words;
    reg [15:0]  mrd_pending_requested_bytes;
    reg [1:0]   mrd_pending_leading_bytes;
    reg [63:0]  mrd_pending_addr;

    reg [7:0] state;
    reg [7:0] after_mrd_state;
    reg [7:0] after_mwr_state;
    // mux 授权只表示拍已离开本源；包尾必须等硬核真正收下对应来源
    // 的 last。全局 tx_packet_done 可能属于 CFG/BAR/USB，要用 src token。
    localparam [2:0] SRC_TG3 = 3'd4;
    wire beat_grant =
        out_pending && tlps_out.tready &&
        (dma_runtime_ready || tlp_open);
    wire tg3_core_packet_done =
        tx_packet_done && (tx_packet_src == SRC_TG3);
    wire mrd_tx_done_event =
        mrd_tlp_open && tg3_core_packet_done;
    wire mwr_tx_done_event =
        mwr_tlp_open && tg3_core_packet_done;
    // Completion 接收器只在硬核收下本源 MRd 包尾后武装。
    wire mrd_request_accepted = mrd_tx_done_event;

    reg         cpl_active;
    reg         cpl_packet_match;
    reg [1:0]   cpl_kind;
    reg [10:0]  cpl_store_base_byte;
    reg [9:0]   cpl_expected_words;
    reg [15:0]  cpl_requested_bytes;
    reg [1:0]   cpl_leading_bytes;
    reg [6:0]   cpl_request_lower_addr;
    reg [9:0]   cpl_received_words;
    reg [19:0]  cpl_timeout;
    reg         cpl_done;
    reg         cpl_error;
    reg [2:0]   cpl_error_reason;
    reg         dma_fault_latched;

    assign completion_pending = cpl_active;
    // 固定 tag 0x1E：上一笔 MRd 的 CPL 没回来之前绝不能再发。
    // 禁用进 D3 后完成包可能永远不来；启用时复用 tag 会把主机冻死。
    // dma_fault_latched 是 tag 隔离：超时后跨 teardown / NOW 保持，
    // 冷却到期前禁止再发同一 tag。
    wire dma_may_issue_mrd =
        dma_runtime_ready && !cpl_active && !dma_fault_latched;
    // 准备状态也参与预约，避免外部DMA连续复用Tag 0x1F时内部永久饿死。
    assign read_channel_lock =
        mrd_tlp_open || cpl_active ||
        (state == S_TX_DESC_REQUEST) ||
        (state == S_RX_DESC_REQUEST) ||
        (state == S_TX_DATA_PREP) ||
        (state == S_MRD_QUEUE) ||
        (state == S_MRD_WAIT_SENT) ||
        (state == S_MRD_WAIT_TX_DONE) ||
        (state == S_MRD_WAIT_CPL);
    // 软复位只在内部事务和 64 位发送器均静止后完成。
    assign dma_quiescent =
        ((state == S_WAIT_CONFIG) || (state == S_IDLE)) &&
        !cpl_active && !tlp_open &&
        !out_pending && !out_tvalid && dma_tx_idle;

    // 软复位请求(lifecycle_reset_req)是单拍脉冲，与正在发送的 TLP 完全
    // 异步。若立即整体复位会把半个包留在发送通路上：下游 mux 授权后锁定
    // 到 tlast，被复位的引擎永远不再发出 tlast，配置/BAR 完成包会全部
    // 堵在 mux 后面（主机 MMIO 读超时 → 整机冻结）。因此这里只锁存请求
    // 并立即解除数据面武装，状态机沿既有 abort 路径收尾（已打开的 MWr
    // 保证完整发完），到达包边界安全点后才执行真正的同步复位。
    bit  reset_pending;
    wire dma_reset_safe =
        ((state == S_WAIT_CONFIG) || (state == S_IDLE)) &&
        !cpl_active && !tlp_open &&
        !out_pending && !out_tvalid && dma_tx_idle;
    wire reset_apply = rst || (reset_pending && dma_reset_safe);

    wire cpl_is_completion =
        (tlps_cpl_in.tdata[31:25] == 7'b0000101) ||
        (tlps_cpl_in.tdata[31:25] == 7'b0100101);
    wire cpl_has_data =
        (tlps_cpl_in.tdata[31:25] == 7'b0100101);
    wire [15:0] cpl_requester_id =
        {pcie_id[7:0], pcie_id[15:8]};
    wire cpl_first_completion =
        tlps_cpl_in.tvalid && tlps_cpl_in.tuser[0] &&
        cpl_active && cpl_is_completion;
    wire cpl_requester_matches =
        (tlps_cpl_in.tdata[95:80] == cpl_requester_id);
    wire cpl_tag_matches =
        (tlps_cpl_in.tdata[79:72] == DMA_TAG);
    wire cpl_first_addressed =
        cpl_first_completion &&
        cpl_requester_matches && cpl_tag_matches;
    wire [15:0] cpl_valid_bytes_received =
        ({4'h0, cpl_received_words, 2'b00} >=
         {14'h0000, cpl_leading_bytes}) ?
            ({4'h0, cpl_received_words, 2'b00} -
             {14'h0000, cpl_leading_bytes}) : 16'h0000;
    wire [15:0] cpl_bytes_remaining =
        cpl_requested_bytes - cpl_valid_bytes_received;
    wire [6:0] cpl_expected_lower_addr =
        cpl_request_lower_addr + cpl_valid_bytes_received[6:0];
    wire cpl_header_identity_matches =
        (tlps_cpl_in.tdata[43:32] == cpl_bytes_remaining[11:0]) &&
        (tlps_cpl_in.tdata[70:64] == cpl_expected_lower_addr);
    wire cpl_first_owned =
        cpl_first_addressed && cpl_header_identity_matches;
    wire cpl_data_length_plausible =
        (tlps_cpl_in.tdata[9:0] != 0) &&
        (tlps_cpl_in.tdata[9:0] <=
         (cpl_expected_words - cpl_received_words));
    wire cpl_first_match =
        cpl_first_owned &&
        cpl_has_data && cpl_data_length_plausible &&
        (tlps_cpl_in.tdata[47:45] == 3'b000) &&
        tlps_cpl_in.tkeepdw[3];
    // 内部MRd的Completion不得再送给DMA软件；后续拍由首拍认领状态保持。
    assign completion_claim = tlps_cpl_in.tvalid &&
        ((tlps_cpl_in.tuser[0] && cpl_first_owned) ||
         (!tlps_cpl_in.tuser[0] && cpl_packet_match));
    wire [2:0] cpl_cont_word_count =
        {2'b00, tlps_cpl_in.tkeepdw[0]} +
        {2'b00, tlps_cpl_in.tkeepdw[1]} +
        {2'b00, tlps_cpl_in.tkeepdw[2]} +
        {2'b00, tlps_cpl_in.tkeepdw[3]};
    wire [2:0] cpl_word_count =
        tlps_cpl_in.tuser[0] ?
            ((cpl_has_data && tlps_cpl_in.tkeepdw[3]) ? 3'd1 : 3'd0) :
            cpl_cont_word_count;

    // 关键字段由固定 case 直接锁存，不再从帧缓存建立额外读口。
    task automatic store_tx_packet_fields(
        input [10:0] byte_offset,
        input [7:0]  byte_value
    );
        reg [10:0] dns_qname_idx;
        begin
            dns_qname_idx = 11'd0;
            case (byte_offset)
                11'd0:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[0] <= byte_value;
                11'd1:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[1] <= byte_value;
                11'd2:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[2] <= byte_value;
                11'd3:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[3] <= byte_value;
                11'd4:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[4] <= byte_value;
                11'd5:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[5] <= byte_value;
                11'd6:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[6] <= byte_value;
                11'd7:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[7] <= byte_value;
                11'd8:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[8] <= byte_value;
                11'd9:  if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[9] <= byte_value;
                11'd10: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[10] <= byte_value;
                11'd11: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                            dbg_packet_bytes[11] <= byte_value;
                11'd12: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[12] <= byte_value;
                    frame_ethertype[15:8] <= byte_value;
                end
                11'd13: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[13] <= byte_value;
                    frame_ethertype[7:0] <= byte_value;
                end
                11'd14: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[14] <= byte_value;
                11'd15: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[15] <= byte_value;
                11'd16: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[16] <= byte_value;
                11'd17: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[17] <= byte_value;
                11'd18: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[18] <= byte_value;
                11'd19: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[19] <= byte_value;
                11'd20: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[20] <= byte_value;
                    frame_arp_opcode[15:8] <= byte_value;
                end
                11'd21: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[21] <= byte_value;
                    frame_arp_opcode[7:0] <= byte_value;
                end
                11'd22: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[22] <= byte_value;
                    frame_arp_sender_mac[47:40] <= byte_value;
                end
                11'd23: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[23] <= byte_value;
                    frame_arp_sender_mac[39:32] <= byte_value;
                    frame_ip_protocol <= byte_value;
                end
                11'd24: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[24] <= byte_value;
                    frame_arp_sender_mac[31:24] <= byte_value;
                end
                11'd25: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[25] <= byte_value;
                    frame_arp_sender_mac[23:16] <= byte_value;
                end
                11'd26: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[26] <= byte_value;
                    frame_arp_sender_mac[15:8] <= byte_value;
                end
                11'd27: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[27] <= byte_value;
                    frame_arp_sender_mac[7:0] <= byte_value;
                end
                11'd28: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[28] <= byte_value;
                    frame_arp_sender_ip[31:24] <= byte_value;
                end
                11'd29: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[29] <= byte_value;
                    frame_arp_sender_ip[23:16] <= byte_value;
                end
                11'd30: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[30] <= byte_value;
                    frame_arp_sender_ip[15:8] <= byte_value;
                end
                11'd31: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[31] <= byte_value;
                    frame_arp_sender_ip[7:0] <= byte_value;
                end
                11'd32: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[32] <= byte_value;
                11'd33: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[33] <= byte_value;
                11'd34: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[34] <= byte_value;
                    frame_udp_ports[31:24] <= byte_value;
                end
                11'd35: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[35] <= byte_value;
                    frame_udp_ports[23:16] <= byte_value;
                end
                11'd36: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[36] <= byte_value;
                    frame_udp_ports[15:8] <= byte_value;
                end
                11'd37: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[37] <= byte_value;
                    frame_udp_ports[7:0] <= byte_value;
                end
                11'd38: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[38] <= byte_value;
                    frame_arp_target_ip[31:24] <= byte_value;
                end
                11'd39: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[39] <= byte_value;
                    frame_arp_target_ip[23:16] <= byte_value;
                end
                11'd40: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[40] <= byte_value;
                    frame_arp_target_ip[15:8] <= byte_value;
                end
                11'd41: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[41] <= byte_value;
                    frame_arp_target_ip[7:0] <= byte_value;
                end
                11'd42: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[42] <= byte_value;
                11'd43: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[43] <= byte_value;
                11'd44: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[44] <= byte_value;
                11'd45: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                             dbg_packet_bytes[45] <= byte_value;
                11'd46: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[46] <= byte_value;
                    frame_dhcp_xid[31:24] <= byte_value;
                end
                11'd47: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[47] <= byte_value;
                    frame_dhcp_xid[23:16] <= byte_value;
                end
                11'd48: frame_dhcp_xid[15:8] <= byte_value;
                11'd49: frame_dhcp_xid[7:0] <= byte_value;
                11'd52: frame_dhcp_flags[15:8] <= byte_value;
                11'd53: frame_dhcp_flags[7:0] <= byte_value;
                11'd70: frame_dhcp_chaddr[47:40] <= byte_value;
                11'd71: frame_dhcp_chaddr[39:32] <= byte_value;
                11'd72: frame_dhcp_chaddr[31:24] <= byte_value;
                11'd73: frame_dhcp_chaddr[23:16] <= byte_value;
                11'd74: frame_dhcp_chaddr[15:8] <= byte_value;
                11'd75: frame_dhcp_chaddr[7:0] <= byte_value;
                11'd278: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[48] <= byte_value;
                    frame_dhcp_cookie[31:24] <= byte_value;
                end
                11'd279: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[49] <= byte_value;
                    frame_dhcp_cookie[23:16] <= byte_value;
                end
                11'd280: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[50] <= byte_value;
                    frame_dhcp_cookie[15:8] <= byte_value;
                end
                11'd281: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[51] <= byte_value;
                    frame_dhcp_cookie[7:0] <= byte_value;
                end
                11'd282: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[52] <= byte_value;
                    frame_dhcp_message_option[23:16] <= byte_value;
                end
                11'd283: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[53] <= byte_value;
                    frame_dhcp_message_option[15:8] <= byte_value;
                end
                11'd284: begin
                    if (!PRODUCTION && !dbg_packet_candidate_frozen)
                        dbg_packet_bytes[54] <= byte_value;
                    frame_dhcp_message_option[7:0] <= byte_value;
                end
                11'd285: if (!PRODUCTION && !dbg_packet_candidate_frozen)
                               dbg_packet_bytes[55] <= byte_value;
                default: begin end
            endcase

            if (byte_offset == 11'd6)
                frame_src_mac[47:40] <= byte_value;
            if (byte_offset == 11'd7)
                frame_src_mac[39:32] <= byte_value;
            if (byte_offset == 11'd8)
                frame_src_mac[31:24] <= byte_value;
            if (byte_offset == 11'd9)
                frame_src_mac[23:16] <= byte_value;
            if (byte_offset == 11'd10)
                frame_src_mac[15:8] <= byte_value;
            if (byte_offset == 11'd11)
                frame_src_mac[7:0] <= byte_value;

            // 动态协议偏移解析。固定偏移赋值继续保留以兼容既有网表和
            // 诊断快照；动态赋值在 VLAN/IHL 扩展帧的后续字节覆盖它们。
            case (byte_offset)
                11'd0: frame_dst_mac[47:40] <= byte_value;
                11'd1: frame_dst_mac[39:32] <= byte_value;
                11'd2: frame_dst_mac[31:24] <= byte_value;
                11'd3: frame_dst_mac[23:16] <= byte_value;
                11'd4: frame_dst_mac[15:8]  <= byte_value;
                11'd5: frame_dst_mac[7:0]   <= byte_value;
                11'd12: frame_outer_ethertype[15:8] <= byte_value;
                11'd13: begin
                    frame_outer_ethertype[7:0] <= byte_value;
                    if (({frame_outer_ethertype[15:8], byte_value} ==
                         16'h8100) ||
                        ({frame_outer_ethertype[15:8], byte_value} ==
                         16'h88A8)) begin
                        frame_vlan_tagged <= 1'b1;
                        frame_l3_offset <= 11'd18;
                    end
                end
                default: begin end
            endcase

            if (frame_vlan_tagged && (byte_offset == 11'd16))
                frame_ethertype[15:8] <= byte_value;
            if (frame_vlan_tagged && (byte_offset == 11'd17))
                frame_ethertype[7:0] <= byte_value;

            if (byte_offset == 11'd13)
                frame_vlan_tpid <= {frame_outer_ethertype[15:8], byte_value};
            if (frame_vlan_tagged && byte_offset == 11'd14)
                frame_vlan_tci[15:8] <= byte_value;
            if (frame_vlan_tagged && byte_offset == 11'd15)
                frame_vlan_tci[7:0] <= byte_value;

            // IPv4版本/IHL决定UDP和BOOTP的真实起点。
            if (byte_offset == frame_l3_offset) begin
                frame_ipv4_header_valid <=
                    (byte_value[7:4] == 4'h4) &&
                    (byte_value[3:0] >= 4'd5);
                if ((byte_value[7:4] == 4'h4) &&
                    (byte_value[3:0] >= 4'd5)) begin
                    frame_l4_offset <= frame_l3_offset +
                        {5'h00, byte_value[3:0], 2'b00};
                    frame_bootp_offset <= frame_l3_offset +
                        {5'h00, byte_value[3:0], 2'b00} + 11'd8;
                    frame_dhcp_options_offset <= frame_l3_offset +
                        {5'h00, byte_value[3:0], 2'b00} + 11'd248;
                end
            end

            if (byte_offset == (frame_l3_offset + 11'd9))
                frame_ip_protocol <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd1))
                frame_ipv4_tos <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd2))
                frame_ipv4_total_length[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd3))
                frame_ipv4_total_length[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd4))
                frame_ipv4_identification[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd5))
                frame_ipv4_identification[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd6))
                frame_ipv4_flags_fragment[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd7))
                frame_ipv4_flags_fragment[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd12))
                frame_ipv4_src[31:24] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd13))
                frame_ipv4_src[23:16] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd14))
                frame_ipv4_src[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd15))
                frame_ipv4_src[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd16))
                frame_ipv4_dst[31:24] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd17))
                frame_ipv4_dst[23:16] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd18))
                frame_ipv4_dst[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd19))
                frame_ipv4_dst[7:0] <= byte_value;

            if (byte_offset == frame_l4_offset)
                frame_icmp_type <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd1))
                frame_icmp_code <= byte_value;
            // 对 Echo Reply 的整个 ICMP 报文重新求和。类型字节和原校验和
            // 按回复中的零值处理，不依赖主机发送校验和卸载是否已经回填。
            if ((frame_ip_protocol == 8'h01) &&
                (byte_offset >= frame_l4_offset) &&
                (byte_offset < (frame_l3_offset +
                                frame_ipv4_total_length)) &&
                (byte_offset < ICMP_CAPTURE_BYTES) &&
                (byte_offset != frame_l4_offset) &&
                (byte_offset != (frame_l4_offset + 11'd2)) &&
                (byte_offset != (frame_l4_offset + 11'd3))) begin
                if (byte_offset[0] == frame_l4_offset[0])
                    frame_icmp_even_byte_sum <=
                        frame_icmp_even_byte_sum + byte_value;
                else
                    frame_icmp_odd_byte_sum <=
                        frame_icmp_odd_byte_sum + byte_value;
            end

            if (byte_offset == frame_l4_offset)
                frame_udp_ports[31:24] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd1))
                frame_udp_ports[23:16] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd2))
                frame_udp_ports[15:8] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd3))
                frame_udp_ports[7:0] <= byte_value;

            if (byte_offset == (frame_l4_offset + 11'd4)) begin
                frame_udp_length[15:8] <= byte_value;
                frame_tcp_seq[31:24] <= byte_value;
            end
            if (byte_offset == (frame_l4_offset + 11'd5)) begin
                frame_udp_length[7:0] <= byte_value;
                frame_tcp_seq[23:16] <= byte_value;
            end
            if (byte_offset == (frame_l4_offset + 11'd6))
                frame_tcp_seq[15:8] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd7))
                frame_tcp_seq[7:0] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd8))
                frame_tcp_ack[31:24] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd9))
                frame_tcp_ack[23:16] <= byte_value;
            if (byte_offset == (frame_l4_offset + 11'd10)) begin
                frame_tcp_ack[15:8] <= byte_value;
                frame_dns_flags[15:8] <= byte_value;
            end
            if (byte_offset == (frame_l4_offset + 11'd11)) begin
                frame_tcp_ack[7:0] <= byte_value;
                frame_dns_flags[7:0] <= byte_value;
            end
            if (byte_offset == (frame_l4_offset + 11'd12)) begin
                frame_tcp_data_offset <= byte_value[7:4];
                frame_dns_qdcount[15:8] <= byte_value;
            end
            if (byte_offset == (frame_l4_offset + 11'd13)) begin
                frame_tcp_flags <= byte_value;
                frame_dns_qdcount[7:0] <= byte_value;
            end

            // 标准 DNS question 从 UDP 头后的 12 字节 DNS header 之后开始。
            // 逐字节寻找 QNAME 的零终止符，避免在组合逻辑中构造 128 路扫描器。
            if ((frame_ip_protocol == 8'h11) &&
                (byte_offset >= (frame_l4_offset + 11'd20)) &&
                (byte_offset < ICMP_CAPTURE_BYTES)) begin
                if (!frame_dns_qname_done) begin
                    dns_qname_idx =
                        byte_offset - (frame_l4_offset + 11'd20);
                    if ((dns_qname_idx >= 11'd18) ||
                        (dns_ascii_lower(byte_value) !=
                         ncsi_dns_qname_byte(dns_qname_idx[4:0])))
                        frame_dns_ncsi_ok <= 1'b0;
                    if ((dns_qname_idx >= 25) || dns_ascii_lower(byte_value) != web_dns_qname_byte(dns_qname_idx[5:0])) frame_dns_web_ok <= 0;
                    if ((dns_qname_idx >= 18) || dns_ascii_lower(byte_value) != legacy_dns_qname_byte(dns_qname_idx[5:0])) frame_dns_legacy_ok <= 0;
                    if (byte_value == 8'h00) begin
                        if (dns_qname_idx == 24 && frame_dns_web_ok) frame_dns_web <= 1;
                        if (dns_qname_idx == 17 && frame_dns_legacy_ok) frame_dns_legacy <= 1;
                        frame_dns_qname_done <= 1'b1;
                        frame_dns_question_end <= byte_offset + 11'd5;
                        if ((dns_qname_idx == 11'd17) &&
                            frame_dns_ncsi_ok)
                            frame_dns_ncsi <= 1'b1;
                    end
                end
                else begin
                    if (byte_offset == (frame_dns_question_end - 11'd4))
                        frame_dns_qtype[15:8] <= byte_value;
                    if (byte_offset == (frame_dns_question_end - 11'd3))
                        frame_dns_qtype[7:0] <= byte_value;
                    if (byte_offset == (frame_dns_question_end - 11'd2))
                        frame_dns_qclass[15:8] <= byte_value;
                    if (byte_offset == (frame_dns_question_end - 11'd1))
                        frame_dns_qclass[7:0] <= byte_value;
                end
            end

            // ARP字段均相对于L3起点，因而同样支持单层VLAN。
            if (byte_offset == (frame_l3_offset + 11'd6))
                frame_arp_opcode[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd7))
                frame_arp_opcode[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd8))
                frame_arp_sender_mac[47:40] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd9))
                frame_arp_sender_mac[39:32] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd10))
                frame_arp_sender_mac[31:24] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd11))
                frame_arp_sender_mac[23:16] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd12))
                frame_arp_sender_mac[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd13))
                frame_arp_sender_mac[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd14))
                frame_arp_sender_ip[31:24] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd15))
                frame_arp_sender_ip[23:16] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd16))
                frame_arp_sender_ip[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd17))
                frame_arp_sender_ip[7:0] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd24))
                frame_arp_target_ip[31:24] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd25))
                frame_arp_target_ip[23:16] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd26))
                frame_arp_target_ip[15:8] <= byte_value;
            if (byte_offset == (frame_l3_offset + 11'd27))
                frame_arp_target_ip[7:0] <= byte_value;

            // BOOTP固定头字段相对于动态UDP起点。
            if (byte_offset == frame_bootp_offset) frame_bootp_op <= byte_value;
            if (byte_offset == frame_bootp_offset+11'd1) frame_bootp_htype <= byte_value;
            if (byte_offset == frame_bootp_offset+11'd2) frame_bootp_hlen <= byte_value;
            if (byte_offset >= frame_bootp_offset+11'd12 && byte_offset < frame_bootp_offset+11'd16)
                frame_bootp_ciaddr <= {frame_bootp_ciaddr[23:0],byte_value};
            if (byte_offset >= frame_bootp_offset+11'd24 && byte_offset < frame_bootp_offset+11'd28)
                frame_bootp_giaddr <= {frame_bootp_giaddr[23:0],byte_value};
            if (byte_offset == (frame_bootp_offset + 11'd4))
                frame_dhcp_xid[31:24] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd5))
                frame_dhcp_xid[23:16] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd6))
                frame_dhcp_xid[15:8] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd7))
                frame_dhcp_xid[7:0] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd10))
                frame_dhcp_flags[15:8] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd11))
                frame_dhcp_flags[7:0] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd28))
                frame_dhcp_chaddr[47:40] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd29))
                frame_dhcp_chaddr[39:32] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd30))
                frame_dhcp_chaddr[31:24] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd31))
                frame_dhcp_chaddr[23:16] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd32))
                frame_dhcp_chaddr[15:8] <= byte_value;
            if (byte_offset == (frame_bootp_offset + 11'd33))
                frame_dhcp_chaddr[7:0] <= byte_value;

            if (byte_offset == (frame_bootp_offset + 11'd236)) begin
                frame_dhcp_cookie[31:24] <= byte_value;
                if (!PRODUCTION && !dbg_packet_candidate_frozen)
                    dbg_packet_bytes[48] <= byte_value;
            end
            if (byte_offset == (frame_bootp_offset + 11'd237)) begin
                frame_dhcp_cookie[23:16] <= byte_value;
                if (!PRODUCTION && !dbg_packet_candidate_frozen)
                    dbg_packet_bytes[49] <= byte_value;
            end
            if (byte_offset == (frame_bootp_offset + 11'd238)) begin
                frame_dhcp_cookie[15:8] <= byte_value;
                if (!PRODUCTION && !dbg_packet_candidate_frozen)
                    dbg_packet_bytes[50] <= byte_value;
            end
            if (byte_offset == (frame_bootp_offset + 11'd239)) begin
                frame_dhcp_cookie[7:0] <= byte_value;
                if (!PRODUCTION && !dbg_packet_candidate_frozen)
                    dbg_packet_bytes[51] <= byte_value;
            end

            if (!dbg_packet_candidate_frozen &&
                (byte_offset == frame_dhcp_options_offset))
                dbg_packet_bytes[52] <= byte_value;
            if (!dbg_packet_candidate_frozen &&
                (byte_offset == (frame_dhcp_options_offset + 11'd1)))
                dbg_packet_bytes[53] <= byte_value;
            if (!dbg_packet_candidate_frozen &&
                (byte_offset == (frame_dhcp_options_offset + 11'd2)))
                dbg_packet_bytes[54] <= byte_value;
            if (!dbg_packet_candidate_frozen &&
                (byte_offset == (frame_dhcp_options_offset + 11'd3)))
                dbg_packet_bytes[55] <= byte_value;

            if (frame_ip_protocol==8'h06 && frame_tcp_data_offset>=5 &&
                byte_offset >= frame_l4_offset+frame_tcp_header_bytes &&
                byte_offset < frame_l3_offset+frame_ipv4_total_length &&
                (byte_offset-frame_l4_offset-frame_tcp_header_bytes)<TCP_REQUEST_MAX_BYTES)
                tcp_frame_bytes[byte_offset-frame_l4_offset-frame_tcp_header_bytes] <= byte_value;

            // DHCP options流式扫描：支持PAD、END和任意选项顺序。
            if ((frame_dhcp_options_offset != 0) &&
                (byte_offset >= frame_dhcp_options_offset) &&
                (byte_offset < (frame_l4_offset + frame_udp_length)) &&
                (byte_offset < (frame_l3_offset + frame_ipv4_total_length))) begin
                case (dhcp_scan_state)
                    DHCP_SCAN_CODE: begin
                        if (byte_value == 8'h00) begin
                            dhcp_scan_state <= DHCP_SCAN_CODE;
                        end
                        else if (byte_value == 8'hFF) begin
                            dhcp_scan_state <= DHCP_SCAN_END;
                            frame_dhcp_end_seen <= 1'b1;
                        end
                        else begin
                            dhcp_scan_code <= byte_value;
                            dhcp_scan_state <= DHCP_SCAN_LEN;
                        end
                    end
                    DHCP_SCAN_LEN: begin
                        dhcp_scan_length <= byte_value;
                        dhcp_scan_remaining <= byte_value;
                        dhcp_scan_index <= 0;
                        if (byte_value == 0) begin
                            frame_dhcp_malformed <= 1'b1;
                            dhcp_scan_state <= DHCP_SCAN_END;
                        end
                        else begin
                            dhcp_scan_state <= DHCP_SCAN_VALUE;
                            if (((dhcp_scan_code == 8'd53) && (byte_value != 8'd1)) ||
                                (((dhcp_scan_code == 8'd50) || (dhcp_scan_code == 8'd54)) && (byte_value != 8'd4)) ||
                                ((dhcp_scan_code == 8'd61) && ((byte_value < 8'd2) || frame_dhcp_client_id_valid)))
                                frame_dhcp_malformed <= 1'b1;
                        end
                    end
                    DHCP_SCAN_VALUE: begin
                        if ((dhcp_scan_code == 8'd61) && (dhcp_scan_index < 8'd255))
                            dhcp_client_id_bytes[dhcp_scan_index] <= byte_value;
                        if ((dhcp_scan_code == 8'd53) &&
                            (dhcp_scan_index == 0))
                            frame_dhcp_message_type <= byte_value;

                        if (dhcp_scan_code == 8'd50) begin
                            case (dhcp_scan_index)
                                8'd0: frame_dhcp_requested_ip[31:24] <= byte_value;
                                8'd1: frame_dhcp_requested_ip[23:16] <= byte_value;
                                8'd2: frame_dhcp_requested_ip[15:8] <= byte_value;
                                8'd3: frame_dhcp_requested_ip[7:0] <= byte_value;
                                default: begin end
                            endcase
                        end
                        if (dhcp_scan_code == 8'd54) begin
                            case (dhcp_scan_index)
                                8'd0: frame_dhcp_server_id[31:24] <= byte_value;
                                8'd1: frame_dhcp_server_id[23:16] <= byte_value;
                                8'd2: frame_dhcp_server_id[15:8] <= byte_value;
                                8'd3: frame_dhcp_server_id[7:0] <= byte_value;
                                default: begin end
                            endcase
                        end

                        if (dhcp_scan_remaining <= 1) begin
                            if ((dhcp_scan_code == 8'd53) &&
                                (dhcp_scan_length == 8'd1))
                                frame_dhcp_message_type_valid <= 1'b1;
                            if ((dhcp_scan_code == 8'd50) &&
                                (dhcp_scan_length == 8'd4))
                                frame_dhcp_requested_ip_valid <= 1'b1;
                            if ((dhcp_scan_code == 8'd54) &&
                                (dhcp_scan_length == 8'd4))
                                frame_dhcp_server_id_valid <= 1'b1;
                            if (dhcp_scan_code == 8'd61) begin
                                frame_dhcp_client_id_valid <= 1'b1;
                                frame_dhcp_client_id_length <= dhcp_scan_length;
                            end
                            dhcp_scan_remaining <= 0;
                            dhcp_scan_state <= DHCP_SCAN_CODE;
                        end
                        else begin
                            dhcp_scan_remaining <= dhcp_scan_remaining - 1'b1;
                            dhcp_scan_index <= dhcp_scan_index + 1'b1;
                        end
                    end
                    default: begin
                        dhcp_scan_state <= DHCP_SCAN_END;
                    end
                endcase
            end
        end
    endtask

    task automatic store_completion_word(
        input [9:0] destination_word,
        input [31:0] raw_word
    );
        reg [31:0] host_word;
        begin
            host_word = byte_swap32(raw_word);
            if ((cpl_kind == RD_TX_DATA) &&
                (destination_word < 32))
                tx_data_words[destination_word[4:0]] <= host_word;
            else if (destination_word < 8) begin
                desc_words[destination_word[2:0]] <= host_word;
            end
        end
    endtask

    integer desc_reset_index;
    integer tx_data_reset_index;
    always @(posedge clk) begin
        // 只在复位安全点拆掉 Completion 接收器。BME/D3 让
        // dma_runtime_ready 掉下去时，在途 MRd 的 CPL 仍必须收完，
        // 否则 tag 悬挂，启用后下一笔 DMA 会把主机冻死。
        if (reset_apply) begin
            cpl_active           <= 1'b0;
            cpl_packet_match     <= 1'b0;
            cpl_kind             <= RD_TX_DESC;
            cpl_store_base_byte  <= 0;
            cpl_expected_words   <= 0;
            cpl_requested_bytes  <= 0;
            cpl_leading_bytes    <= 0;
            cpl_request_lower_addr <= 0;
            cpl_received_words   <= 0;
            cpl_timeout          <= 0;
            cpl_done             <= 1'b0;
            cpl_error            <= 1'b0;
            cpl_error_reason     <= 0;
            if (rst) begin
                dbg_cpl_seen_count <= 0;
                dbg_cpl_match_count <= 0;
                dbg_cpl_reqid_mismatch_count <= 0;
                dbg_cpl_tag_mismatch_count <= 0;
                dbg_cpl_status_error_count <= 0;
                dbg_cpl_nodata_error_count <= 0;
                dbg_cpl_timeout_count <= 0;
                dbg_cpl_malformed_count <= 0;
                dbg_last_cpl_tag <= 0;
                dbg_last_cpl_status <= 0;
            end
            // 描述符缓存只能由 Completion 接收器这一处时序块驱动。
            // 否则 Vivado 会把主状态机中的第二驱动折成常零，破坏 DMA 地址和长度。
            for (desc_reset_index = 0; desc_reset_index < 8;
                 desc_reset_index = desc_reset_index + 1)
                desc_words[desc_reset_index] <= 0;
            for (tx_data_reset_index = 0; tx_data_reset_index < 32;
                 tx_data_reset_index = tx_data_reset_index + 1)
                tx_data_words[tx_data_reset_index] <= 0;
        end
        else begin
            cpl_done  <= 1'b0;
            cpl_error <= 1'b0;

            if (mrd_request_accepted) begin
                cpl_active          <= 1'b1;
                cpl_packet_match    <= 1'b0;
                cpl_kind            <= mrd_pending_kind;
                cpl_store_base_byte <= mrd_pending_store_base_byte;
                cpl_expected_words  <= mrd_pending_expected_words;
                cpl_requested_bytes <= mrd_pending_requested_bytes;
                cpl_leading_bytes   <= mrd_pending_leading_bytes;
                cpl_request_lower_addr <= mrd_pending_addr[6:0];
                cpl_received_words  <= 0;
                cpl_timeout         <= 0;
                cpl_error_reason    <= 0;
            end
            else if (cpl_active) begin
                // 禁用/D3 时常离 L0，旧逻辑只在 L0 加超时 → 永远等不到，
                // 启用后再用同一 tag 发 MRd，主机冻死。武装掉后约 65µs 放弃。
                if ((cpl_timeout == 20'hFFFFF) ||
                    (!dma_runtime_ready && (cpl_timeout[11:0] == 12'hFFF))) begin
                    cpl_active <= 1'b0;
                    cpl_error  <= 1'b1;
                    cpl_error_reason <= 3'd1;
                    dbg_cpl_timeout_count <=
                        dbg_cpl_timeout_count + 1'b1;
                end
                // 链路离开 L0（重训练/降速/ASPM 进出）期间完成包必然延迟，
                // 此时暂停计时，避免把链路抖动误判为 MRd 超时。
                // 武装已掉时必须继续计时：D3/L1 不会再回完成包。
                else if (link_in_l0 || !dma_runtime_ready) begin
                    cpl_timeout <= cpl_timeout + 1'b1;
                end

                if (cpl_first_completion) begin
                    dbg_cpl_seen_count <= dbg_cpl_seen_count + 1'b1;
                    dbg_last_cpl_tag <= tlps_cpl_in.tdata[79:72];
                    dbg_last_cpl_status <= tlps_cpl_in.tdata[47:45];
                    if (!cpl_requester_matches)
                        dbg_cpl_reqid_mismatch_count <=
                            dbg_cpl_reqid_mismatch_count + 1'b1;
                    if (!cpl_tag_matches)
                        dbg_cpl_tag_mismatch_count <=
                            dbg_cpl_tag_mismatch_count + 1'b1;
                    cpl_packet_match <= cpl_first_match;
                    if (cpl_first_owned) begin
                        dbg_cpl_match_count <=
                            dbg_cpl_match_count + 1'b1;
                        if (!cpl_has_data) begin
                            cpl_active <= 1'b0;
                            cpl_error  <= 1'b1;
                            cpl_error_reason <= 3'd2;
                            dbg_cpl_nodata_error_count <=
                                dbg_cpl_nodata_error_count + 1'b1;
                        end
                        else if (tlps_cpl_in.tdata[47:45] != 3'b000) begin
                            cpl_active <= 1'b0;
                            cpl_error  <= 1'b1;
                            cpl_error_reason <= 3'd3;
                            dbg_cpl_status_error_count <=
                                dbg_cpl_status_error_count + 1'b1;
                        end
                        else if (!tlps_cpl_in.tkeepdw[3] ||
                                 !cpl_data_length_plausible) begin
                            cpl_active <= 1'b0;
                            cpl_error  <= 1'b1;
                            cpl_error_reason <= 3'd4;
                            dbg_cpl_malformed_count <=
                                dbg_cpl_malformed_count + 1'b1;
                        end
                        else begin
                            store_completion_word(
                                cpl_received_words,
                                tlps_cpl_in.tdata[127:96]);
                            cpl_received_words <= cpl_received_words + 1'b1;
                            if ((cpl_received_words + 1'b1) >=
                                cpl_expected_words) begin
                                cpl_active <= 1'b0;
                                cpl_done   <= 1'b1;
                            end
                        end
                    end
                end
                else if (tlps_cpl_in.tvalid && cpl_packet_match) begin
                    if (tlps_cpl_in.tkeepdw[0])
                        store_completion_word(
                            cpl_received_words,
                            tlps_cpl_in.tdata[31:0]);
                    if (tlps_cpl_in.tkeepdw[1])
                        store_completion_word(
                            cpl_received_words + 1,
                            tlps_cpl_in.tdata[63:32]);
                    if (tlps_cpl_in.tkeepdw[2])
                        store_completion_word(
                            cpl_received_words + 2,
                            tlps_cpl_in.tdata[95:64]);
                    if (tlps_cpl_in.tkeepdw[3])
                        store_completion_word(
                            cpl_received_words + 3,
                            tlps_cpl_in.tdata[127:96]);

                    cpl_received_words <=
                        cpl_received_words + cpl_word_count;
                    if ((cpl_received_words + cpl_word_count) >=
                        cpl_expected_words) begin
                        cpl_active <= 1'b0;
                        cpl_done   <= 1'b1;
                    end
                end

                if (tlps_cpl_in.tvalid && tlps_cpl_in.tlast)
                    cpl_packet_match <= 1'b0;
            end
        end
    end

    // --------------------------------------------------------------------
    // 主数据面状态机。
    // --------------------------------------------------------------------
    reg [9:0]  mrd_dw_count;
    reg [3:0]  mrd_first_be;
    reg [3:0]  mrd_last_be;

    reg [63:0] tx_segment_addr;
    reg [15:0] tx_segment_remaining;
    reg [15:0] tx_segment_flags;
    reg [15:0] tx_chunk_bytes;
    reg [15:0] tx_chunk_drain_index;
    reg [15:0] packet_total_bytes;
    reg        packet_overflow;

    function automatic [7:0] tx_chunk_byte(
        input [15:0] logical_byte_index
    );
        reg [7:0] payload_byte_index;
        reg [31:0] host_word;
        begin
            payload_byte_index =
                {6'h00, cpl_leading_bytes} + logical_byte_index[7:0];
            host_word = tx_data_words[payload_byte_index[6:2]];
            case (payload_byte_index[1:0])
                2'd0: tx_chunk_byte = host_word[7:0];
                2'd1: tx_chunk_byte = host_word[15:8];
                2'd2: tx_chunk_byte = host_word[23:16];
                default: tx_chunk_byte = host_word[31:24];
            endcase
        end
    endfunction

    // 独立的单写口模板让 Vivado 稳定推断前 128 字节为 LUTRAM。
    // 地址和数据同时复用于字段锁存，避免重复生成加法器和字节选择器。
    wire [10:0] tx_capture_write_offset =
        packet_total_bytes[10:0] + tx_chunk_drain_index[10:0];
    wire [7:0] tx_capture_write_data =
        tx_chunk_byte(tx_chunk_drain_index);
    wire tx_capture_write_enable =
        (state == S_TX_DATA_DRAIN) &&
        (tx_capture_write_offset < ICMP_CAPTURE_BYTES);

    always @(posedge clk) begin
        if (tx_capture_write_enable)
            icmp_request_bytes[tx_capture_write_offset[6:0]] <=
                tx_capture_write_data;
    end

    reg [9:0] dhcp_option_offset;
    reg [7:0] dhcp_message_type;
    reg [31:0] dhcp_requested_ip;
    reg [31:0] dhcp_server_id;
    reg        dhcp_requested_ip_valid;
    reg        dhcp_server_id_valid;

    reg [1:0]  mwr_kind;
    reg [63:0] mwr_base_addr;
    reg [15:0] mwr_total_bytes;
    reg [15:0] mwr_offset_bytes;
    reg [15:0] mwr_chunk_bytes;
    reg [9:0]  mwr_chunk_words;
    reg [9:0]  mwr_words_sent;
    reg [2:0]  mwr_words_this_beat;
    reg [1:0]  mwr_beat_build_index;
    // reply_byte 组合云是全设计最深路径。首拍锁进 mwr_beat_build_q，
    // 次拍再写入 beat/queued/dbg，让 STA 看见寄存器切分。
    reg        mwr_build_settle;
    reg [31:0] mwr_beat_build_q;
    reg [31:0] mwr_beat_word0;
    reg [31:0] mwr_beat_word1;
    reg [31:0] mwr_beat_word2;

    reg force_status_pending;
    // fault 冷却计时：约 0.27s（24'hFFFFFF @62.5MHz）后自动解除锁死。
    reg [23:0] fault_cooldown;
    reg [15:0] rx_wait_count;
    reg [19:0] tx_watchdog_count;
    // 未开包：有 tready 才计发送超时。已开包（header 已上 mux）禁止
    // 局部 watchdog 截断，否则 tlast 永远不到，mux 锁死。
    wire mrd_tx_watchdog_active =
        ((state == S_MRD_WAIT_SENT) &&
         tlps_out.tready && !tlp_open);
    wire mwr_tx_watchdog_active =
        (((state == S_MWR_HDR_WAIT) ||
          (state == S_MWR_DATA_WAIT)) &&
         tlps_out.tready && !tlp_open);
    wire tx_watchdog_expired = &tx_watchdog_count;
    wire watchdog_may_abort = tx_watchdog_expired && !tlp_open;

    // 保留区只读诊断快照，不使用 ILA，也不参与数据面控制。
    // PRODUCTION=1 时恒 0，dbg_* / dbg_packet_bytes 无扇出，可被综合掉。
    generate
    if (PRODUCTION) begin : g_prod_dbg
        assign debug_snapshot = 704'h0;
        assign debug_ext = 448'h0;
    end else begin : g_dbg
        assign debug_snapshot[31:0] = {
        1'b0,
        hostcc_pending,
        dbg_last_drop_reason,
        dbg_last_rx_addr_low,
        dbg_last_tx_addr_low,
        reply_type,
        packet_overflow,
        force_status_pending,
        cpl_active,
        dma_fault_latched,
        dma_config_ready,
        dma_runtime_armed,
        state
    };
    assign debug_snapshot[63:32] = {
        dbg_tx_end_count, dbg_tx_desc_count
    };
    assign debug_snapshot[95:64] = {
        dbg_arp_count,
        dbg_dhcp_request_count,
        dbg_dhcp_discover_count,
        dbg_classified_count
    };
    assign debug_snapshot[127:96] = {
        dbg_rx_reject_count, dbg_rx_accept_count
    };
    assign debug_snapshot[159:128] = {
        dbg_status_commit_count, dbg_reply_commit_count
    };
    assign debug_snapshot[191:160] = {
        rx_std_cons_idx, rx_ret_prod_idx
    };
    // 0x3F80--0x3FBC 冻结首个 DHCP 长度候选帧。
    // 前 48 字节用于判断帧起点和协议头，末 8 字节对应 278--285。
    assign debug_snapshot[223:192] = {
        dbg_packet_length, dbg_packet_flags
    };
    assign debug_snapshot[255:224] = {
        dbg_packet_valid,
        dbg_packet_candidate_frozen,
        dbg_packet_overflow,
        5'h00,
        dbg_packet_capture_count,
        dbg_packet_tx_cons_idx
    };
    assign debug_snapshot[287:256] = {
        dbg_packet_bytes[0], dbg_packet_bytes[1],
        dbg_packet_bytes[2], dbg_packet_bytes[3]
    };
    assign debug_snapshot[319:288] = {
        dbg_packet_bytes[4], dbg_packet_bytes[5],
        dbg_packet_bytes[6], dbg_packet_bytes[7]
    };
    assign debug_snapshot[351:320] = {
        dbg_packet_bytes[8], dbg_packet_bytes[9],
        dbg_packet_bytes[10], dbg_packet_bytes[11]
    };
    assign debug_snapshot[383:352] = {
        dbg_packet_bytes[12], dbg_packet_bytes[13],
        dbg_packet_bytes[14], dbg_packet_bytes[15]
    };
    assign debug_snapshot[415:384] = {
        dbg_packet_bytes[16], dbg_packet_bytes[17],
        dbg_packet_bytes[18], dbg_packet_bytes[19]
    };
    assign debug_snapshot[447:416] = {
        dbg_packet_bytes[20], dbg_packet_bytes[21],
        dbg_packet_bytes[22], dbg_packet_bytes[23]
    };
    assign debug_snapshot[479:448] = {
        dbg_packet_bytes[24], dbg_packet_bytes[25],
        dbg_packet_bytes[26], dbg_packet_bytes[27]
    };
    assign debug_snapshot[511:480] = {
        dbg_packet_bytes[28], dbg_packet_bytes[29],
        dbg_packet_bytes[30], dbg_packet_bytes[31]
    };
    assign debug_snapshot[543:512] = {
        dbg_packet_bytes[32], dbg_packet_bytes[33],
        dbg_packet_bytes[34], dbg_packet_bytes[35]
    };
    assign debug_snapshot[575:544] = {
        dbg_packet_bytes[36], dbg_packet_bytes[37],
        dbg_packet_bytes[38], dbg_packet_bytes[39]
    };
    assign debug_snapshot[607:576] = {
        dbg_packet_bytes[40], dbg_packet_bytes[41],
        dbg_packet_bytes[42], dbg_packet_bytes[43]
    };
    assign debug_snapshot[639:608] = {
        dbg_packet_bytes[44], dbg_packet_bytes[45],
        dbg_packet_bytes[46], dbg_packet_bytes[47]
    };
    assign debug_snapshot[671:640] = {
        dbg_packet_bytes[48], dbg_packet_bytes[49],
        dbg_packet_bytes[50], dbg_packet_bytes[51]
    };
    assign debug_snapshot[703:672] = {
        dbg_packet_bytes[52], dbg_packet_bytes[53],
        dbg_packet_bytes[54], dbg_packet_bytes[55]
    };

    // 0x3E10--0x3E2C 扩展诊断。计数只在硬复位时清零，避免每次 NOW
    // 清空启动失败的证据。
    assign debug_ext[31:0] = {
        dbg_hostcc_accept_count, dbg_hostcc_deferred_count
    };
    assign debug_ext[63:32] = {
        dbg_mrd_tx_timeout_count, dbg_mwr_tx_timeout_count
    };
    assign debug_ext[95:64] = {
        dbg_cpl_seen_count, dbg_cpl_match_count
    };
    assign debug_ext[127:96] = {
        dbg_cpl_reqid_mismatch_count, dbg_cpl_tag_mismatch_count
    };
    assign debug_ext[159:128] = {
        dbg_cpl_status_error_count, dbg_cpl_nodata_error_count
    };
    assign debug_ext[191:160] = {
        dbg_cpl_timeout_count, dbg_cpl_malformed_count
    };
    assign debug_ext[223:192] = {
        dbg_last_cpl_tag,
        dbg_last_cpl_status,
        cpl_received_words,
        cpl_expected_words,
        1'b0
    };
    assign debug_ext[255:224] = {
        dbg_tx_packet_total, dbg_rx_packet_total
    };
    assign debug_ext[287:256] = dbg_offer_addr[31:0];
    assign debug_ext[319:288] = dbg_offer_addr[63:32];
    assign debug_ext[351:320] = dbg_offer_ret_addr;
    assign debug_ext[383:352] = dbg_offer_opaque;
    assign debug_ext[415:384] = dbg_offer_idx_len;
    assign debug_ext[447:416] = {
        dbg_offer_frozen, dbg_offer_irq, 6'h00,
        dbg_offer_tag, dbg_offer_prod
    };
    end
    endgenerate

    wire [63:0] tx_desc_current_addr =
        tx_ring_addr + ({48'h0, tx_cons_idx} << 4);
    wire [63:0] rx_desc_current_addr =
        rx_std_ring_addr + ({48'h0, rx_std_cons_idx} << 5);
    wire [63:0] rx_ret_current_addr =
        rx_ret_ring_addr + ({48'h0, rx_ret_prod_idx} << 5);

    wire [12:0] tx_page_remaining =
        13'd4096 - {1'b0, tx_segment_addr[11:0]};
    wire [15:0] tx_tlp_data_limit =
        16'd128 - {14'h0000, tx_segment_addr[1:0]};
    wire [15:0] tx_chunk_limit =
        (tx_segment_remaining > tx_tlp_data_limit) ?
            tx_tlp_data_limit : tx_segment_remaining;
    wire [15:0] tx_chunk_calculated =
        (tx_page_remaining < tx_chunk_limit) ?
            {3'b000, tx_page_remaining} : tx_chunk_limit;

    wire [63:0] mwr_current_addr =
        mwr_base_addr + mwr_offset_bytes;
    wire [63:0] mwr_tlp_addr =
        {mwr_current_addr[63:2], 2'b00};
    wire [15:0] mwr_remaining_bytes =
        mwr_total_bytes - mwr_offset_bytes;
    wire [12:0] mwr_page_remaining =
        13'd4096 - {1'b0, mwr_current_addr[11:0]};
    wire [15:0] mwr_tlp_data_limit =
        16'd128 - {14'h0000, mwr_current_addr[1:0]};
    wire [15:0] mwr_chunk_limit =
        (mwr_remaining_bytes > mwr_tlp_data_limit) ?
            mwr_tlp_data_limit : mwr_remaining_bytes;
    wire [15:0] mwr_chunk_calculated =
        (mwr_page_remaining < mwr_chunk_limit) ?
            {3'b000, mwr_page_remaining} : mwr_chunk_limit;
    wire [9:0] mwr_chunk_words_calculated =
        ({14'h0000, mwr_current_addr[1:0]} +
         mwr_chunk_calculated + 3) >> 2;
    wire [3:0] mwr_first_be_calculated =
        first_be_for_range(
            mwr_current_addr[1:0], mwr_chunk_calculated);
    wire [3:0] mwr_last_be_calculated =
        last_be_for_range(
            mwr_current_addr[1:0], mwr_chunk_calculated);

    wire [9:0] mwr_words_left =
        mwr_chunk_words - mwr_words_sent;
    wire [2:0] mwr_words_next_beat =
        (mwr_words_left >= 4) ? 3'd4 : mwr_words_left[2:0];
    wire [3:0] mwr_keep_next_beat =
        (mwr_words_next_beat == 4) ? 4'b1111 :
        (mwr_words_next_beat == 3) ? 4'b0111 :
        (mwr_words_next_beat == 2) ? 4'b0011 :
                                     4'b0001;
    function automatic [7:0] mwr_source_byte(
        input [15:0] source_byte_offset
    );
        reg [31:0] source_word;
        begin
            if (mwr_kind == WR_REPLY) begin
                mwr_source_byte = reply_byte(source_byte_offset[9:0]);
            end
            else begin
                source_word = mwr_host_dword(
                    mwr_kind, source_byte_offset[11:2]);
                case (source_byte_offset[1:0])
                    2'd0: mwr_source_byte = source_word[7:0];
                    2'd1: mwr_source_byte = source_word[15:8];
                    2'd2: mwr_source_byte = source_word[23:16];
                    default: mwr_source_byte = source_word[31:24];
                endcase
            end
        end
    endfunction

    function automatic [7:0] mwr_payload_byte(
        input [11:0] payload_byte_offset
    );
        reg [15:0] source_byte_offset;
        begin
            mwr_payload_byte = 8'h00;
            if ((payload_byte_offset >= mwr_current_addr[1:0]) &&
                ((payload_byte_offset - mwr_current_addr[1:0]) <
                 mwr_chunk_bytes)) begin
                source_byte_offset =
                    mwr_offset_bytes + payload_byte_offset -
                    mwr_current_addr[1:0];
                mwr_payload_byte =
                    mwr_source_byte(source_byte_offset);
            end
        end
    endfunction

    function automatic [31:0] mwr_payload_host_dword(
        input [9:0] payload_word_index
    );
        reg [11:0] payload_byte_base;
        begin
            payload_byte_base = {payload_word_index, 2'b00};
            mwr_payload_host_dword = {
                mwr_payload_byte(payload_byte_base + 2'd3),
                mwr_payload_byte(payload_byte_base + 2'd2),
                mwr_payload_byte(payload_byte_base + 1'b1),
                mwr_payload_byte(payload_byte_base)
            };
        end
    endfunction

    // 每周期只生成一个 DWORD，避免 128 位数据拍把 reply_byte 组合逻辑复制 16 份。
    wire [31:0] mwr_beat_build_word = byte_swap32(
        mwr_payload_host_dword(
            mwr_words_sent + {{8{1'b0}}, mwr_beat_build_index})
    );
    wire [15:0] dbg_offer_src_base =
        mwr_offset_bytes +
        (({6'h0, mwr_words_sent} +
          {14'h0, mwr_beat_build_index}) << 2);
    wire [31:0] dbg_offer_host_dword = byte_swap32(mwr_beat_build_word);
    wire dbg_offer_capture_en =
        !PRODUCTION &&
        (state == S_MWR_DATA_QUEUE) &&
        !out_pending &&
        (mwr_kind == WR_REPLY) &&
        (reply_type == REPLY_DHCP_OFFER) &&
        !dbg_offer_bytes_done &&
        (mwr_current_addr[1:0] == 2'b00);

    wire rx_buffer_available =
        rx_config_ready && (rx_std_cons_idx != rx_std_prod_idx);
    wire rx_return_available =
        rx_config_ready &&
        (ring_next(rx_ret_prod_idx, rx_ret_ring_size) !=
         rx_ret_cons_idx);

    // 首次 NOW 负责武装数据面；后续只在发送路径完全空闲时接受。
    // 已解除武装时也必须等 queued/open 清空：否则旧 queued_tdata 会
    // 在下一拍 NOW 后打到驱动已释放的 DMA 地址。
    // 软复位挂起期间不接受 NOW，请求保持锁存。
    wire hostcc_request = hostcc_now || hostcc_pending;
    wire dma_tx_path_clear =
        ((state == S_WAIT_CONFIG) || (state == S_IDLE)) &&
        !cpl_active && !tlp_open && !out_pending && !out_tvalid;
    wire hostcc_reset_accepted =
        hostcc_request && tx_config_ready &&
        !reset_pending && !lifecycle_reset_req &&
        dma_tx_path_clear;

    always @(posedge clk) begin
        if (reset_apply) begin
            state                    <= S_WAIT_CONFIG;
            after_mrd_state          <= S_IDLE;
            after_mwr_state          <= S_IDLE;
            irq_request              <= 1'b0;
            tx_stat_event            <= 1'b0;
            rx_stat_event            <= 1'b0;
            tx_stat_bytes            <= 0;
            rx_stat_bytes            <= 0;
            tx_stat_class            <= 0;
            rx_stat_class            <= 0;
            force_status_pending     <= 1'b0;
            link_status_pending      <= 1'b0;
            status_link_change       <= 1'b0;
            dma_runtime_armed        <= 1'b0;
            // 硬复位清空挂起的 NOW；软复位(reset_pending)保留，使复位窗口内
            // 收到的武装请求在驱动重写同代配置后仍能重新武装数据面。否则该
            // NOW 被丢弃，dma_runtime_armed 永不置位，FSM 卡在 S_WAIT_CONFIG，
            // DHCP 应答与 irq_request 均无法产生。
            hostcc_pending           <= rst ? 1'b0 : hostcc_pending;
            reset_pending            <= 1'b0;
            dma_fault_latched        <= 1'b0;
            fault_cooldown           <= 0;
            rx_wait_count            <= 0;
            tx_watchdog_count        <= 0;
            dbg_tx_desc_count        <= 0;
            dbg_tx_end_count         <= 0;
            dbg_classified_count     <= 0;
            dbg_dhcp_discover_count  <= 0;
            dbg_dhcp_request_count   <= 0;
            dbg_arp_count            <= 0;
            dbg_rx_accept_count      <= 0;
            dbg_rx_reject_count      <= 0;
            dbg_reply_commit_count   <= 0;
            dbg_status_commit_count  <= 0;
            dbg_last_drop_reason     <= 0;
            dbg_last_tx_addr_low     <= 0;
            dbg_last_rx_addr_low     <= 0;
            dbg_packet_length        <= 0;
            dbg_packet_flags         <= 0;
            dbg_packet_tx_cons_idx   <= 0;
            dbg_packet_capture_count <= 0;
            dbg_packet_valid         <= 1'b0;
            dbg_packet_candidate_frozen <= 1'b0;
            dbg_packet_overflow      <= 1'b0;
            dbg_offer_pending        <= 1'b0;
            dbg_offer_frozen         <= 1'b0;
            dbg_offer_bytes_done     <= 1'b0;
            dbg_offer_irq            <= 1'b0;
            dbg_offer_addr           <= 0;
            dbg_offer_ret_addr       <= 0;
            dbg_offer_opaque         <= 0;
            dbg_offer_idx_len        <= 0;
            dbg_offer_prod           <= 0;
            dbg_offer_tag            <= 0;
            dbg_hostcc_accept_count  <= 0;
            dbg_hostcc_deferred_count <= 0;
            dbg_mrd_tx_timeout_count <= 0;
            dbg_mwr_tx_timeout_count <= 0;
            dbg_tx_packet_total      <= 0;
            dbg_rx_packet_total      <= 0;
            frame_ethertype          <= 0;
            frame_ip_protocol        <= 0;
            frame_arp_opcode         <= 0;
            frame_arp_sender_mac     <= 0;
            frame_arp_sender_ip      <= 0;
            frame_arp_target_ip      <= 0;
            frame_udp_ports          <= 0;
            frame_dhcp_xid           <= 0;
            frame_dhcp_flags         <= 0;
            frame_dhcp_chaddr        <= 0;
            frame_dhcp_cookie        <= 0;
            frame_dhcp_message_option <= 0;
            frame_dst_mac            <= 0;
            frame_outer_ethertype    <= 0;
            frame_vlan_tagged        <= 1'b0;
            frame_ipv4_header_valid  <= 1'b0;
            frame_l3_offset          <= 11'd14;
            frame_l4_offset          <= 0;
            frame_bootp_offset       <= 0;
            frame_dhcp_options_offset <= 0;
            frame_dhcp_message_type  <= 0;
            frame_dhcp_message_type_valid <= 1'b0;
            frame_dhcp_requested_ip  <= 0;
            frame_dhcp_server_id     <= 0;
            frame_dhcp_requested_ip_valid <= 1'b0;
            frame_dhcp_server_id_valid <= 1'b0;
            frame_vlan_tci <= 0;
            frame_vlan_tpid <= 16'h8100;
            frame_bootp_op <= 0; frame_bootp_htype <= 0; frame_bootp_hlen <= 0;
            frame_bootp_ciaddr <= 0; frame_bootp_giaddr <= 0;
            frame_dhcp_malformed <= 0; frame_dhcp_end_seen <= 0;
            frame_dhcp_client_id_valid <= 0; frame_dhcp_client_id_length <= 0;
            frame_dns_web_ok <= 1; frame_dns_legacy_ok <= 1;
            frame_dns_web <= 0; frame_dns_legacy <= 0;
            frame_src_mac            <= 0;
            frame_ipv4_src           <= 0;
            frame_ipv4_dst           <= 0;
            frame_ipv4_tos           <= 0;
            frame_ipv4_total_length  <= 0;
            frame_ipv4_identification <= 0;
            frame_ipv4_flags_fragment <= 0;
            frame_udp_length         <= 0;
            frame_dns_flags          <= 0;
            frame_dns_qdcount        <= 0;
            frame_dns_qtype          <= 0;
            frame_dns_qclass         <= 0;
            frame_dns_question_end   <= 0;
            frame_dns_qname_done     <= 1'b0;
            frame_dns_ncsi_ok        <= 1'b1;
            frame_dns_ncsi           <= 1'b0;
            frame_tcp_seq            <= 0;
            frame_tcp_ack            <= 0;
            frame_tcp_data_offset    <= 0;
            frame_tcp_flags          <= 0;
            frame_icmp_type          <= 0;
            frame_icmp_code          <= 0;
            frame_icmp_even_byte_sum <= 0;
            frame_icmp_odd_byte_sum  <= 0;
            dhcp_scan_state          <= DHCP_SCAN_CODE;
            dhcp_scan_code           <= 0;
            dhcp_scan_length         <= 0;
            dhcp_scan_remaining      <= 0;
            dhcp_scan_index          <= 0;
            for (dbg_packet_reset_index = 0;
                 dbg_packet_reset_index < 56;
                 dbg_packet_reset_index =
                     dbg_packet_reset_index + 1)
                dbg_packet_bytes[dbg_packet_reset_index] <= 0;

            out_pending              <= 1'b0;
            out_sent                 <= 1'b0;
            out_tvalid               <= 1'b0;
            queued_tdata             <= 0;
            queued_tkeepdw           <= 0;
            queued_tlast             <= 1'b0;
            queued_tuser             <= 0;
            queued_starts_mwr        <= 1'b0;
            out_tdata                <= 0;
            out_tkeepdw              <= 0;
            out_tlast                <= 1'b0;
            out_tuser                <= 0;
            mrd_tlp_open             <= 1'b0;
            mwr_tlp_open             <= 1'b0;
            mrd_tx_done_latched      <= 1'b0;
            mwr_tx_done_latched      <= 1'b0;

            mrd_kind                 <= RD_TX_DESC;
            mrd_store_base_byte      <= 0;
            mrd_expected_words       <= 0;
            mrd_requested_bytes      <= 0;
            mrd_leading_bytes        <= 0;
            mrd_addr                 <= 0;
            mrd_pending_kind         <= RD_TX_DESC;
            mrd_pending_store_base_byte <= 0;
            mrd_pending_expected_words <= 0;
            mrd_pending_requested_bytes <= 0;
            mrd_pending_leading_bytes <= 0;
            mrd_pending_addr         <= 0;
            mrd_dw_count             <= 0;
            mrd_first_be             <= 0;
            mrd_last_be              <= 0;

            tx_cons_idx              <= 0;
            rx_std_cons_idx          <= 0;
            rx_ret_prod_idx          <= 0;
            status_tag               <= 0;
            committed_status_tag     <= 0;
            packet_total_bytes       <= 0;
            packet_overflow          <= 1'b0;
            reply_type               <= REPLY_NONE;
            reply_frame_length       <= 0;
            client_mac               <= 0;
            client_ipv4              <= 0;
            dhcp_xid                 <= 0;
            dhcp_flags               <= 0;
            dhcp_acked               <= 1'b0;
            icmp_l3_offset           <= 0;
            icmp_l4_offset           <= 0;
            icmp_reply_ip_checksum   <= 0;
            icmp_reply_message_checksum <= 0;
            icmp_request_length      <= 0;
            dns_question_end         <= 0;
            dns_ncsi                 <= 1'b0;
            reply_ipv4_identification <= 0;
            tcp_client_port          <= 0;
            tcp_session_active <= 0; tcp_session_established <= 0; tcp_fin_sent <= 0; tcp_client_fin_seen <= 0;
            tcp_peer_mac <= 0; tcp_peer_ip <= 0; tcp_peer_port <= 0;
            tcp_peer_vlan <= 0; tcp_peer_tci <= 0; tcp_peer_tpid <= 16'h8100;
            tcp_client_next_seq <= 0; tcp_server_next_seq <= 0;
            tcp_session_age <= 0; tcp_retry_age <= 0; tcp_retry_count <= 0; tcp_response_pending <= 0;
            tcp_scan_index <= 0; tcp_http_bytes <= 0; tcp_line_index <= 0;
            tcp_header_tail <= 0; tcp_first_line <= 1;
            tcp_modern_match <= 1; tcp_legacy_match <= 1;
            tcp_path_modern <= 0; tcp_path_legacy <= 0; tcp_http_10 <= 0;
            tcp_host_prefix_match <= 1; tcp_host_started <= 0; tcp_host_tail <= 0;
            tcp_host_modern_match <= 1; tcp_host_legacy_match <= 1;
            tcp_host_modern_seen <= 0; tcp_host_legacy_seen <= 0;
            tcp_host_index <= 0; tcp_header_index <= 0; http_legacy_reply <= 0;
            reply_vlan <= 0; reply_vlan_tci <= 0; reply_vlan_tpid <= 16'h8100;
            dns_answer_count <= 0; dns_rcode <= 0;
            service_ipv4 <= gw_ipv4; tcp_peer_service_ipv4 <= gw_ipv4;
            dhcp_ciaddr <= 0; dhcp_client_id_valid <= 0; dhcp_client_id_length <= 0;
            dhcp_reply_udp_checksum <= 0; dhcp_checksum_sum <= 0;
            dhcp_checksum_index <= 0; dhcp_checksum_running <= 0; dhcp_checksum_byte_q <= 0;
            tcp_reply_sequence       <= 0;
            tcp_reply_acknowledgment <= 0;
            bg_tick_count            <= 0;
            bg_lfsr                  <= BG_LFSR_SEED;
            bg_source_slot           <= 0;
            bg_target_host           <= 8'd120;

            tx_segment_addr          <= 0;
            tx_segment_remaining     <= 0;
            tx_segment_flags         <= 0;
            tx_chunk_bytes           <= 0;
            tx_chunk_drain_index     <= 0;

            dhcp_option_offset       <= 0;
            dhcp_message_type        <= 0;
            dhcp_requested_ip        <= 0;
            dhcp_server_id           <= 0;
            dhcp_requested_ip_valid  <= 1'b0;
            dhcp_server_id_valid     <= 1'b0;

            mwr_kind                 <= WR_STATUS;
            mwr_base_addr            <= 0;
            mwr_total_bytes          <= 0;
            mwr_offset_bytes         <= 0;
            mwr_chunk_bytes          <= 0;
            mwr_chunk_words          <= 0;
            mwr_words_sent           <= 0;
            mwr_words_this_beat      <= 0;
            mwr_beat_build_index     <= 0;
            mwr_build_settle         <= 1'b0;
            mwr_beat_build_q         <= 0;
            mwr_beat_word0           <= 0;
            mwr_beat_word1           <= 0;
            mwr_beat_word2           <= 0;

            rx_buffer_addr           <= 0;
            rx_buffer_capacity       <= 0;
            rx_buffer_index          <= 0;
            rx_buffer_opaque         <= 0;

        end
        else begin
            irq_request <= 1'b0;
            tx_stat_event <= 1'b0;
            rx_stat_event <= 1'b0;
            out_tvalid  <= 1'b0;
            out_sent    <= 1'b0;

            // 软复位请求是单拍脉冲，先锁存，待包边界安全点(reset_apply)
            // 再执行真正的整体复位。
            if (lifecycle_reset_req)
                reset_pending <= 1'b1;

            if (!dma_runtime_ready) begin
                bg_tick_count <= 0;
                bg_lfsr       <= BG_LFSR_SEED;
            end
            else if (bg_due) begin
                bg_tick_count <= 0;
                bg_lfsr <= {bg_lfsr[14:0], bg_lfsr_feedback};
            end
            else if (BG_ENABLE) begin
                bg_tick_count <= bg_tick_count + 1'b1;
            end
            else begin
                bg_tick_count <= 0;
            end

            if (!tcp_session_active) tcp_session_age <= 0;
            else if (tcp_session_age >= 32'd1875000000) begin
                tcp_session_active <= 0; tcp_session_established <= 0;
                tcp_response_pending <= 0;
            end else tcp_session_age <= tcp_session_age+1'b1;
            if (!tcp_response_pending) tcp_retry_age <= 0;
            else if (tcp_retry_age < 32'd62500000) tcp_retry_age <= tcp_retry_age+1'b1;

            // 链路事件只请求状态块写回，不复用会清空数据面的 HOSTCC.NOW。
            if (link_event)
                link_status_pending <= 1'b1;

            // NOW 是事件而不是电平。配置未完成或状态机暂忙时先锁存，
            // 待同一代配置完整且数据面安全空闲后只消费一次。
            if (hostcc_now && !hostcc_reset_accepted) begin
                hostcc_pending <= 1'b1;
                dbg_hostcc_deferred_count <=
                    dbg_hostcc_deferred_count + 1'b1;
            end
            if (hostcc_reset_accepted) begin
                hostcc_pending <= 1'b0;
                dbg_hostcc_accept_count <=
                    dbg_hostcc_accept_count + 1'b1;
            end

            if (mrd_tx_watchdog_active || mwr_tx_watchdog_active) begin
                if (!tx_watchdog_expired)
                    tx_watchdog_count <= tx_watchdog_count + 1'b1;
            end
            else begin
                tx_watchdog_count <= 0;
            end

            // 每次有效 NOW（包括初始化阶段第一次）都请求状态块更新。
            // 第一次 NOW 同拍解锁数据面，挂起请求保留到下一拍再执行。
            if (hostcc_reset_accepted && tx_config_ready)
                force_status_pending <= 1'b1;

            if (hostcc_reset_accepted) begin
                dbg_tx_desc_count        <= 0;
                dbg_tx_end_count         <= 0;
                dbg_classified_count     <= 0;
                dbg_dhcp_discover_count  <= 0;
                dbg_dhcp_request_count   <= 0;
                dbg_arp_count            <= 0;
                dbg_rx_accept_count      <= 0;
                dbg_rx_reject_count      <= 0;
                dbg_reply_commit_count   <= 0;
                dbg_status_commit_count  <= 0;
                dbg_last_drop_reason     <= 0;
                dbg_last_tx_addr_low     <= 0;
                dbg_last_rx_addr_low     <= 0;
                dbg_packet_length        <= 0;
                dbg_packet_flags         <= 0;
                dbg_packet_tx_cons_idx   <= 0;
                dbg_packet_capture_count <= 0;
                dbg_packet_valid         <= 1'b0;
                dbg_packet_candidate_frozen <= 1'b0;
                dbg_packet_overflow      <= 1'b0;
                dbg_offer_pending        <= 1'b0;
                dbg_offer_frozen         <= 1'b0;
                dbg_offer_bytes_done     <= 1'b0;
                dbg_offer_irq            <= 1'b0;
                dbg_offer_addr           <= 0;
                dbg_offer_ret_addr       <= 0;
                dbg_offer_opaque         <= 0;
                dbg_offer_idx_len        <= 0;
                dbg_offer_prod           <= 0;
                dbg_offer_tag            <= 0;
                frame_ethertype          <= 0;
                frame_ip_protocol        <= 0;
                frame_arp_opcode         <= 0;
                frame_arp_sender_mac     <= 0;
                frame_arp_sender_ip      <= 0;
                frame_arp_target_ip      <= 0;
                frame_udp_ports          <= 0;
                frame_dhcp_xid           <= 0;
                frame_dhcp_flags         <= 0;
                frame_dhcp_chaddr        <= 0;
                frame_dhcp_cookie        <= 0;
                frame_dhcp_message_option <= 0;
                frame_dst_mac            <= 0;
                frame_outer_ethertype    <= 0;
                frame_vlan_tagged        <= 1'b0;
                frame_ipv4_header_valid  <= 1'b0;
                frame_l3_offset          <= 11'd14;
                frame_l4_offset          <= 0;
                frame_bootp_offset       <= 0;
                frame_dhcp_options_offset <= 0;
                frame_dhcp_message_type  <= 0;
                frame_dhcp_message_type_valid <= 1'b0;
                frame_dhcp_requested_ip  <= 0;
                frame_dhcp_server_id     <= 0;
                frame_dhcp_requested_ip_valid <= 1'b0;
                frame_dhcp_server_id_valid <= 1'b0;
                frame_vlan_tci <= 0;
                frame_vlan_tpid <= 16'h8100;
                frame_bootp_op <= 0; frame_bootp_htype <= 0; frame_bootp_hlen <= 0;
                frame_bootp_ciaddr <= 0; frame_bootp_giaddr <= 0;
                frame_dhcp_malformed <= 0; frame_dhcp_end_seen <= 0;
                frame_dhcp_client_id_valid <= 0; frame_dhcp_client_id_length <= 0;
                frame_dns_web_ok <= 1; frame_dns_legacy_ok <= 1;
                frame_dns_web <= 0; frame_dns_legacy <= 0;
                frame_src_mac            <= 0;
                frame_ipv4_src           <= 0;
                frame_ipv4_dst           <= 0;
                frame_ipv4_tos           <= 0;
                frame_ipv4_total_length  <= 0;
                frame_ipv4_identification <= 0;
                frame_ipv4_flags_fragment <= 0;
                frame_udp_length         <= 0;
                frame_dns_flags          <= 0;
                frame_dns_qdcount        <= 0;
                frame_dns_qtype          <= 0;
                frame_dns_qclass         <= 0;
                frame_dns_question_end   <= 0;
                frame_dns_qname_done     <= 1'b0;
                frame_dns_ncsi_ok        <= 1'b1;
                frame_dns_ncsi           <= 1'b0;
                frame_tcp_seq            <= 0;
                frame_tcp_ack            <= 0;
                frame_tcp_data_offset    <= 0;
                frame_tcp_flags          <= 0;
                frame_icmp_type          <= 0;
                frame_icmp_code          <= 0;
                frame_icmp_even_byte_sum <= 0;
                frame_icmp_odd_byte_sum  <= 0;
                dhcp_scan_state          <= DHCP_SCAN_CODE;
                dhcp_scan_code           <= 0;
                dhcp_scan_length         <= 0;
                dhcp_scan_remaining      <= 0;
                dhcp_scan_index          <= 0;
                tx_chunk_drain_index     <= 0;
                icmp_reply_message_checksum <= 0;
                dns_question_end         <= 0;
                dns_ncsi                 <= 1'b0;
                reply_ipv4_identification <= 0;
                tcp_client_port          <= 0;
                tcp_session_active <= 0; tcp_session_established <= 0; tcp_fin_sent <= 0; tcp_client_fin_seen <= 0;
                tcp_peer_mac <= 0; tcp_peer_ip <= 0; tcp_peer_port <= 0;
                tcp_peer_vlan <= 0; tcp_peer_tci <= 0; tcp_peer_tpid <= 16'h8100;
                tcp_client_next_seq <= 0; tcp_server_next_seq <= 0;
                tcp_session_age <= 0; tcp_retry_age <= 0; tcp_retry_count <= 0; tcp_response_pending <= 0;
                tcp_scan_index <= 0; tcp_http_bytes <= 0; tcp_line_index <= 0;
                tcp_header_tail <= 0; tcp_first_line <= 1;
                tcp_modern_match <= 1; tcp_legacy_match <= 1;
                tcp_path_modern <= 0; tcp_path_legacy <= 0; tcp_http_10 <= 0;
                tcp_host_prefix_match <= 1; tcp_host_started <= 0; tcp_host_tail <= 0;
                tcp_host_modern_match <= 1; tcp_host_legacy_match <= 1;
                tcp_host_modern_seen <= 0; tcp_host_legacy_seen <= 0;
                tcp_host_index <= 0; tcp_header_index <= 0; http_legacy_reply <= 0;
                reply_vlan <= 0; reply_vlan_tci <= 0; reply_vlan_tpid <= 16'h8100;
                dns_answer_count <= 0; dns_rcode <= 0;
            service_ipv4 <= gw_ipv4; tcp_peer_service_ipv4 <= gw_ipv4;
                dhcp_ciaddr <= 0; dhcp_client_id_valid <= 0; dhcp_client_id_length <= 0;
                dhcp_reply_udp_checksum <= 0; dhcp_checksum_sum <= 0;
                dhcp_checksum_index <= 0; dhcp_checksum_running <= 0; dhcp_checksum_byte_q <= 0;
                tcp_reply_sequence       <= 0;
                tcp_reply_acknowledgment <= 0;
                for (dbg_packet_reset_index = 0;
                     dbg_packet_reset_index < 56;
                     dbg_packet_reset_index =
                         dbg_packet_reset_index + 1)
                    dbg_packet_bytes[dbg_packet_reset_index] <= 0;
            end

            if (!tx_config_ready)
                dma_runtime_armed <= 1'b0;
            else if (hostcc_reset_accepted) begin
                dma_runtime_armed <= 1'b1;
                // tag 隔离不随 NOW 清除：迟到的旧 Completion 仍可能
                // 匹配下一笔固定 tag 0x1E。冷却到期后自行恢复。
            end
            // 会话结束沿（BME/D0 1→0）：丢弃跨会话的挂起 NOW——否则启用后
            // BME 一开，旧请求会在驱动重编地址前自动武装数据面。
            bme_prev <= bus_master_enable;
            d0_prev  <= power_state_d0;
            if (dma_session_end)
                hostcc_pending <= 1'b0;
            // 软复位挂起期间保持解除武装：状态机立即沿既有 abort 路径
            // 收尾（已打开的 MWr 保证发完），不再发起任何新事务。
            if (reset_pending || lifecycle_reset_req)
                dma_runtime_armed <= 1'b0;

            // MRd 超时等故障不再永久锁死：冷却约 0.27s 后自动清除，
            // 同一代配置仍有效时数据面自行恢复。冷却时长远超完成包的
            // 实际延迟，迟到的陈旧完成包到达时已无人认领而被正常丢弃，
            // 不会错配到新的 MRd。驱动写 HOSTCC.NOW 仍可立即恢复。
            if (!dma_fault_latched)
                fault_cooldown <= 0;
            else if (fault_cooldown == 24'hFFFFFF)
                dma_fault_latched <= 1'b0;
            else
                fault_cooldown <= fault_cooldown + 1'b1;

            if (state != S_WAIT_RX_BUFFER)
                rx_wait_count <= 0;

            // 禁用时丢掉尚未获得 mux 授权的排队拍，并清 queued_tdata。
            // 这些拍还没上 AXI，丢弃不会留下半包；留下则会：has_data=0、
            // mux 不给 tready、teardown 等 pending=0、watchdog 不计时。
            if (!dma_runtime_ready && queued_ungranted) begin
                out_pending       <= 1'b0;
                out_sent          <= 1'b0;
                queued_tdata      <= 128'h0;
                queued_tkeepdw    <= 4'h0;
                queued_tlast      <= 1'b0;
                queued_tuser      <= 9'h0;
                queued_starts_mwr <= 1'b0;
                dbg_last_drop_reason <= 8'h0B;
            end
            // 将已排队的一个 128 位拍转换为适配器要求的单周期 valid。
            else if (beat_grant) begin
                out_tdata    <= queued_tdata;
                out_tkeepdw  <= queued_tkeepdw;
                out_tlast    <= queued_tlast;
                out_tuser    <= queued_tuser;
                out_tvalid   <= 1'b1;
                out_pending  <= 1'b0;
                out_sent     <= 1'b1;
                if (state == S_MRD_WAIT_SENT)
                    mrd_tlp_open <= 1'b1;
                if (queued_starts_mwr)
                    mwr_tlp_open <= 1'b1;
            end

            // 本源 TLP 被硬核收下 last 后才能清 open。
            if (mrd_tx_done_event) begin
                mrd_tlp_open        <= 1'b0;
                mrd_tx_done_latched <= 1'b1;
            end
            if (mwr_tx_done_event) begin
                mwr_tlp_open <= 1'b0;
                mwr_tx_done_latched <= 1'b1;
            end

            // BME 清除 / D3 / 环地址被清时解除武装，但必须先把已打开的
            // MRd/MWr 完整发到 tlast，并收完对应 Completion。半包留在
            // mux 上会堵死 BAR/CFG 完成包，主机 MMIO 读超时 → 整机冻结。
            if (!dma_runtime_ready &&
                !tlp_open && !cpl_active &&
                !out_pending && !out_tvalid && dma_tx_idle) begin
                state                <= S_WAIT_CONFIG;
                out_pending          <= 1'b0;
                out_tvalid           <= 1'b0;
                out_sent             <= 1'b0;
                queued_tdata         <= 128'h0;
                queued_tkeepdw       <= 4'h0;
                queued_tlast         <= 1'b0;
                queued_tuser         <= 9'h0;
                queued_starts_mwr    <= 1'b0;
                mrd_tlp_open         <= 1'b0;
                mwr_tlp_open         <= 1'b0;
                mrd_tx_done_latched  <= 1'b0;
                mwr_tx_done_latched  <= 1'b0;
                // TX/status 配置仍有效时保留首次 NOW 挂起的状态写请求，避免启动死锁。
                if (!tx_config_ready) begin
                    force_status_pending <= 1'b0;
                    link_status_pending  <= 1'b0;
                    status_link_change   <= 1'b0;
                end
                // tag 隔离跨 teardown 保留，冷却到期前不得复用 0x1E。
                rx_wait_count        <= 0;
                tx_cons_idx          <= 0;
                rx_std_cons_idx      <= 0;
                rx_ret_prod_idx      <= 0;
                status_tag           <= 0;
                committed_status_tag <= 0;
            end
            else begin
                case (state)
                    S_WAIT_CONFIG: begin
                        if (!dma_runtime_ready) begin
                            tx_cons_idx     <= 0;
                            rx_std_cons_idx <= 0;
                            rx_ret_prod_idx <= 0;
                            status_tag      <= 0;
                            committed_status_tag <= 0;
                        end
                        else if (!dma_fault_latched) begin
                            state <= S_IDLE;
                        end
                    end

                    S_IDLE: begin
                        reply_type <= REPLY_NONE;
                        reply_vlan <= 1'b0;
                        if (!dma_runtime_ready) begin
                            state <= S_WAIT_CONFIG;
                        end
                        else if (force_status_pending ||
                                 link_status_pending) begin
                            state <= S_STATUS_PREP;
                        end
                        else if (tx_cons_idx != tx_prod_idx) begin
                            packet_total_bytes <= 0;
                            packet_overflow    <= 1'b0;
                            frame_ethertype    <= 0;
                            frame_ip_protocol  <= 0;
                            frame_arp_opcode   <= 0;
                            frame_arp_sender_mac <= 0;
                            frame_arp_sender_ip  <= 0;
                            frame_arp_target_ip  <= 0;
                            frame_udp_ports      <= 0;
                            frame_dhcp_xid       <= 0;
                            frame_dhcp_flags     <= 0;
                            frame_dhcp_chaddr    <= 0;
                            frame_dhcp_cookie    <= 0;
                            frame_dhcp_message_option <= 0;
                            frame_dst_mac        <= 0;
                            frame_outer_ethertype <= 0;
                            frame_vlan_tagged    <= 1'b0;
                            frame_ipv4_header_valid <= 1'b0;
                            frame_l3_offset      <= 11'd14;
                            frame_l4_offset      <= 0;
                            frame_bootp_offset   <= 0;
                            frame_dhcp_options_offset <= 0;
                            frame_dhcp_message_type <= 0;
                            frame_dhcp_message_type_valid <= 1'b0;
                            frame_dhcp_requested_ip <= 0;
                            frame_dhcp_server_id <= 0;
                            frame_dhcp_requested_ip_valid <= 1'b0;
                            frame_dhcp_server_id_valid <= 1'b0;
                            frame_vlan_tci <= 0;
                            frame_vlan_tpid <= 16'h8100;
                            frame_bootp_op <= 0; frame_bootp_htype <= 0; frame_bootp_hlen <= 0;
                            frame_bootp_ciaddr <= 0; frame_bootp_giaddr <= 0;
                            frame_dhcp_malformed <= 0; frame_dhcp_end_seen <= 0;
                            frame_dhcp_client_id_valid <= 0; frame_dhcp_client_id_length <= 0;
                            frame_dns_web_ok <= 1; frame_dns_legacy_ok <= 1;
                            frame_dns_web <= 0; frame_dns_legacy <= 0;
                            frame_src_mac        <= 0;
                            frame_ipv4_src       <= 0;
                            frame_ipv4_dst       <= 0;
                            frame_ipv4_tos       <= 0;
                            frame_ipv4_total_length <= 0;
                            frame_ipv4_identification <= 0;
                            frame_ipv4_flags_fragment <= 0;
                            frame_udp_length     <= 0;
                            frame_dns_flags      <= 0;
                            frame_dns_qdcount    <= 0;
                            frame_dns_qtype      <= 0;
                            frame_dns_qclass     <= 0;
                            frame_dns_question_end <= 0;
                            frame_dns_qname_done <= 1'b0;
                            frame_dns_ncsi_ok    <= 1'b1;
                            frame_dns_ncsi       <= 1'b0;
                            frame_tcp_seq        <= 0;
                            frame_tcp_ack        <= 0;
                            frame_tcp_data_offset <= 0;
                            frame_tcp_flags      <= 0;
                            frame_icmp_type      <= 0;
                            frame_icmp_code      <= 0;
                            frame_icmp_even_byte_sum <= 0;
                            frame_icmp_odd_byte_sum <= 0;
                            dhcp_scan_state      <= DHCP_SCAN_CODE;
                            dhcp_scan_code       <= 0;
                            dhcp_scan_length     <= 0;
                            dhcp_scan_remaining  <= 0;
                            dhcp_scan_index      <= 0;
                            state              <= S_TX_DESC_REQUEST;
                        end
                        else if (tcp_retransmit_due && rx_buffer_available && rx_return_available) begin
                            reply_vlan<=tcp_peer_vlan; reply_vlan_tci<=tcp_peer_tci; reply_vlan_tpid<=tcp_peer_tpid;
                            client_mac<=tcp_peer_mac; client_ipv4<=tcp_peer_ip; tcp_client_port<=tcp_peer_port;
                            service_ipv4<=tcp_peer_service_ipv4;
                            tcp_reply_sequence<=tcp_server_next_seq-http_reply_bytes-1;
                            tcp_reply_acknowledgment<=tcp_client_next_seq;
                            reply_type<=REPLY_HTTP;
                            reply_frame_length<=54+http_reply_bytes+(tcp_peer_vlan ? 4 : 0);
                            tcp_retry_age<=0; tcp_retry_count<=tcp_retry_count+1'b1;
                            state<=S_WAIT_RX_BUFFER;
                        end
                        else if (bg_due && rx_buffer_available &&
                                 rx_return_available) begin
                            // Status/link work and host TX always win over background traffic.
                            bg_source_slot     <= bg_lfsr[1:0];
                            bg_target_host     <=
                                8'd120 + {2'b00, bg_lfsr[9:4]};
                            if (ENABLE_BG_MDNS_LLMNR &&
                                (bg_lfsr[3:2] == 2'd2)) begin
                                reply_type         <= REPLY_BG_MDNS;
                                reply_frame_length <=
                                    (bg_frame_length < 16'd70) ?
                                        16'd70 : bg_frame_length;
                            end
                            else if (ENABLE_BG_MDNS_LLMNR &&
                                     (bg_lfsr[3:2] == 2'd3)) begin
                                reply_type         <= REPLY_BG_LLMNR;
                                reply_frame_length <=
                                    (bg_frame_length < 16'd64) ?
                                        16'd64 : bg_frame_length;
                            end
                            else begin
                                reply_type         <= REPLY_BG_ARP;
                                reply_frame_length <= bg_frame_length;
                            end
                            state              <= S_WAIT_RX_BUFFER;
                        end
                    end

                    S_TX_DESC_REQUEST: begin
                        if (!external_dma_tag_busy) begin
                            mrd_addr            <= tx_desc_current_addr;
                            mrd_dw_count        <= 10'd4;
                            mrd_first_be        <= 4'hF;
                            mrd_last_be         <= 4'hF;
                            mrd_kind            <= RD_TX_DESC;
                            mrd_store_base_byte <= 0;
                            mrd_expected_words  <= 10'd4;
                            mrd_requested_bytes <= 16'd16;
                            mrd_leading_bytes   <= 0;
                            after_mrd_state     <= S_TX_DESC_READY;
                            state               <= S_MRD_QUEUE;
                        end
                    end

                    S_RX_DESC_REQUEST: begin
                        if (!external_dma_tag_busy) begin
                            mrd_addr            <= rx_desc_current_addr;
                            mrd_dw_count        <= 10'd8;
                            mrd_first_be        <= 4'hF;
                            mrd_last_be         <= 4'hF;
                            mrd_kind            <= RD_RX_DESC;
                            mrd_store_base_byte <= 0;
                            mrd_expected_words  <= 10'd8;
                            mrd_requested_bytes <= 16'd32;
                            mrd_leading_bytes   <= 0;
                            after_mrd_state     <= S_RX_DESC_READY;
                            state               <= S_MRD_QUEUE;
                        end
                    end

                    S_MRD_QUEUE: begin
                        // 外部FIFO可能在上一拍刚取得授权，Tag到本拍才可见；
                        // 入队前再次确认，关闭该一拍竞争窗口。
                        // 禁用后不得再排新 MRd，避免陈旧环地址出站。
                        if (!out_pending && !external_dma_tag_busy &&
                            dma_may_issue_mrd) begin
                            mrd_tx_done_latched <= 1'b0;
                            queued_tdata <= {
                                mrd_addr[31:2], 2'b00,
                                mrd_addr[63:32],
                                pcie_id[7:0], pcie_id[15:8],
                                DMA_TAG,
                                (mrd_dw_count == 1) ? 4'h0 : mrd_last_be,
                                mrd_first_be,
                                22'h080000, mrd_dw_count
                            };
                            queued_tkeepdw <= 4'b1111;
                            queued_tlast   <= 1'b1;
                            queued_tuser   <= 9'b000000011;
                            queued_starts_mwr <= 1'b0;
                            // Completion接收器使用已入队MRd的冻结参数，
                            // 避免tx_packet_done延迟时采到下一笔/上一笔设置。
                            mrd_pending_kind <= mrd_kind;
                            mrd_pending_store_base_byte <=
                                mrd_store_base_byte;
                            mrd_pending_expected_words <=
                                mrd_expected_words;
                            mrd_pending_requested_bytes <=
                                mrd_requested_bytes;
                            mrd_pending_leading_bytes <=
                                mrd_leading_bytes;
                            mrd_pending_addr <= mrd_addr;
                            out_pending    <= 1'b1;
                            state          <= S_MRD_WAIT_SENT;
                        end
                    end

                    S_MRD_WAIT_SENT: begin
                        if (watchdog_may_abort) begin
                            dbg_mrd_tx_timeout_count <=
                                dbg_mrd_tx_timeout_count + 1'b1;
                            dbg_last_drop_reason <= 8'h07;
                            dma_fault_latched <= 1'b1;
                            out_pending <= 1'b0;
                            mrd_tlp_open <= 1'b0;
                            mrd_tx_done_latched <= 1'b0;
                            force_status_pending <= 1'b0;
                            state <= S_WAIT_CONFIG;
                        end
                        else if (out_sent)
                            state <= S_MRD_WAIT_TX_DONE;
                    end

                    S_MRD_WAIT_TX_DONE: begin
                        if (watchdog_may_abort) begin
                            dbg_mrd_tx_timeout_count <=
                                dbg_mrd_tx_timeout_count + 1'b1;
                            dbg_last_drop_reason <= 8'h07;
                            dma_fault_latched <= 1'b1;
                            out_pending <= 1'b0;
                            mrd_tlp_open <= 1'b0;
                            mrd_tx_done_latched <= 1'b0;
                            force_status_pending <= 1'b0;
                            state <= S_WAIT_CONFIG;
                        end
                        else if (mrd_tx_done_event ||
                            mrd_tx_done_latched) begin
                            mrd_tx_done_latched <= 1'b0;
                            state <= S_MRD_WAIT_CPL;
                        end
                    end

                    S_MRD_WAIT_CPL: begin
                        if (cpl_done)
                            state <= after_mrd_state;
                        else if (cpl_error) begin
                            // 超时后暂缓复用同一 Tag；冷却(fault_cooldown)
                            // 到期或驱动重新配置 DMA 环(HOSTCC.NOW)后恢复。
                            dma_fault_latched <= 1'b1;
                            case (cpl_error_reason)
                                3'd3: dbg_last_drop_reason <= 8'h09;
                                3'd2,
                                3'd4: dbg_last_drop_reason <= 8'h0A;
                                default:
                                    dbg_last_drop_reason <= 8'h06;
                            endcase
                            state <= S_WAIT_CONFIG;
                        end
                    end

                    S_TX_DESC_READY: begin
                        dbg_tx_desc_count    <= dbg_tx_desc_count + 1'b1;
                        dbg_last_tx_addr_low <= desc_words[1][1:0];
                        tx_segment_addr      <= {desc_words[0], desc_words[1]};
                        tx_segment_remaining <= desc_words[2][31:16];
                        tx_segment_flags     <= desc_words[2][15:0];

                        if ((desc_words[2][31:16] == 0) ||
                            packet_overflow ||
                            (({1'b0, packet_total_bytes} +
                              {1'b0, desc_words[2][31:16]}) >
                             MAX_TX_FRAME_BYTES)) begin
                            packet_overflow <= 1'b1;
                            dbg_last_drop_reason <= 8'h01;
                            state           <= S_TX_DROP_PROGRESS;
                        end
                        else begin
                            state <= S_TX_DATA_PREP;
                        end
                    end

                    S_TX_DATA_PREP: begin
                        if (!external_dma_tag_busy) begin
                            tx_chunk_bytes       <= tx_chunk_calculated;
                            mrd_addr             <= tx_segment_addr;
                            mrd_dw_count         <=
                                ({14'h0000, tx_segment_addr[1:0]} +
                                 tx_chunk_calculated + 3) >> 2;
                            mrd_first_be         <=
                                first_be_for_range(
                                    tx_segment_addr[1:0],
                                    tx_chunk_calculated);
                            mrd_last_be          <=
                                last_be_for_range(
                                    tx_segment_addr[1:0],
                                    tx_chunk_calculated);
                            mrd_kind             <= RD_TX_DATA;
                            mrd_store_base_byte  <= packet_total_bytes;
                            mrd_expected_words   <=
                                ({14'h0000, tx_segment_addr[1:0]} +
                                 tx_chunk_calculated + 3) >> 2;
                            mrd_requested_bytes  <= tx_chunk_calculated;
                            mrd_leading_bytes    <= tx_segment_addr[1:0];
                            after_mrd_state      <= S_TX_DATA_CHUNK_DONE;
                            state                <= S_MRD_QUEUE;
                        end
                    end

                    S_TX_DATA_CHUNK_DONE: begin
                        tx_chunk_drain_index <= 0;
                        state <= S_TX_DATA_DRAIN;
                    end

                    S_TX_DATA_DRAIN: begin
                        // Completion 先进入最多 32 个 DWORD 的小寄存器组，
                        // 再逐字节锁存关键字段，避免任何多写口帧 RAM。
                        store_tx_packet_fields(
                            tx_capture_write_offset,
                            tx_capture_write_data
                        );
                        if ((tx_chunk_drain_index + 1'b1) >=
                            tx_chunk_bytes) begin
                            tx_segment_addr <=
                                tx_segment_addr + tx_chunk_bytes;
                            tx_segment_remaining <=
                                tx_segment_remaining - tx_chunk_bytes;
                            packet_total_bytes <=
                                packet_total_bytes + tx_chunk_bytes;
                            state <= S_TX_SEG_PROGRESS;
                        end
                        else begin
                            tx_chunk_drain_index <=
                                tx_chunk_drain_index + 1'b1;
                        end
                    end

                    S_TX_SEG_PROGRESS: begin
                        if (tx_segment_remaining != 0) begin
                            state <= S_TX_DATA_PREP;
                        end
                        else begin
                            tx_cons_idx <=
                                ring_next(tx_cons_idx, tx_ring_size);
                            if (tx_segment_flags[2]) begin
                                dbg_tx_end_count <= dbg_tx_end_count + 1'b1;
                                if (!packet_overflow) begin
                                    tx_stat_event <= 1'b1;
                                    tx_stat_bytes <= packet_total_bytes;
                                    dbg_tx_packet_total <=
                                        dbg_tx_packet_total + 1'b1;
                                    if (frame_dst_mac == 48'hFFFFFFFFFFFF)
                                        tx_stat_class <= STAT_BROADCAST;
                                    else if (frame_dst_mac[40])
                                        tx_stat_class <= STAT_MULTICAST;
                                    else
                                        tx_stat_class <= STAT_UNICAST;
                                end
                                if (!PRODUCTION && !dbg_packet_candidate_frozen) begin
                                    dbg_packet_length <= packet_total_bytes;
                                    dbg_packet_flags <= tx_segment_flags;
                                    dbg_packet_tx_cons_idx <=
                                        ring_next(
                                            tx_cons_idx,
                                            tx_ring_size);
                                    dbg_packet_overflow <= packet_overflow;
                                    dbg_packet_valid <= 1'b1;
                                    dbg_packet_capture_count <=
                                        dbg_packet_capture_count + 1'b1;
                                    // 数据字节已在 Completion 到达时直接锁存。
                                    // 命中常见 DHCP 长度后冻结当前证据。
                                    if (!packet_overflow &&
                                        (packet_total_bytes >= 16'd280) &&
                                        (packet_total_bytes <= 16'd400))
                                        dbg_packet_candidate_frozen <= 1'b1;
                                end
                                state <= S_PARSE_CLASSIFY;
                            end
                            else if (ring_next(
                                         tx_cons_idx,
                                         tx_ring_size) == tx_prod_idx)
                                state <= S_TX_CHAIN_WAIT;
                            else
                                state <= S_TX_DESC_REQUEST;
                        end
                    end

                    S_TX_DROP_PROGRESS: begin
                        tx_cons_idx <= ring_next(tx_cons_idx, tx_ring_size);
                        if (tx_segment_flags[2]) begin
                            dbg_tx_end_count <= dbg_tx_end_count + 1'b1;
                            state <= S_STATUS_PREP;
                        end
                        else if (ring_next(
                                     tx_cons_idx,
                                     tx_ring_size) == tx_prod_idx)
                            state <= S_TX_CHAIN_WAIT;
                        else
                            state <= S_TX_DESC_REQUEST;
                    end

                    S_TX_CHAIN_WAIT: begin
                        // 非 END 描述符之后必须等 producer 推进，不能读取未提交项。
                        if (!dma_runtime_ready)
                            state <= S_WAIT_CONFIG;
                        else if (tx_cons_idx != tx_prod_idx)
                            state <= S_TX_DESC_REQUEST;
                    end

                    S_PARSE_CLASSIFY: begin
                        service_ipv4 <= EMULATE_REMOTE_NCSI ? frame_ipv4_dst : gw_ipv4;
                        reply_vlan <= frame_vlan_tagged;
                        reply_vlan_tci <= frame_vlan_tci;
                        reply_vlan_tpid <= frame_vlan_tpid;
                        // 对手动IP同样学习主机身份；DHCP的0.0.0.0不会误触发。
                        if ((frame_src_mac != 48'h000000000000) &&
                            (frame_src_mac != 48'hFFFFFFFFFFFF) &&
                            !frame_src_mac[40])
                            client_mac <= frame_src_mac;
                        if ((frame_ethertype == 16'h0800) &&
                            (frame_ipv4_src != 32'h00000000)) begin
                            client_ipv4 <= frame_ipv4_src;
                        end
                        else if ((frame_ethertype == 16'h0806) &&
                                 (frame_arp_sender_ip != 32'h00000000)) begin
                            client_ipv4 <= frame_arp_sender_ip;
                        end

                        if (!packet_overflow &&
                            (frame_dhcp_options_offset != 0) &&
                            (packet_total_bytes >
                             frame_dhcp_options_offset) &&
                            (frame_ethertype == 16'h0800) &&
                            frame_ipv4_header_valid &&
                            (frame_ip_protocol == 8'h11) &&
                            (frame_udp_ports == 32'h00440043) &&
                            (frame_dhcp_cookie == 32'h63825363) &&
                             frame_dhcp_message_type_valid &&
                             frame_dhcp_end_seen && !frame_dhcp_malformed &&
                             (frame_bootp_op == 8'd1) &&
                             (frame_bootp_htype == 8'd1) && (frame_bootp_hlen == 8'd6) &&
                             (frame_bootp_giaddr == 0) &&
                             ((frame_ipv4_flags_fragment & 16'h3FFF) == 0) &&
                             (frame_udp_length >= 16'd249) &&
                             (frame_ipv4_total_length ==
                              ((frame_l4_offset-frame_l3_offset)+frame_udp_length)) &&
                             ((frame_l3_offset+frame_ipv4_total_length) <= packet_total_bytes)) begin
                            dhcp_xid <= frame_dhcp_xid;
                             dhcp_flags <= frame_dhcp_flags;
                             dhcp_ciaddr <= frame_bootp_ciaddr;
                             dhcp_client_id_valid <= frame_dhcp_client_id_valid;
                             dhcp_client_id_length <= frame_dhcp_client_id_length;
                            // DHCP 以 BOOTP chaddr 为准，避免依赖硬件插入的以太源地址。
                            client_mac <= frame_dhcp_chaddr;
                            dhcp_option_offset      <=
                                frame_dhcp_options_offset[9:0];
                            dhcp_message_type       <=
                                frame_dhcp_message_type;
                            dhcp_requested_ip       <=
                                frame_dhcp_requested_ip;
                            dhcp_server_id          <=
                                frame_dhcp_server_id;
                            dhcp_requested_ip_valid <=
                                frame_dhcp_requested_ip_valid;
                            dhcp_server_id_valid    <=
                                frame_dhcp_server_id_valid;
                            state                   <= S_DHCP_OPTIONS;
                        end
                        else if (!packet_overflow &&
                                 frame_dns_qname_done &&
                                 (frame_dns_question_end >= 11'd59) &&
                                 (frame_dns_question_end <=
                                  ICMP_CAPTURE_BYTES) &&
                                 (packet_total_bytes >=
                                  frame_dns_question_end) &&
                                 (frame_ethertype == 16'h0800) &&
                                 frame_ipv4_header_valid &&
                                 ((frame_l3_offset == 11'd14) || (frame_l3_offset == 11'd18)) &&
                                 (frame_l4_offset == frame_l3_offset+11'd20) &&
                                 (frame_ip_protocol == 8'h11) &&
                                 ((frame_ipv4_dst == gw_ipv4) || (EMULATE_REMOTE_NCSI && frame_ipv4_dst!=0 && frame_ipv4_dst!=32'hFFFFFFFF && frame_ipv4_dst[31:28]<4'hE)) &&
                                 ((frame_ipv4_flags_fragment &
                                   16'h3FFF) == 0) &&
                                 ((frame_ipv4_total_length + frame_l3_offset) <=
                                  packet_total_bytes) &&
                                 (frame_udp_ports[15:0] == 16'd53) &&
                                 (frame_udp_length >=
                                  (frame_dns_question_end - frame_l4_offset)) &&
                                 !frame_dns_flags[15] &&
                                 (frame_dns_flags[14:11] == 4'h0) &&
                                 (frame_dns_qdcount == 16'd1) &&
                                 ((frame_dns_qtype == 16'd1) || (frame_dns_qtype == 16'd28)) &&
                                 (frame_dns_qclass == 16'd1)) begin
                            client_mac          <= frame_src_mac;
                            client_ipv4         <= frame_ipv4_src;
                             dns_question_end <= frame_dns_question_end[9:0]- (frame_vlan_tagged ? 10'd4 : 10'd0);
                             dns_rcode <= (frame_dns_ncsi || frame_dns_web || frame_dns_legacy) ? 0 : 3;
                             dns_answer_count <= (frame_dns_qtype == 1) ?
                                (frame_dns_ncsi ? 5 : ((frame_dns_web || frame_dns_legacy) ? 1 : 0)) : 0;
                            dns_ncsi            <= frame_dns_ncsi;
                            reply_ipv4_identification <=
                                frame_ipv4_identification + 1'b1;
                            reply_type          <= REPLY_DNS;
                            reply_frame_length  <=
                                 frame_dns_question_end + ((frame_dns_qtype == 1) ?
                                 (frame_dns_ncsi ? 11'd80 : ((frame_dns_web || frame_dns_legacy) ? 11'd16 : 11'd0)) : 11'd0);
                            dbg_classified_count <=
                                dbg_classified_count + 1'b1;
                            state               <= S_WAIT_RX_BUFFER;
                        end
                        else if (ENABLE_TCP_HTTP && !packet_overflow && frame_ethertype==16'h0800 &&
                            frame_ipv4_header_valid && (frame_l3_offset==14 || frame_l3_offset==18) &&
                            frame_l4_offset==frame_l3_offset+20 && frame_ip_protocol==6 &&
                            (frame_ipv4_dst==gw_ipv4 || (EMULATE_REMOTE_NCSI && frame_ipv4_dst!=0 && frame_ipv4_dst!=32'hFFFFFFFF && frame_ipv4_dst[31:28]<4'hE)) && (frame_ipv4_flags_fragment & 16'h3FFF)==0 &&
                            frame_tcp_data_offset>=5 && frame_ipv4_total_length>=20+frame_tcp_header_bytes &&
                            frame_ipv4_total_length+frame_l3_offset<=packet_total_bytes &&
                            frame_udp_ports[15:0]==80) begin
                            client_mac <= frame_src_mac; client_ipv4 <= frame_ipv4_src;
                            tcp_client_port <= frame_udp_ports[31:16];
                            reply_ipv4_identification <= frame_ipv4_identification+1'b1;
                            tcp_reply_sequence <= frame_tcp_ack;
                            tcp_reply_acknowledgment <= frame_tcp_seq+frame_tcp_payload_length+frame_tcp_flags[0];
                            reply_frame_length <= 60+(frame_vlan_tagged ? 4 : 0);
                            reply_type <= REPLY_NONE;
                            state <= S_STATUS_PREP;
                            if (frame_tcp_flags[2]) begin
                                if (tcp_peer_match) begin tcp_session_active<=0; tcp_response_pending<=0; end
                            end else if ((frame_tcp_flags & 8'h17)==8'h02 && frame_tcp_payload_length==0) begin
                                if (!tcp_session_active || (tcp_peer_match && !tcp_session_established &&
                                    frame_tcp_seq+1==tcp_client_next_seq)) begin
                                    tcp_session_active<=1; tcp_session_established<=0; tcp_fin_sent<=0; tcp_client_fin_seen<=0;
                                    tcp_peer_mac<=frame_src_mac; tcp_peer_ip<=frame_ipv4_src;
                                    tcp_peer_service_ipv4<=EMULATE_REMOTE_NCSI ? frame_ipv4_dst : gw_ipv4;
                                    tcp_peer_port<=frame_udp_ports[31:16];
                                    tcp_peer_vlan<=frame_vlan_tagged; tcp_peer_tci<=frame_vlan_tci; tcp_peer_tpid<=frame_vlan_tpid;
                                    tcp_client_next_seq<=frame_tcp_seq+1; tcp_server_next_seq<=TCP_INITIAL_SEQUENCE+1;
                                    tcp_reply_sequence<=TCP_INITIAL_SEQUENCE; tcp_reply_acknowledgment<=frame_tcp_seq+1;
                                    tcp_session_age<=0; tcp_response_pending<=0;
                                    tcp_first_line<=1; tcp_modern_match<=1; tcp_legacy_match<=1;
                                    tcp_path_modern<=0; tcp_path_legacy<=0; tcp_http_10<=0;
                                    tcp_host_modern_seen<=0; tcp_host_legacy_seen<=0;
                                    tcp_host_prefix_match<=1; tcp_host_started<=0; tcp_host_tail<=0;
                                    tcp_host_modern_match<=1; tcp_host_legacy_match<=1;
                                    tcp_http_bytes<=0; tcp_line_index<=0; tcp_header_index<=0; tcp_host_index<=0; tcp_header_tail<=0;
                                    reply_type<=REPLY_TCP_SYNACK; state<=S_WAIT_RX_BUFFER;
                                end else begin reply_type<=REPLY_TCP_RST; state<=S_WAIT_RX_BUFFER; end
                            end else if (!tcp_session_active && tcp_peer_tuple_match && frame_tcp_flags==8'h11 &&
                                         frame_tcp_seq+1==tcp_client_next_seq && frame_tcp_ack==tcp_server_next_seq) begin
                                tcp_reply_sequence<=tcp_server_next_seq; tcp_reply_acknowledgment<=tcp_client_next_seq;
                                reply_type<=REPLY_TCP_ACK; state<=S_WAIT_RX_BUFFER;
                            end else if (tcp_peer_match && frame_tcp_flags[4] && !frame_tcp_flags[1] &&
                                         frame_tcp_ack<=tcp_server_next_seq && frame_tcp_ack>=TCP_INITIAL_SEQUENCE+1) begin
                                tcp_session_established<=1; tcp_session_age<=0;
                                if (frame_tcp_ack==tcp_server_next_seq) begin
                                    tcp_response_pending<=0;
                                    if (tcp_fin_sent && tcp_client_fin_seen) tcp_session_active<=0;
                                end
                                tcp_reply_sequence<=tcp_server_next_seq;
                                tcp_reply_acknowledgment<=tcp_client_next_seq;
                                if (frame_tcp_seq != tcp_client_next_seq) begin
                                    reply_type<=REPLY_TCP_ACK; state<=S_WAIT_RX_BUFFER;
                                    if (tcp_response_pending && frame_tcp_ack!=tcp_server_next_seq) begin
                                        reply_type<=REPLY_HTTP;
                                        tcp_reply_sequence<=tcp_server_next_seq-http_reply_bytes-1;
                                        reply_frame_length<=54+http_reply_bytes+(frame_vlan_tagged ? 4 : 0);
                                    end
                                end else if (frame_tcp_payload_length!=0 && !tcp_fin_sent) begin
                                    if (frame_tcp_payload_length>TCP_REQUEST_MAX_BYTES ||
                                        tcp_http_bytes+frame_tcp_payload_length>TCP_REQUEST_MAX_BYTES) begin
                                        reply_type<=REPLY_TCP_RST; tcp_session_active<=0; tcp_response_pending<=0;
                                        state<=S_WAIT_RX_BUFFER;
                                    end else begin tcp_scan_index<=0; state<=S_TCP_REQUEST_SCAN; end
                                end else if (frame_tcp_flags[0]) begin
                                    tcp_client_next_seq<=tcp_client_next_seq+1;
                                    tcp_reply_acknowledgment<=tcp_client_next_seq+1;
                                    tcp_client_fin_seen<=1;
                                    if (tcp_fin_sent && frame_tcp_ack==tcp_server_next_seq) tcp_session_active<=0;
                                    if (!tcp_fin_sent) begin
                                        reply_type<=REPLY_TCP_FIN; tcp_fin_sent<=1;
                                        tcp_server_next_seq<=tcp_server_next_seq+1;
                                    end else reply_type<=REPLY_TCP_ACK;
                                    state<=S_WAIT_RX_BUFFER;
                                end
                            end else begin reply_type<=REPLY_TCP_RST; state<=S_WAIT_RX_BUFFER; end
                        end
                        else if (!packet_overflow &&
                                 (packet_total_bytes >= 16'd42) &&
                                 (packet_total_bytes <=
                                  ICMP_CAPTURE_BYTES) &&
                                 (frame_ethertype == 16'h0800) &&
                                 frame_ipv4_header_valid &&
                                 ((frame_l3_offset == 11'd14) || (frame_l3_offset == 11'd18)) &&
                                 (frame_l4_offset == frame_l3_offset+11'd20) &&
                                 (frame_ip_protocol == 8'h01) &&
                                 (frame_ipv4_dst == gw_ipv4) &&
                                 ((frame_ipv4_flags_fragment &
                                   16'h3FFF) == 0) &&
                                 (frame_ipv4_total_length >= 16'd28) &&
                                 ((frame_ipv4_total_length + frame_l3_offset) <=
                                  packet_total_bytes) &&
                                 (frame_icmp_type == 8'h08) &&
                                 (frame_icmp_code == 8'h00)) begin
                            client_mac           <= frame_src_mac;
                            client_ipv4          <= frame_ipv4_src;
                             icmp_l3_offset <= 14;
                             icmp_l4_offset <= 34;
                            icmp_reply_message_checksum <=
                                icmp_checksum_from_byte_sums(
                                    frame_icmp_even_byte_sum,
                                    frame_icmp_odd_byte_sum);
                            icmp_reply_ip_checksum <=
                                ipv4_reply_header_checksum(
                                    {8'h45, frame_ipv4_tos},
                                    frame_ipv4_total_length,
                                    frame_ipv4_identification,
                                    frame_ipv4_flags_fragment,
                                    16'h4001,
                                    frame_ipv4_src);
                             icmp_request_length <= packet_total_bytes-(frame_vlan_tagged ? 16'd4 : 16'd0);
                            reply_type           <= REPLY_ICMP;
                            reply_frame_length   <= packet_total_bytes;
                            dbg_classified_count <=
                                dbg_classified_count + 1'b1;
                            state                <= S_WAIT_RX_BUFFER;
                        end
                        else if (!packet_overflow &&
                                 (packet_total_bytes >= 16'd42) &&
                                 (frame_ethertype == 16'h0806) &&
                                 (frame_arp_opcode == 16'h0001) &&
                                 (frame_arp_target_ip ==
                                  gw_ipv4)) begin
                            // ARP 回复使用请求中的发送端硬件地址。
                            client_mac         <= frame_arp_sender_mac;
                            client_ipv4        <= frame_arp_sender_ip;
                            reply_type         <= REPLY_ARP;
                            reply_frame_length <= 16'd60+(frame_vlan_tagged ? 16'd4 : 16'd0);
                            dbg_classified_count <=
                                dbg_classified_count + 1'b1;
                            dbg_arp_count <= dbg_arp_count + 1'b1;
                            state              <= S_WAIT_RX_BUFFER;
                        end
                        else begin
                            dbg_last_drop_reason <= 8'h02;
                            state <= S_STATUS_PREP;
                        end
                    end

                    S_TCP_REQUEST_SCAN: begin
                        tcp_header_tail <= {tcp_header_tail[23:0],tcp_scan_byte};
                        tcp_http_bytes <= tcp_http_bytes+1'b1;
                        if (tcp_first_line) begin
                            if (tcp_line_index != 28 || (tcp_scan_byte!=8'h30 && tcp_scan_byte!=8'h31))
                                if (tcp_scan_byte!=http_modern_line_byte(tcp_line_index)) tcp_modern_match<=0;
                            if (tcp_line_index != 21 || (tcp_scan_byte!=8'h30 && tcp_scan_byte!=8'h31))
                                if (tcp_scan_byte!=http_legacy_line_byte(tcp_line_index)) tcp_legacy_match<=0;
                            if ((tcp_line_index==28 || tcp_line_index==21) && tcp_scan_byte==8'h30) tcp_http_10<=1;
                            tcp_line_index<=tcp_line_index+1'b1;
                            if (tcp_header_tail[7:0]==8'h0D && tcp_scan_byte==8'h0A) begin
                                tcp_first_line<=0;
                                tcp_path_modern<=tcp_modern_match && tcp_line_index==30;
                                tcp_path_legacy<=tcp_legacy_match && tcp_line_index==23;
                                tcp_header_index<=0; tcp_host_index<=0;
                            end
                        end else begin
                            tcp_header_index<=tcp_header_index+1'b1;
                            if (tcp_scan_byte!=8'h0D && tcp_scan_byte!=8'h0A) begin
                                if (tcp_header_index<5) begin
                                    if (dns_ascii_lower(tcp_scan_byte)!=http_host_prefix_byte(tcp_header_index)) tcp_host_prefix_match<=0;
                                end else if (tcp_host_prefix_match) begin
                                    if (!tcp_host_started && (tcp_scan_byte==8'h20 || tcp_scan_byte==8'h09)) begin end
                                    else if (tcp_scan_byte==8'h20 || tcp_scan_byte==8'h09) tcp_host_tail<=1;
                                    else begin
                                        tcp_host_started<=1;
                                        if (tcp_host_tail || dns_ascii_lower(tcp_scan_byte)!=http_modern_host_byte(tcp_host_index)) tcp_host_modern_match<=0;
                                        if (tcp_host_tail || dns_ascii_lower(tcp_scan_byte)!=http_legacy_host_byte(tcp_host_index)) tcp_host_legacy_match<=0;
                                        tcp_host_index<=tcp_host_index+1'b1;
                                    end
                                end
                            end
                            if (tcp_header_tail[7:0]==8'h0D && tcp_scan_byte==8'h0A) begin
                                if (tcp_host_prefix_match && tcp_host_started) begin
                                    if (tcp_host_modern_match && (tcp_host_index==23 || tcp_host_index==26)) tcp_host_modern_seen<=1;
                                    if (tcp_host_legacy_match && (tcp_host_index==16 || tcp_host_index==19)) tcp_host_legacy_seen<=1;
                                end
                                tcp_header_index<=0; tcp_host_index<=0; tcp_host_prefix_match<=1;
                                tcp_host_started<=0; tcp_host_tail<=0; tcp_host_modern_match<=1; tcp_host_legacy_match<=1;
                            end
                        end
                        if ({tcp_header_tail[23:0],tcp_scan_byte}==32'h0D0A0D0A) begin
                            tcp_client_next_seq<=frame_tcp_seq+frame_tcp_payload_length+frame_tcp_flags[0];
                            tcp_reply_acknowledgment<=frame_tcp_seq+frame_tcp_payload_length+frame_tcp_flags[0];
                            tcp_reply_sequence<=tcp_server_next_seq;
                            if ((tcp_path_modern && (tcp_host_modern_seen || tcp_http_10)) ||
                                (tcp_path_legacy && (tcp_host_legacy_seen || tcp_http_10))) begin
                                http_legacy_reply<=tcp_path_legacy;
                                reply_type<=REPLY_HTTP;
                                reply_frame_length<=54+(tcp_path_legacy ? 72 : 80)+reply_vlan_bytes;
                                tcp_server_next_seq<=tcp_server_next_seq+(tcp_path_legacy ? 72 : 80)+1;
                                tcp_client_fin_seen<=frame_tcp_flags[0];
                                tcp_fin_sent<=1; tcp_response_pending<=1; tcp_retry_count<=0; tcp_retry_age<=0;
                            end else begin
                                reply_type<=REPLY_TCP_RST; reply_frame_length<=60+reply_vlan_bytes;
                                tcp_session_active<=0; tcp_response_pending<=0;
                            end
                            state<=S_WAIT_RX_BUFFER;
                        end else if (tcp_scan_index+1>=frame_tcp_payload_length) begin
                            tcp_client_next_seq<=frame_tcp_seq+frame_tcp_payload_length+frame_tcp_flags[0];
                            tcp_reply_acknowledgment<=frame_tcp_seq+frame_tcp_payload_length+frame_tcp_flags[0];
                            tcp_reply_sequence<=tcp_server_next_seq;
                            if (frame_tcp_flags[0]) begin
                                reply_type<=REPLY_TCP_RST; tcp_session_active<=0; tcp_response_pending<=0;
                            end else reply_type<=REPLY_TCP_ACK;
                            reply_frame_length<=60+reply_vlan_bytes;
                            state<=S_WAIT_RX_BUFFER;
                        end else tcp_scan_index<=tcp_scan_index+1'b1;
                    end

                    S_DHCP_OPTIONS: begin
                        if ((dhcp_message_type == 8'd1) && (dhcp_ciaddr == 0)) begin
                            reply_type <= REPLY_DHCP_OFFER;
                            reply_frame_length <= dhcp_frame_bytes(0,dhcp_client_id_valid,dhcp_client_id_length)+reply_vlan_bytes;
                            dbg_dhcp_discover_count <= dbg_dhcp_discover_count+1'b1;
                            dbg_classified_count <= dbg_classified_count+1'b1;
                            state <= S_DHCP_CSUM_INIT;
                        end else if (dhcp_message_type == 8'd3) begin
                            if (dhcp_server_id_valid && dhcp_server_id != gw_ipv4) begin
                                reply_type <= REPLY_NONE;
                                state <= S_STATUS_PREP;
                            end else if ((dhcp_server_id_valid && (!dhcp_requested_ip_valid || dhcp_ciaddr != 0)) ||
                                         (!dhcp_server_id_valid && dhcp_requested_ip_valid && dhcp_ciaddr != 0) ||
                                         (!dhcp_server_id_valid && !dhcp_requested_ip_valid && dhcp_ciaddr == 0)) begin
                                reply_type <= REPLY_NONE;
                                state <= S_STATUS_PREP;
                            end else begin
                                if ((dhcp_requested_ip_valid && dhcp_requested_ip != lease_ipv4) ||
                                    (dhcp_ciaddr != 0 && dhcp_ciaddr != lease_ipv4)) begin
                                    reply_type <= REPLY_DHCP_NAK;
                                    dhcp_flags <= 16'h8000;
                                    dhcp_ciaddr <= 0;
                                    reply_frame_length <= dhcp_frame_bytes(1,dhcp_client_id_valid,dhcp_client_id_length)+reply_vlan_bytes;
                                end else begin
                                    reply_type <= REPLY_DHCP_ACK;
                                    client_ipv4 <= lease_ipv4;
                                    reply_frame_length <= dhcp_frame_bytes(0,dhcp_client_id_valid,dhcp_client_id_length)+reply_vlan_bytes;
                                end
                                dbg_dhcp_request_count <= dbg_dhcp_request_count+1'b1;
                                dbg_classified_count <= dbg_classified_count+1'b1;
                                state <= S_DHCP_CSUM_INIT;
                            end
                        end else begin
                            reply_type <= REPLY_NONE;
                            state <= S_STATUS_PREP;
                        end
                    end

                    S_DHCP_CSUM_INIT: begin
                        dhcp_checksum_running <= 1;
                        dhcp_checksum_index <= 34;
                        dhcp_checksum_sum <= {16'h0,gw_ipv4[31:16]}+{16'h0,gw_ipv4[15:0]}+
                            ((reply_type == REPLY_DHCP_NAK || (dhcp_ciaddr == 0 && dhcp_flags[15])) ? 32'h1FFFE :
                             ((dhcp_ciaddr != 0) ? ({16'h0,dhcp_ciaddr[31:16]}+{16'h0,dhcp_ciaddr[15:0]}) :
                              ({16'h0,lease_ipv4[31:16]}+{16'h0,lease_ipv4[15:0]})))+
                            32'd17+reply_core_length-16'd34;
                        state <= S_DHCP_CSUM_BYTE;
                    end
                    S_DHCP_CSUM_BYTE: begin
                        dhcp_checksum_byte_q <= dhcp_udp_payload_byte(dhcp_checksum_index);
                        state <= S_DHCP_CSUM;
                    end
                    S_DHCP_CSUM: begin
                        dhcp_checksum_sum <= dhcp_checksum_next_sum;
                        if (dhcp_checksum_index+1 >= reply_core_length)
                            state <= S_DHCP_CSUM_FOLD;
                        else begin
                            dhcp_checksum_index <= dhcp_checksum_index+1'b1;
                            state <= S_DHCP_CSUM_BYTE;
                        end
                    end
                    S_DHCP_CSUM_FOLD: begin
                        dhcp_reply_udp_checksum <= (internet_checksum(dhcp_checksum_sum)==0) ?
                            16'hFFFF : internet_checksum(dhcp_checksum_sum);
                        dhcp_checksum_running <= 0;
                        state <= S_WAIT_RX_BUFFER;
                    end

                    S_WAIT_RX_BUFFER: begin
                        if (!dma_runtime_ready) begin
                            state <= S_WAIT_CONFIG;
                        end
                        else if (rx_buffer_available &&
                                 rx_return_available) begin
                            state <= S_RX_DESC_REQUEST;
                        end
                        else if (&rx_wait_count) begin
                            // RX 环暂时没有资源时丢弃模拟回复，但仍回写 TX 状态。
                            reply_type <= REPLY_NONE;
                            dbg_rx_reject_count <=
                                dbg_rx_reject_count + 1'b1;
                            dbg_last_drop_reason <= 8'h04;
                            state      <= S_STATUS_PREP;
                        end
                        else begin
                            rx_wait_count <= rx_wait_count + 1'b1;
                        end
                    end

                    S_RX_DESC_READY: begin
                        dbg_last_rx_addr_low <= desc_words[1][1:0];
                        rx_buffer_addr     <= {desc_words[0], desc_words[1]};
                        rx_buffer_capacity <= desc_words[2][15:0];
                        // 实卡把生产描述符的索引字段原样带到返回描述符。
                        rx_buffer_index    <= desc_words[2][31:16];
                        rx_buffer_opaque   <= desc_words[7];

                        if (desc_words[2][15:0] <
                            (reply_frame_length + 16'd4)) begin
                            dbg_rx_reject_count <=
                                dbg_rx_reject_count + 1'b1;
                            dbg_last_drop_reason <= 8'h05;
                            state <= S_STATUS_PREP;
                        end
                        else begin
                            dbg_rx_accept_count <=
                                dbg_rx_accept_count + 1'b1;
                            mwr_kind         <= WR_REPLY;
                            mwr_base_addr    <= {desc_words[0], desc_words[1]};
                            mwr_total_bytes  <= reply_frame_length;
                            mwr_offset_bytes <= 0;
                            after_mwr_state  <= S_REPLY_WRITTEN;
                            state            <= S_MWR_PREP;
                        end
                    end

                    S_REPLY_WRITTEN: begin
                        dbg_reply_commit_count <=
                            dbg_reply_commit_count + 1'b1;
                        if (!PRODUCTION &&
                            (reply_type == REPLY_DHCP_OFFER) &&
                            !dbg_offer_frozen) begin
                            dbg_offer_bytes_done <= 1'b1;
                            dbg_offer_pending <= 1'b1;
                            dbg_packet_length <= reply_frame_length;
                            dbg_packet_flags <= 16'h0001;
                            dbg_packet_valid <= 1'b1;
                        end
                        if (reply_type == REPLY_DHCP_ACK)
                            dhcp_acked <= 1'b1;
                        rx_stat_event <= 1'b1;
                        rx_stat_bytes <= reply_frame_length;
                        dbg_rx_packet_total <=
                            dbg_rx_packet_total + 1'b1;
                        if (reply_type == REPLY_BG_ARP)
                            rx_stat_class <= STAT_BROADCAST;
                        else if ((reply_type == REPLY_BG_MDNS) ||
                                 (reply_type == REPLY_BG_LLMNR))
                            rx_stat_class <= STAT_MULTICAST;
                        else if ((reply_type == REPLY_DHCP_OFFER) ||
                                 (reply_type == REPLY_DHCP_ACK) || (reply_type == REPLY_DHCP_NAK))
                             rx_stat_class <= ((reply_type == REPLY_DHCP_NAK) || (dhcp_ciaddr == 0 && dhcp_flags[15])) ?
                                STAT_BROADCAST : STAT_UNICAST;
                        else
                            rx_stat_class <= STAT_UNICAST;
                        state <= S_RX_DESC_WRITE;
                    end

                    S_RX_DESC_WRITE: begin
                        if (!PRODUCTION && dbg_offer_pending)
                            dbg_offer_ret_addr <=
                                rx_ret_current_addr[31:0];
                        mwr_kind         <= WR_RX_DESC;
                        mwr_base_addr    <= rx_ret_current_addr;
                        mwr_total_bytes  <= 16'd32;
                        mwr_offset_bytes <= 0;
                        after_mwr_state  <= S_RX_DESC_WRITTEN;
                        state            <= S_MWR_PREP;
                    end

                    S_RX_DESC_WRITTEN: begin
                        rx_std_cons_idx <=
                            ring_next(rx_std_cons_idx, rx_std_ring_size);
                        rx_ret_prod_idx <=
                            ring_next(rx_ret_prod_idx, rx_ret_ring_size);
                        state <= S_STATUS_PREP;
                    end

                    S_STATUS_PREP: begin
                        // 锁存本次状态块类型，保证单周期 link_event 不会丢失。
                        status_link_change <= link_status_pending;
                        if (status_addr == 0) begin
                            state <= S_IDLE;
                        end
                        else begin
                            status_tag <= status_tag + 1'b1;
                            state      <= S_STATUS_WRITE;
                        end
                    end

                    S_STATUS_WRITE: begin
                        mwr_kind         <= WR_STATUS;
                        mwr_base_addr    <= status_addr;
                        // HOSTCC_MODE 使用 32BYTE 模式；DW5-DW7 为未启用队列写零。
                        mwr_total_bytes  <= 16'd32;
                        mwr_offset_bytes <= 0;
                        after_mwr_state  <= S_STATUS_WRITTEN;
                        state            <= S_MWR_PREP;
                    end

                    S_MWR_PREP: begin
                        // 每个 chunk 都是新 TLP。禁用后不得开新包；
                        // 已打开的 MWr 仍由 DATA_QUEUE 发到 tlast。
                        if (!out_pending && dma_runtime_ready) begin
                            mwr_tx_done_latched <= 1'b0;
                            mwr_chunk_bytes <= mwr_chunk_calculated;
                            mwr_chunk_words <=
                                mwr_chunk_words_calculated;
                            mwr_words_sent <= 0;
                            mwr_beat_build_index <= 0;
                            mwr_build_settle <= 1'b0;
                            mwr_beat_build_q <= 0;
                            mwr_beat_word0 <= 0;
                            mwr_beat_word1 <= 0;
                            mwr_beat_word2 <= 0;
                            queued_tdata <= {
                                mwr_tlp_addr[31:2], 2'b00,
                                mwr_tlp_addr[63:32],
                                pcie_id[7:0], pcie_id[15:8],
                                8'h00,
                                (mwr_chunk_words_calculated == 1) ?
                                    4'h0 : mwr_last_be_calculated,
                                mwr_first_be_calculated,
                                22'h180000,
                                mwr_chunk_words_calculated
                            };
                            queued_tkeepdw <= 4'b1111;
                            queued_tlast   <= 1'b0;
                            queued_tuser   <= 9'b000000001;
                            queued_starts_mwr <= 1'b1;
                            out_pending    <= 1'b1;
                            state          <= S_MWR_HDR_WAIT;
                        end
                    end

                    S_MWR_HDR_WAIT: begin
                        if (watchdog_may_abort) begin
                            dbg_mwr_tx_timeout_count <=
                                dbg_mwr_tx_timeout_count + 1'b1;
                            dbg_last_drop_reason <= 8'h08;
                            dma_fault_latched <= 1'b1;
                            out_pending <= 1'b0;
                            mwr_tlp_open <= 1'b0;
                            mwr_tx_done_latched <= 1'b0;
                            force_status_pending <= 1'b0;
                            state <= S_WAIT_CONFIG;
                        end
                        else if (out_sent)
                            state <= S_MWR_DATA_QUEUE;
                    end

                    S_MWR_DATA_QUEUE: begin
                        if (!out_pending) begin
                            // reply_byte 组合云是全设计最深路径，且 clk_pcie 域未被
                            // 时序约束（见 FIDELITY_PLAN T13）。每个 payload 字给两
                            // 个周期：首拍让云稳定、不采样，次拍再锁存，避免采到未
                            // 稳定的中间值导致主机侧帧内容损坏。
                            if (!mwr_build_settle) begin
                                mwr_build_settle <= 1'b1;
                                mwr_beat_build_q <= mwr_beat_build_word;
                            end
                            else begin
                                mwr_build_settle <= 1'b0;
                            if (dbg_offer_capture_en) begin
                                if (dbg_offer_src_base < 16'd48)
                                    dbg_packet_bytes[dbg_offer_src_base[5:0]] <=
                                        mwr_beat_build_q[31:24];
                                else if ((dbg_offer_src_base >= 16'd278) &&
                                         (dbg_offer_src_base <= 16'd285))
                                    dbg_packet_bytes[6'd48 +
                                        (dbg_offer_src_base - 16'd278)] <=
                                        mwr_beat_build_q[31:24];
                                if ((dbg_offer_src_base + 16'd1) < 16'd48)
                                    dbg_packet_bytes[dbg_offer_src_base[5:0] + 6'd1] <=
                                        mwr_beat_build_q[23:16];
                                else if (((dbg_offer_src_base + 16'd1) >= 16'd278) &&
                                         ((dbg_offer_src_base + 16'd1) <= 16'd285))
                                    dbg_packet_bytes[6'd48 +
                                        (dbg_offer_src_base + 16'd1 - 16'd278)] <=
                                        mwr_beat_build_q[23:16];
                                if ((dbg_offer_src_base + 16'd2) < 16'd48)
                                    dbg_packet_bytes[dbg_offer_src_base[5:0] + 6'd2] <=
                                        mwr_beat_build_q[15:8];
                                else if (((dbg_offer_src_base + 16'd2) >= 16'd278) &&
                                         ((dbg_offer_src_base + 16'd2) <= 16'd285))
                                    dbg_packet_bytes[6'd48 +
                                        (dbg_offer_src_base + 16'd2 - 16'd278)] <=
                                        mwr_beat_build_q[15:8];
                                if ((dbg_offer_src_base + 16'd3) < 16'd48)
                                    dbg_packet_bytes[dbg_offer_src_base[5:0] + 6'd3] <=
                                        mwr_beat_build_q[7:0];
                                else if (((dbg_offer_src_base + 16'd3) >= 16'd278) &&
                                         ((dbg_offer_src_base + 16'd3) <= 16'd285))
                                    dbg_packet_bytes[6'd48 +
                                        (dbg_offer_src_base + 16'd3 - 16'd278)] <=
                                        mwr_beat_build_q[7:0];
                            end
                            case (mwr_beat_build_index)
                                2'd0: mwr_beat_word0 <=
                                          mwr_beat_build_q;
                                2'd1: mwr_beat_word1 <=
                                          mwr_beat_build_q;
                                2'd2: mwr_beat_word2 <=
                                          mwr_beat_build_q;
                                default: begin
                                end
                            endcase

                            if (({1'b0, mwr_beat_build_index} + 3'd1) >=
                                mwr_words_next_beat) begin
                                case (mwr_beat_build_index)
                                    2'd0: queued_tdata <= {
                                        96'h0, mwr_beat_build_q
                                    };
                                    2'd1: queued_tdata <= {
                                        64'h0, mwr_beat_build_q,
                                        mwr_beat_word0
                                    };
                                    2'd2: queued_tdata <= {
                                        32'h0, mwr_beat_build_q,
                                        mwr_beat_word1, mwr_beat_word0
                                    };
                                    default: queued_tdata <= {
                                        mwr_beat_build_q,
                                        mwr_beat_word2,
                                        mwr_beat_word1,
                                        mwr_beat_word0
                                    };
                                endcase
                                queued_tkeepdw <= mwr_keep_next_beat;
                                queued_tlast <=
                                    (mwr_words_next_beat ==
                                     mwr_words_left);
                                queued_tuser <= {
                                    7'h00,
                                    (mwr_words_next_beat ==
                                     mwr_words_left),
                                    1'b0
                                };
                                queued_starts_mwr <= 1'b0;
                                mwr_words_this_beat <=
                                    mwr_words_next_beat;
                                mwr_beat_build_index <= 0;
                                out_pending <= 1'b1;
                                state       <= S_MWR_DATA_WAIT;
                            end
                            else begin
                                mwr_beat_build_index <=
                                    mwr_beat_build_index + 1'b1;
                            end
                            end
                        end
                    end

                    S_MWR_DATA_WAIT: begin
                        if (watchdog_may_abort) begin
                            dbg_mwr_tx_timeout_count <=
                                dbg_mwr_tx_timeout_count + 1'b1;
                            dbg_last_drop_reason <= 8'h08;
                            dma_fault_latched <= 1'b1;
                            out_pending <= 1'b0;
                            mwr_tlp_open <= 1'b0;
                            mwr_tx_done_latched <= 1'b0;
                            force_status_pending <= 1'b0;
                            state <= S_WAIT_CONFIG;
                        end
                        else if (out_sent) begin
                            if ((mwr_words_sent +
                                 mwr_words_this_beat) >=
                                mwr_chunk_words) begin
                                if (mwr_tx_done_event ||
                                    mwr_tx_done_latched) begin
                                    mwr_tx_done_latched <= 1'b0;
                                    mwr_offset_bytes <=
                                        mwr_offset_bytes +
                                        mwr_chunk_bytes;
                                    state <= S_MWR_CHUNK_DONE;
                                end
                                else begin
                                    state <= S_MWR_WAIT_TX_DONE;
                                end
                            end
                            else begin
                                mwr_words_sent <=
                                    mwr_words_sent +
                                    mwr_words_this_beat;
                                state <= S_MWR_DATA_QUEUE;
                            end
                        end
                    end

                    S_MWR_WAIT_TX_DONE: begin
                        if (watchdog_may_abort) begin
                            dbg_mwr_tx_timeout_count <=
                                dbg_mwr_tx_timeout_count + 1'b1;
                            dbg_last_drop_reason <= 8'h08;
                            dma_fault_latched <= 1'b1;
                            out_pending <= 1'b0;
                            mwr_tlp_open <= 1'b0;
                            mwr_tx_done_latched <= 1'b0;
                            force_status_pending <= 1'b0;
                            state <= S_WAIT_CONFIG;
                        end
                        else if (mwr_tx_done_event ||
                            mwr_tx_done_latched) begin
                            mwr_tx_done_latched <= 1'b0;
                            mwr_offset_bytes <=
                                mwr_offset_bytes + mwr_chunk_bytes;
                            state <= S_MWR_CHUNK_DONE;
                        end
                    end

                    S_MWR_CHUNK_DONE: begin
                        if (mwr_offset_bytes >= mwr_total_bytes)
                            state <= after_mwr_state;
                        else
                            state <= S_MWR_PREP;
                    end

                    S_STATUS_WRITTEN: begin
                        force_status_pending <= 1'b0;
                        if (status_link_change) begin
                            link_status_pending <= 1'b0;
                            status_link_change  <= 1'b0;
                        end
                        // 只有完整状态 MWr 已被发送后，邮箱比较器才能看到新 tag。
                        committed_status_tag <= status_tag;
                        dbg_status_commit_count <=
                            dbg_status_commit_count + 1'b1;
                        state <= S_IRQ;
                    end

                    S_IRQ: begin
                        irq_request <= 1'b1;
                        if (!PRODUCTION &&
                            dbg_offer_pending &&
                            !dbg_offer_frozen) begin
                            dbg_offer_frozen <= 1'b1;
                            dbg_offer_pending <= 1'b0;
                            dbg_offer_irq <= 1'b1;
                            dbg_offer_addr <= rx_buffer_addr;
                            dbg_offer_opaque <= rx_buffer_opaque;
                            dbg_offer_idx_len <= {
                                rx_buffer_index,
                                reply_frame_length + 16'd4
                            };
                            dbg_offer_prod <= rx_ret_prod_idx;
                            dbg_offer_tag <= status_tag;
                        end
                        state       <= S_IDLE;
                    end

                    default: state <= S_WAIT_CONFIG;
                endcase
            end

            // Completion 超时在另一时序块里只打一拍 cpl_error；teardown
            // 可能同拍抢到 !cpl_active。这里最后赋值，保证 tag 隔离置位。
            if (cpl_error)
                dma_fault_latched <= 1'b1;
        end
    end

endmodule



// ------------------------------------------------------------------------
// TLP-AXI-STREAM destination:
// Forward the data to output device (FT601, etc.). 
// ------------------------------------------------------------------------
module pcileech_tlps128_dst_fifo(
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    IfAXIS128.sink_lite     tlps_in,
    IfPCIeFifoTlp.mp_pcie   dfifo
);
    
    wire         tvalid;
    wire [127:0] tdata;
    wire [3:0]   tkeepdw;
    wire         tlast;
    wire         first;
       
    fifo_134_134_clk2 i_fifo_134_134_clk2 (
        .rst        ( rst               ),
        .wr_clk     ( clk_pcie          ),
        .rd_clk     ( clk_sys           ),
        .din        ( { tlps_in.tuser[0], tlps_in.tlast, tlps_in.tkeepdw, tlps_in.tdata } ),
        .wr_en      ( tlps_in.tvalid    ),
        .rd_en      ( dfifo.rx_rd_en    ),
        .dout       ( { first, tlast, tkeepdw, tdata } ),
        .full       (                   ),
        .empty      (                   ),
        .valid      ( tvalid            )
    );

    assign dfifo.rx_data[0]  = tdata[31:0];
    assign dfifo.rx_data[1]  = tdata[63:32];
    assign dfifo.rx_data[2]  = tdata[95:64];
    assign dfifo.rx_data[3]  = tdata[127:96];
    assign dfifo.rx_first[0] = first;
    assign dfifo.rx_first[1] = 0;
    assign dfifo.rx_first[2] = 0;
    assign dfifo.rx_first[3] = 0;
    assign dfifo.rx_last[0]  = tlast && (tkeepdw == 4'b0001);
    assign dfifo.rx_last[1]  = tlast && (tkeepdw == 4'b0011);
    assign dfifo.rx_last[2]  = tlast && (tkeepdw == 4'b0111);
    assign dfifo.rx_last[3]  = tlast && (tkeepdw == 4'b1111);
    assign dfifo.rx_valid[0] = tvalid && tkeepdw[0];
    assign dfifo.rx_valid[1] = tvalid && tkeepdw[1];
    assign dfifo.rx_valid[2] = tvalid && tkeepdw[2];
    assign dfifo.rx_valid[3] = tvalid && tkeepdw[3];

endmodule



// ------------------------------------------------------------------------
// TLP-AXI-STREAM FILTER:
// Filter away certain packet types such as CfgRd/CfgWr or non-Cpl/CplD
// ------------------------------------------------------------------------
module pcileech_tlps128_filter(
    input                   rst,
    input                   clk_pcie,
    input                   alltlp_filter,
    input                   cfgtlp_filter,
    input                   drop_tlp,
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source_lite   tlps_out
);

    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tvalid  = 0;
    bit [8:0]       tuser;
    bit             tlast;
    
    assign tlps_out.tdata   = tdata;
    assign tlps_out.tkeepdw = tkeepdw;
    assign tlps_out.tvalid  = tvalid;
    assign tlps_out.tuser   = tuser;
    assign tlps_out.tlast   = tlast;
    
    bit  filter = 0;
    wire first = tlps_in.tuser[0];
    wire is_tlphdr_cpl = first && (
                        (tlps_in.tdata[31:25] == 7'b0000101) ||      // Cpl:  Fmt[2:0]=000b (3 DW header, no data), Cpl=0101xb
                        (tlps_in.tdata[31:25] == 7'b0100101)         // CplD: Fmt[2:0]=010b (3 DW header, data),    CplD=0101xb
                      );
    wire is_tlphdr_cfg = first && (
                        (tlps_in.tdata[31:25] == 7'b0000010) ||      // CfgRd: Fmt[2:0]=000b (3 DW header, no data), CfgRd0/CfgRd1=0010xb
                        (tlps_in.tdata[31:25] == 7'b0100010)         // CfgWr: Fmt[2:0]=010b (3 DW header, data),    CfgWr0/CfgWr1=0010xb
                      );
    wire filter_next = (filter && !first) ||
                       (drop_tlp && first) ||
                       (cfgtlp_filter && first && is_tlphdr_cfg) ||
                       (alltlp_filter && first && !is_tlphdr_cpl && !is_tlphdr_cfg);
                      
    always @ ( posedge clk_pcie ) begin
        tdata   <= tlps_in.tdata;
        tkeepdw <= tlps_in.tkeepdw;
        tvalid  <= tlps_in.tvalid && !filter_next && !rst;
        tuser   <= tlps_in.tuser;
        tlast   <= tlps_in.tlast;
        filter  <= filter_next && !rst;
    end
    
endmodule



// ------------------------------------------------------------------------
// RX FROM FIFO - TLP-AXI-STREAM:
// Convert 32-bit incoming data to 128-bit TLP-AXI-STREAM to be sent onwards to mux/pcie core. 
// ------------------------------------------------------------------------
module pcileech_tlps128_src_fifo (
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    input [31:0]            dfifo_tx_data,
    input                   dfifo_tx_last,
    input                   dfifo_tx_valid,
    IfAXIS128.source        tlps_out
);

    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk_sys )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 1;
        end
        else begin
            tlast   <= dfifo_tx_valid && dfifo_tx_last;
            tkeepdw <= tvalid ? (dfifo_tx_valid ? 4'b0001 : 4'b0000) : (dfifo_tx_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= tvalid ? tlast : first;
            if ( dfifo_tx_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= dfifo_tx_data;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= dfifo_tx_data;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= dfifo_tx_data;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= dfifo_tx_data;   
            end
        end
		
    // --------------------------------------------------------------------
    // 2.1 - 溢出保护：宁可丢整包，也绝不向链路发半个坏包。
    // 数据/标记 FIFO 满时写入会被 IP 静默丢弃，包框架随之损坏（拍丢失、
    // 包粘连），下游会把畸形 TLP 发到链路上（主机 AER/Fatal）。写侧规则：
    //   - 任何丢拍都置 wr_poison，当前包的标记记为 poison=1；
    //   - 包尾拍被丢弃时置 term_pending，腾出空间后补写一条 tlast 终结
    //     记录，保证数据 FIFO 中每段记录都以 tlast 收尾；
    //   - term_pending 期间暂停一切正常写入（后续包被整体压制，poison
    //     保证恢复写入后其残段标记仍为丢弃态）。
    // 读侧按标记逐包调度：poison=0 正常转发，poison=1 整段丢弃。
    // 标记 FIFO 深度(1024)大于数据 FIFO(512)可容纳的最大包段数，永不会满。
    // PCILeech 上位机对丢失的注入 TLP 会超时重试，对主机无害。
    // --------------------------------------------------------------------
    wire        data_fifo_full;
    wire        mark_fifo_full;
    bit         term_pending = 1'b0;    // 需补写 tlast 终结记录
    bit         wr_poison    = 1'b0;    // 当前包已发生丢拍

    wire        beat_drop   = tvalid && (term_pending || data_fifo_full);
    wire        frag_drop   = term_pending && dfifo_tx_valid;
    wire        pkt_end_any = tvalid && tlast;
    wire        pkt_end     = pkt_end_any && !term_pending;
    wire        mark_din    = wr_poison || beat_drop;

    always @ ( posedge clk_sys ) begin
        if ( rst ) begin
            wr_poison    <= 1'b0;
            term_pending <= 1'b0;
        end
        else begin
            if ( pkt_end_any )
                wr_poison <= 1'b0;
            else if ( beat_drop || frag_drop )
                wr_poison <= 1'b1;

            if ( pkt_end && data_fifo_full )
                term_pending <= 1'b1;
            else if ( term_pending && !data_fifo_full )
                term_pending <= 1'b0;
        end
    end

    // 2.2 - 读侧按标记逐包调度。两个 FIFO 均为 FWFT 模式：dout 即队首、
    // valid=有数据、rd_en=弹出（消费），弹出只发生在拍被下游真正消费
    // （或丢弃）的时刻，停顿期间队首保持不动，不存在"读出即丢失"的窗口。
    localparam [1:0] RS_IDLE    = 2'd0;
    localparam [1:0] RS_STREAM  = 2'd1;
    localparam [1:0] RS_DISCARD = 2'd2;
    bit [1:0]   rstate = RS_IDLE;
    wire        mark_empty;
    wire        mark_dout;                  // 队首包的 poison 标记
    wire        data_valid;

    // 弹出 = 消费/丢弃当前队首拍；包尾拍弹出时同时弹出其 poison 标记。
    wire        pop_beat  = ((rstate == RS_STREAM) && data_valid &&
                             tlps_out.tready) ||
                            ((rstate == RS_DISCARD) && data_valid);
    wire        frag_done = pop_beat && tlps_out.tlast;

    always @ ( posedge clk_pcie ) begin
        if ( rst )
            rstate <= RS_IDLE;
        else
            case ( rstate )
                RS_IDLE:
                    if ( !mark_empty )
                        rstate <= mark_dout ? RS_DISCARD : RS_STREAM;
                RS_STREAM:
                    if ( frag_done )
                        rstate <= RS_IDLE;
                RS_DISCARD:
                    if ( frag_done )
                        rstate <= RS_IDLE;
                default:
                    rstate <= RS_IDLE;
            endcase
    end

    assign tlps_out.has_data = (rstate == RS_STREAM);
    assign tlps_out.tvalid   = (rstate == RS_STREAM) && data_valid;
    wire        data_rd_en   = pop_beat;
    wire        mark_rd_en   = frag_done;

    // 标记 FIFO：每个包尾写入 1 位 poison 标记（FWFT）。
    fifo_1_1_clk2 i_fifo_1_1_clk2(
        .rst            ( rst                        ),
        .wr_clk         ( clk_sys                    ),
        .rd_clk         ( clk_pcie                   ),
        .din            ( mark_din                   ),
        .wr_en          ( pkt_end && !mark_fifo_full ),
        .rd_en          ( mark_rd_en                 ),
        .dout           ( mark_dout                  ),
        .full           ( mark_fifo_full             ),
        .empty          ( mark_empty                 ),
        .valid          (                            )
    );
        
    // 2.3 - 数据 FIFO：存放完整包拍序列；term_pending 时补写 tlast 终结记录。
    fifo_134_134_clk2_rxfifo i_fifo_134_134_clk2_rxfifo(
        .rst            ( rst                       ),
        .wr_clk         ( clk_sys                   ),
        .rd_clk         ( clk_pcie                  ),
        .din            ( term_pending ? {1'b0, 1'b1, 4'b0000, 128'h0}
                                      : {first, tlast, tkeepdw, tdata} ),
        .wr_en          ( term_pending ? !data_fifo_full
                                      : (tvalid && !data_fifo_full) ),
        .rd_en          ( data_rd_en                ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           ( data_fifo_full            ),
        .empty          (                           ),
        .valid          ( data_valid                )
    );

endmodule



// ------------------------------------------------------------------------
// RX MUX - TLP-AXI-STREAM:
// Select the TLP-AXI-STREAM with the highest priority (lowest number) and
// let it transmit its full packet.
//
// 预约/开包两段：id!=0 且 pkt_open=0 只是预约，输出已切到该源，但若源
// 在首拍授权前撤回 has_data/tvalid，立即取消预约。pkt_open=1 后必须
// valid&&ready&&last 才释放。tready 只给已登记的 id，避免 FWFT 源在
// id 仍为 0 时被提前 pop。
// ------------------------------------------------------------------------
module pcileech_tlps128_sink_mux1 (
    input                       clk_pcie,
    input                       rst,
    IfAXIS128.source            tlps_out,
    IfAXIS128.sink              tlps_in1,
    IfAXIS128.sink              tlps_in2,
    IfAXIS128.sink              tlps_in3,
    IfAXIS128.sink              tlps_in4,
    input                       hold_in3
);
    bit [2:0] id = 0;
    bit       pkt_open = 0;
    bit       last_data_was_4 = 0;
    wire      in3_available = tlps_in3.has_data && !hold_in3;
    
    assign tlps_out.has_data    = tlps_in1.has_data ||
                                  tlps_in2.has_data ||
                                  in3_available ||
                                  tlps_in4.has_data;
    
    assign tlps_out.tdata       = (id==1) ? tlps_in1.tdata :
                                  (id==2) ? tlps_in2.tdata :
                                  (id==3) ? tlps_in3.tdata :
                                  (id==4) ? tlps_in4.tdata : 0;
    
    assign tlps_out.tkeepdw     = (id==1) ? tlps_in1.tkeepdw :
                                  (id==2) ? tlps_in2.tkeepdw :
                                  (id==3) ? tlps_in3.tkeepdw :
                                  (id==4) ? tlps_in4.tkeepdw : 0;
    
    assign tlps_out.tlast       = (id==1) ? tlps_in1.tlast :
                                  (id==2) ? tlps_in2.tlast :
                                  (id==3) ? tlps_in3.tlast :
                                  (id==4) ? tlps_in4.tlast : 0;
    
    wire [8:0] muxed_tuser      = (id==1) ? tlps_in1.tuser :
                                  (id==2) ? tlps_in2.tuser :
                                  (id==3) ? tlps_in3.tuser :
                                  (id==4) ? tlps_in4.tuser : 9'h0;
    // [8:6] = mux 来源 id，随拍送到 dst64，硬核包尾按来源回执。
    assign tlps_out.tuser       = {id, muxed_tuser[5:0]};
    
    assign tlps_out.tvalid      = (id==1) ? tlps_in1.tvalid :
                                  (id==2) ? tlps_in2.tvalid :
                                  (id==3) ? tlps_in3.tvalid :
                                  (id==4) ? tlps_in4.tvalid : 0;
    
    // 配置/BAR响应保持最高优先级；DMA软件(in3)和内部网卡DMA(in4)
    // 以整包为单位轮转，避免任一持续流量永久饿死另一方。
    wire [2:0] id_next_data     =
        (in3_available && tlps_in4.has_data) ?
            (last_data_was_4 ? 3 : 4) :
        in3_available ? 3 :
        tlps_in4.has_data ? 4 : 0;
    wire [2:0] id_next_newsel   = tlps_in1.has_data ? 1 :
                                  tlps_in2.has_data ? 2 :
                                  id_next_data;
    
    wire src_has =
        (id==1) ? tlps_in1.has_data :
        (id==2) ? tlps_in2.has_data :
        (id==3) ? in3_available :
        (id==4) ? tlps_in4.has_data : 1'b0;
    wire src_last_fire =
        tlps_out.tvalid && tlps_out.tlast && tlps_out.tready;
    wire src_beat_fire =
        tlps_out.tvalid && tlps_out.tready;
    // 预约后源撤回队列：尚未开包，不能再等永远不会来的 last。
    wire src_withdrawn =
        (id != 3'd0) && !pkt_open &&
        !tlps_out.tvalid && !src_has;
    wire id_release = (id == 3'd0) || src_last_fire || src_withdrawn;
    wire [2:0] id_next = id_release ? id_next_newsel : id;
    
    assign tlps_in1.tready      = tlps_out.tready && (id==1);
    assign tlps_in2.tready      = tlps_out.tready && (id==2);
    assign tlps_in3.tready      = tlps_out.tready && (id==3);
    assign tlps_in4.tready      = tlps_out.tready && (id==4);
    
    always @ ( posedge clk_pcie ) begin
        if (rst) begin
            id <= 0;
            pkt_open <= 1'b0;
            last_data_was_4 <= 0;
        end
        else begin
            id <= id_next;
            if (src_last_fire || src_withdrawn || (id_next == 3'd0))
                pkt_open <= 1'b0;
            else if (src_beat_fire)
                pkt_open <= 1'b1;
            if (id_release &&
                ((id_next_newsel == 3) || (id_next_newsel == 4)))
                last_data_was_4 <= (id_next_newsel == 4);
        end
    end
    
endmodule
