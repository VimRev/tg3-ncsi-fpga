//
// PCILeech FPGA.
//
// PCIe BAR PIO controller.
//
// The PCILeech BAR PIO controller allows for easy user-implementation on top
// of the PCILeech AXIS128 PCIe TLP streaming interface.
// The controller consists of a read engine and a write engine and pluggable
// user-implemented PCIe BAR implementations (found at bottom of the file).
//
// Considerations:
// - The core handles 1 DWORD read + 1 DWORD write per CLK max. If a lot of
//   data is written / read from the TLP streaming interface the core may
//   drop packet silently.
// - The core reads 1 DWORD of data (without byte enable) per CLK.
// - The core writes 1 DWORD of data (with byte enable) per CLK.
// - All user-implemented cores must have the same latency in CLKs for the
//   returned read data or else undefined behavior will take place.
// - 32-bit addresses are passed for read/writes. Larger BARs than 4GB are
//   not supported due to addressing constraints. Lower bits (LSBs) are the
//   BAR offset, Higher bits (MSBs) are the 32-bit base address of the BAR.
// - DO NOT edit read/write engines.
// - DO edit pcileech_tlps128_bar_controller (to swap bar implementations).
// - DO edit the bar implementations (at bottom of the file, if neccessary).
//
// Example implementations exists below, swap out any of the example cores
// against a core of your use case, or modify existing cores.
// Following test cores exist (see below in this file):
// - pcileech_bar_impl_zerowrite4k = zero-initialized read/write BAR.
//     It's possible to modify contents by use of .coe file.
// - pcileech_bar_impl_loopaddr = test core that loops back the 32-bit
//     address of the current read. Does not support writes.
// - pcileech_bar_impl_none = core without any reply.
// 
// (c) Ulf Frisk, 2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_tlps128_bar_controller #(
    parameter PRODUCTION = 0
)(
    input                   rst,
    input                   clk,
    input                   bar_en,
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    IfAXIS128.source        tlps_out,
    input                   lifecycle_reset_req,
    // BME / D0：会话失效信号。禁用/启用路径上 pci.sys 先清 BME，
    // BAR 侧地址作废不等待 DMA 静默（排空由引擎 teardown 保证）。
    input                   bus_master_enable,
    input                   power_state_d0,
    output                  broadcom_dma_reset_req,
    input                   broadcom_dma_quiescent,
    output                  int_enable,
    output                  msix_vaild,
    input                   msix_send_done,
    output [31:0]           msix_address,
    output [31:0]           msix_vector,
    output [31:0]           broadcom_cfg_68_value,
    output [31:0]           broadcom_cfg_6c_value,
    output [31:0]           broadcom_cfg_70_value,
    output [63:0]           broadcom_dma_tx_ring_addr,
    output [15:0]           broadcom_dma_tx_ring_size,
    output [63:0]           broadcom_dma_rx_std_ring_addr,
    output [15:0]           broadcom_dma_rx_std_ring_size,
    output [63:0]           broadcom_dma_rx_ret_ring_addr,
    output [15:0]           broadcom_dma_rx_ret_ring_size,
    output [63:0]           broadcom_dma_status_addr,
    output [15:0]           broadcom_dma_tx_prod_idx,
    output [15:0]           broadcom_dma_rx_std_prod_idx,
    output [15:0]           broadcom_dma_rx_ret_cons_idx,
    output                  broadcom_dma_hostcc_now,
    output                  broadcom_dma_link_event,
    input                   broadcom_dma_irq_request,
    input [7:0]             broadcom_dma_status_tag,
    input [703:0]           broadcom_dma_debug,
    input [447:0]           broadcom_dma_debug_ext,
    input                   broadcom_dma_tx_stat_event,
    input                   broadcom_dma_rx_stat_event,
    input [15:0]            broadcom_dma_tx_stat_bytes,
    input [15:0]            broadcom_dma_rx_stat_bytes,
    input [1:0]             broadcom_dma_tx_stat_class,
    input [1:0]             broadcom_dma_rx_stat_class,
    input [127:0]           broadcom_irq_debug,
    input                   broadcom_cfg_wr_valid,
    input [9:0]             broadcom_cfg_wr_dwaddr,
    input [3:0]             broadcom_cfg_wr_be,
    input [31:0]            broadcom_cfg_wr_data,
    input [31:0]            base_address_register,
    input [31:0]            base_address_register_1,
    input [31:0]            base_address_register_2,
    input [31:0]            base_address_register_3,
    input [31:0]            base_address_register_4,
    input [31:0]            base_address_register_5,
    // T06：DNA 派生 MAC。valid=0 时 0x410/0x414 与 NVRAM 0x7C/0x80 走快照/ROM。
    input                   nvram_mac_override_valid,
    input [31:0]            nvram_mac_word_7c,
    input [31:0]            nvram_mac_word_80
);

    wire bar0_interrupt_level;
    // 中断电平交给配置模块，由硬核当前模式转换为 MSI 或 Legacy INTA。
    assign int_enable    = bar0_interrupt_level;
    assign msix_vaild    = 1'b0;
    assign msix_address  = 32'h00000000;
    assign msix_vector   = 32'h00000000;
    
    // ------------------------------------------------------------------------
    // 1: TLP RECEIVE:
    // Receive incoming BAR requests from the TLP stream:
    // send them onwards to read and write FIFOs
    // ------------------------------------------------------------------------
    wire in_is_wr_ready;
    bit  in_is_wr_last;
    wire in_is_first    = tlps_in.tuser[0];
    wire in_is_bar      = bar_en && (tlps_in.tuser[8:2] != 0);
    wire in_is_rd       = (in_is_first && tlps_in.tlast && ((tlps_in.tdata[31:25] == 7'b0000000) || (tlps_in.tdata[31:25] == 7'b0010000) || (tlps_in.tdata[31:24] == 8'b00000010)));
    wire in_is_wr       = in_is_wr_last || (in_is_first && in_is_wr_ready && ((tlps_in.tdata[31:25] == 7'b0100000) || (tlps_in.tdata[31:25] == 7'b0110000) || (tlps_in.tdata[31:24] == 8'b01000010)));
    
    always @ ( posedge clk )
        if ( rst ) begin
            in_is_wr_last <= 0;
        end
        else if ( tlps_in.tvalid ) begin
            in_is_wr_last <= !tlps_in.tlast && in_is_wr;
        end
    
    wire [6:0]  wr_bar;
    wire [31:0] wr_addr;
    wire [3:0]  wr_be;
    wire [31:0] wr_data;
    wire        wr_valid;
    wire [87:0] rd_req_ctx;
    wire [6:0]  rd_req_bar;
    wire [31:0] rd_req_addr;
    wire [3:0]  rd_req_be;
    wire        rd_req_valid;
    wire [87:0] rd_rsp_ctx;
    wire [31:0] rd_rsp_data;
    wire        rd_rsp_valid;
        
    pcileech_tlps128_bar_rdengine i_pcileech_tlps128_bar_rdengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .pcie_id        ( pcie_id                       ),
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_rd ),
        .tlps_out       ( tlps_out                      ),
        // BAR reads:
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_bar     ( rd_req_bar                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_be      ( rd_req_be                     ),
        .rd_req_valid   ( rd_req_valid                  ),
        .rd_rsp_ctx     ( rd_rsp_ctx                    ),
        .rd_rsp_data    ( rd_rsp_data                   ),
        .rd_rsp_valid   ( rd_rsp_valid                  )
    );

    pcileech_tlps128_bar_wrengine i_pcileech_tlps128_bar_wrengine(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        // TLPs:
        .tlps_in        ( tlps_in                       ),
        .tlps_in_valid  ( tlps_in.tvalid && in_is_bar && in_is_wr ),
        .tlps_in_ready  ( in_is_wr_ready                ),
        // outgoing BAR writes:
        .wr_bar         ( wr_bar                        ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid                      )
    );
    
    wire [87:0] bar_rsp_ctx[7];
    wire [31:0] bar_rsp_data[7];
    wire        bar_rsp_valid[7];
    
    assign rd_rsp_ctx = bar_rsp_valid[0] ? bar_rsp_ctx[0] :
                        bar_rsp_valid[1] ? bar_rsp_ctx[1] :
                        bar_rsp_valid[2] ? bar_rsp_ctx[2] :
                        bar_rsp_valid[3] ? bar_rsp_ctx[3] :
                        bar_rsp_valid[4] ? bar_rsp_ctx[4] :
                        bar_rsp_valid[5] ? bar_rsp_ctx[5] :
                        bar_rsp_valid[6] ? bar_rsp_ctx[6] : 0;
    assign rd_rsp_data = bar_rsp_valid[0] ? bar_rsp_data[0] :
                        bar_rsp_valid[1] ? bar_rsp_data[1] :
                        bar_rsp_valid[2] ? bar_rsp_data[2] :
                        bar_rsp_valid[3] ? bar_rsp_data[3] :
                        bar_rsp_valid[4] ? bar_rsp_data[4] :
                        bar_rsp_valid[5] ? bar_rsp_data[5] :
                        bar_rsp_valid[6] ? bar_rsp_data[6] : 0;
    assign rd_rsp_valid = bar_rsp_valid[0] || bar_rsp_valid[1] || bar_rsp_valid[2] || bar_rsp_valid[3] || bar_rsp_valid[4] || bar_rsp_valid[5] || bar_rsp_valid[6];
    
    pcileech_bar_impl_broadcom_tg3 #(
        .PRODUCTION            ( PRODUCTION                   )
    ) i_bar0(
        .rst                   ( rst                           ),
        .clk                   ( clk                           ),
        .wr_addr               ( wr_addr                       ),
        .wr_be                 ( wr_be                         ),
        .wr_data               ( wr_data                       ),
        .wr_valid              ( wr_valid && wr_bar[0]         ),
        .rd_req_ctx            ( rd_req_ctx                    ),
        .rd_req_addr           ( rd_req_addr                   ),
        .rd_req_valid          ( rd_req_valid && rd_req_bar[0] ),
        .base_address_register ( base_address_register         ),
        .lifecycle_reset_req   ( lifecycle_reset_req           ),
        .bus_master_enable     ( bus_master_enable             ),
        .power_state_d0        ( power_state_d0                ),
        .dma_reset_req         ( broadcom_dma_reset_req        ),
        .dma_quiescent         ( broadcom_dma_quiescent        ),
        .cfg_68_value          ( broadcom_cfg_68_value          ),
        .cfg_6c_value          ( broadcom_cfg_6c_value          ),
        .cfg_70_value          ( broadcom_cfg_70_value          ),
        .dma_tx_ring_addr      ( broadcom_dma_tx_ring_addr      ),
        .dma_tx_ring_size      ( broadcom_dma_tx_ring_size      ),
        .dma_rx_std_ring_addr  ( broadcom_dma_rx_std_ring_addr  ),
        .dma_rx_std_ring_size  ( broadcom_dma_rx_std_ring_size  ),
        .dma_rx_ret_ring_addr  ( broadcom_dma_rx_ret_ring_addr  ),
        .dma_rx_ret_ring_size  ( broadcom_dma_rx_ret_ring_size  ),
        .dma_status_addr       ( broadcom_dma_status_addr       ),
        .dma_tx_prod_idx       ( broadcom_dma_tx_prod_idx       ),
        .dma_rx_std_prod_idx   ( broadcom_dma_rx_std_prod_idx   ),
        .dma_rx_ret_cons_idx   ( broadcom_dma_rx_ret_cons_idx   ),
        .dma_hostcc_now        ( broadcom_dma_hostcc_now        ),
        .dma_link_event        ( broadcom_dma_link_event        ),
        .dma_irq_request       ( broadcom_dma_irq_request       ),
        .dma_status_tag        ( broadcom_dma_status_tag        ),
        .dma_debug             ( broadcom_dma_debug             ),
        .dma_debug_ext         ( broadcom_dma_debug_ext         ),
        .dma_tx_stat_event     ( broadcom_dma_tx_stat_event     ),
        .dma_rx_stat_event     ( broadcom_dma_rx_stat_event     ),
        .dma_tx_stat_bytes     ( broadcom_dma_tx_stat_bytes     ),
        .dma_rx_stat_bytes     ( broadcom_dma_rx_stat_bytes     ),
        .dma_tx_stat_class     ( broadcom_dma_tx_stat_class     ),
        .dma_rx_stat_class     ( broadcom_dma_rx_stat_class     ),
        .irq_debug             ( broadcom_irq_debug             ),
        .nvram_mac_override_valid ( nvram_mac_override_valid    ),
        .nvram_mac_word_7c     ( nvram_mac_word_7c              ),
        .nvram_mac_word_80     ( nvram_mac_word_80              ),
        .cfg_wr_valid          ( broadcom_cfg_wr_valid          ),
        .cfg_wr_dwaddr         ( broadcom_cfg_wr_dwaddr         ),
        .cfg_wr_be             ( broadcom_cfg_wr_be             ),
        .cfg_wr_data           ( broadcom_cfg_wr_data           ),
        .interrupt_level      ( bar0_interrupt_level           ),
        .rd_rsp_ctx            ( bar_rsp_ctx[0]                ),
        .rd_rsp_data           ( bar_rsp_data[0]               ),
        .rd_rsp_valid          ( bar_rsp_valid[0]              )
    );
    
    pcileech_bar_impl_none i_bar1(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[1]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[1] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[1]                ),
        .rd_rsp_data    ( bar_rsp_data[1]               ),
        .rd_rsp_valid   ( bar_rsp_valid[1]              )
    );
    
    pcileech_bar_impl_none i_bar2(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[2]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[2] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[2]                ),
        .rd_rsp_data    ( bar_rsp_data[2]               ),
        .rd_rsp_valid   ( bar_rsp_valid[2]              )
    );
    
    pcileech_bar_impl_none i_bar3(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[3]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[3] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[3]                ),
        .rd_rsp_data    ( bar_rsp_data[3]               ),
        .rd_rsp_valid   ( bar_rsp_valid[3]              )
    );
    
    pcileech_bar_impl_none i_bar4(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[4]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[4] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[4]                ),
        .rd_rsp_data    ( bar_rsp_data[4]               ),
        .rd_rsp_valid   ( bar_rsp_valid[4]              )
    );
    
    pcileech_bar_impl_none i_bar5(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[5]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[5] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[5]                ),
        .rd_rsp_data    ( bar_rsp_data[5]               ),
        .rd_rsp_valid   ( bar_rsp_valid[5]              )
    );
    
    pcileech_bar_impl_none i_bar6_optrom(
        .rst            ( rst                           ),
        .clk            ( clk                           ),
        .wr_addr        ( wr_addr                       ),
        .wr_be          ( wr_be                         ),
        .wr_data        ( wr_data                       ),
        .wr_valid       ( wr_valid && wr_bar[6]         ),
        .rd_req_ctx     ( rd_req_ctx                    ),
        .rd_req_addr    ( rd_req_addr                   ),
        .rd_req_valid   ( rd_req_valid && rd_req_bar[6] ),
        .rd_rsp_ctx     ( bar_rsp_ctx[6]                ),
        .rd_rsp_data    ( bar_rsp_data[6]               ),
        .rd_rsp_valid   ( bar_rsp_valid[6]              )
    );


endmodule



// ------------------------------------------------------------------------
// BAR WRITE ENGINE:
// Receives BAR WRITE TLPs and output BAR WRITE requests.
// Holds a 2048-byte buffer.
// Input flow rate is 16bytes/CLK (max).
// Output flow rate is 4bytes/CLK.
// If write engine overflows incoming TLP is completely discarded silently.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_wrengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    output                  tlps_in_ready,
    // outgoing BAR writes:
    output bit [6:0]        wr_bar,
    output bit [31:0]       wr_addr,
    output bit [3:0]        wr_be,
    output bit [31:0]       wr_data,
    output bit              wr_valid
);

    wire            f_rd_en;
    wire [127:0]    f_tdata;
    wire [3:0]      f_tkeepdw;
    wire [8:0]      f_tuser;
    wire            f_tvalid;
    
    bit [127:0]     tdata;
    bit [3:0]       tkeepdw;
    bit             tlast;
    
    bit [3:0]       be_first;
    bit [3:0]       be_last;
    bit             first_dw;
    bit [31:0]      addr;

    fifo_141_141_clk1_bar_wr i_fifo_141_141_clk1_bar_wr(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( {tlps_in.tuser[8:0], tlps_in.tkeepdw, tlps_in.tdata} ),
        .full           (                               ),
        .prog_empty     ( tlps_in_ready                 ),
        .rd_en          ( f_rd_en                       ),
        .dout           ( {f_tuser, f_tkeepdw, f_tdata} ),    
        .empty          (                               ),
        .valid          ( f_tvalid                      )
    );
    
    // STATE MACHINE:
    `define S_ENGINE_IDLE        3'h0
    `define S_ENGINE_FIRST       3'h1
    `define S_ENGINE_4DW_REQDATA 3'h2
    `define S_ENGINE_TX0         3'h4
    `define S_ENGINE_TX1         3'h5
    `define S_ENGINE_TX2         3'h6
    `define S_ENGINE_TX3         3'h7
    (* KEEP = "TRUE" *) bit [3:0] state = `S_ENGINE_IDLE;
    
    assign f_rd_en = (state == `S_ENGINE_IDLE) ||
                     (state == `S_ENGINE_4DW_REQDATA) ||
                     (state == `S_ENGINE_TX3) ||
                     ((state == `S_ENGINE_TX2 && !tkeepdw[3])) ||
                     ((state == `S_ENGINE_TX1 && !tkeepdw[2])) ||
                     ((state == `S_ENGINE_TX0 && !f_tkeepdw[1]));

    always @ ( posedge clk ) begin
        wr_addr     <= addr;
        wr_valid    <= ((state == `S_ENGINE_TX0) && f_tvalid) || (state == `S_ENGINE_TX1) || (state == `S_ENGINE_TX2) || (state == `S_ENGINE_TX3);
        
    end

    always @ ( posedge clk )
        if ( rst ) begin
            state <= `S_ENGINE_IDLE;
        end
        else case ( state )
            `S_ENGINE_IDLE: begin
                state   <= `S_ENGINE_FIRST;
            end
            `S_ENGINE_FIRST: begin
                if ( f_tvalid && f_tuser[0] ) begin
                    wr_bar      <= f_tuser[8:2];
                    tdata       <= f_tdata;
                    tkeepdw     <= f_tkeepdw;
                    tlast       <= f_tuser[1];
                    first_dw    <= 1;
                    be_first    <= f_tdata[35:32];
                    be_last     <= f_tdata[39:36];
                    if ( f_tdata[31:29] == 8'b010 ) begin       // 3 DW header, with data
                        addr    <= { f_tdata[95:66], 2'b00 };
                        state   <= `S_ENGINE_TX3;
                    end
                    else if ( f_tdata[31:29] == 8'b011 ) begin  // 4 DW header, with data
                        addr    <= { f_tdata[127:98], 2'b00 };
                        state   <= `S_ENGINE_4DW_REQDATA;
                    end 
                end
                else begin
                    state   <= `S_ENGINE_IDLE;
                end
            end 
            `S_ENGINE_4DW_REQDATA: begin
                state   <= `S_ENGINE_TX0;
            end
            `S_ENGINE_TX0: begin
                tdata       <= f_tdata;
                tkeepdw     <= f_tkeepdw;
                tlast       <= f_tuser[1];
                addr        <= addr + 4;
                wr_data     <= { f_tdata[0+00+:8], f_tdata[0+08+:8], f_tdata[0+16+:8], f_tdata[0+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (f_tkeepdw[1] ? 4'hf : be_last);
                state       <= f_tvalid ? (f_tkeepdw[1] ? `S_ENGINE_TX1 : `S_ENGINE_FIRST) : `S_ENGINE_IDLE;
            end
            `S_ENGINE_TX1: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[32+00+:8], tdata[32+08+:8], tdata[32+16+:8], tdata[32+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[2] ? 4'hf : be_last);
                state       <= tkeepdw[2] ? `S_ENGINE_TX2 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX2: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[64+00+:8], tdata[64+08+:8], tdata[64+16+:8], tdata[64+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (tkeepdw[3] ? 4'hf : be_last);
                state       <= tkeepdw[3] ? `S_ENGINE_TX3 : `S_ENGINE_FIRST;
            end
            `S_ENGINE_TX3: begin
                addr        <= addr + 4;
                wr_data     <= { tdata[96+00+:8], tdata[96+08+:8], tdata[96+16+:8], tdata[96+24+:8] };
                first_dw    <= 0;
                wr_be       <= first_dw ? be_first : (!tlast ? 4'hf : be_last);
                state       <= !tlast ? `S_ENGINE_TX0 : `S_ENGINE_FIRST;
            end
        endcase

endmodule




// ------------------------------------------------------------------------
// BAR READ ENGINE:
// Receives BAR READ TLPs and output BAR READ requests.
// ------------------------------------------------------------------------
module pcileech_tlps128_bar_rdengine(
    input                   rst,    
    input                   clk,
    // TLPs:
    input [15:0]            pcie_id,
    IfAXIS128.sink_lite     tlps_in,
    input                   tlps_in_valid,
    IfAXIS128.source        tlps_out,
    // BAR reads:
    output [87:0]           rd_req_ctx,
    output [6:0]            rd_req_bar,
    output [31:0]           rd_req_addr,
    output                  rd_req_valid,
    output [3:0]            rd_req_be,        
    input  [87:0]           rd_rsp_ctx,
    input  [31:0]           rd_rsp_data,
    input                   rd_rsp_valid
);
    // ------------------------------------------------------------------------
    // 1: PROCESS AND QUEUE INCOMING READ TLPs:
    // ------------------------------------------------------------------------
    wire [10:0] rd1_in_dwlen    = (tlps_in.tdata[9:0] == 0) ? 11'd1024 : {1'b0, tlps_in.tdata[9:0]};
    wire [6:0]  rd1_in_bar      = tlps_in.tuser[8:2];
    wire [15:0] rd1_in_reqid    = tlps_in.tdata[63:48];
    wire [7:0]  rd1_in_tag      = tlps_in.tdata[47:40];
    wire [31:0] rd1_in_addr     = { ((tlps_in.tdata[31:29] == 3'b000) ? tlps_in.tdata[95:66] : tlps_in.tdata[127:98]), 2'b00 };
    wire [3:0]  rd1_in_be       = tlps_in.tdata[35:32];
    wire [73:0] rd1_in_data;
    assign rd1_in_data[73:63]   = rd1_in_dwlen;
    assign rd1_in_data[62:56]   = rd1_in_bar;   
    assign rd1_in_data[55:48]   = rd1_in_tag;
    assign rd1_in_data[47:32]   = rd1_in_reqid;
    assign rd1_in_data[31:0]    = rd1_in_addr;

    
    wire [3:0]  rd1_out_be;
    wire        rd1_out_be_valid;
    wire        rd1_out_rden;
    wire [73:0] rd1_out_data;
    wire        rd1_out_valid;
    
    fifo_74_74_clk1_bar_rd1 i_fifo_74_74_clk1_bar_rd1(
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_data                   ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_data                  ),    
        .empty          (                               ),
        .valid          ( rd1_out_valid                 )
    );
    fifo_4_4_clk1_bar_rd1 i_fifo_4_4_clk1_bar_rd1 (
        .srst           ( rst                           ),
        .clk            ( clk                           ),
        .wr_en          ( tlps_in_valid                 ),
        .din            ( rd1_in_be                     ),
        .full           (                               ),
        .rd_en          ( rd1_out_rden                  ),
        .dout           ( rd1_out_be                    ),
        .empty          (                               ),
        .valid          ( rd1_out_be_valid              )

    );
    
    // ------------------------------------------------------------------------
    // 2: PROCESS AND SPLIT READ TLPs INTO RESPONSE TLP READ REQUESTS AND QUEUE:
    //    (READ REQUESTS LARGER THAN 128-BYTES WILL BE SPLIT INTO MULTIPLE).
    // ------------------------------------------------------------------------
    
    wire [10:0] rd1_out_dwlen       = rd1_out_data[73:63];
    wire [4:0]  rd1_out_dwlen5      = rd1_out_data[67:63];
    wire [4:0]  rd1_out_addr5       = rd1_out_data[6:2];
    
    // 1st "instant" packet:
    wire [4:0]  rd2_pkt1_dwlen_pre  = ((rd1_out_addr5 + rd1_out_dwlen5 > 6'h20) || ((rd1_out_addr5 != 0) && (rd1_out_dwlen5 == 0))) ? (6'h20 - rd1_out_addr5) : rd1_out_dwlen5;
    wire [5:0]  rd2_pkt1_dwlen      = (rd2_pkt1_dwlen_pre == 0) ? 6'h20 : rd2_pkt1_dwlen_pre;
    wire [10:0] rd2_pkt1_dwlen_next = rd1_out_dwlen - rd2_pkt1_dwlen;
    wire        rd2_pkt1_large      = (rd1_out_dwlen > 32) || (rd1_out_dwlen != rd2_pkt1_dwlen);
    wire        rd2_pkt1_tiny       = (rd1_out_dwlen == 1);
    wire [11:0] rd2_pkt1_bc         = rd1_out_dwlen << 2;
    wire [85:0] rd2_pkt1;
    assign      rd2_pkt1[85:74]     = rd2_pkt1_bc;
    assign      rd2_pkt1[73:63]     = rd2_pkt1_dwlen;
    assign      rd2_pkt1[62:0]      = rd1_out_data[62:0];
    
    // Nth packet (if split should take place):
    bit  [10:0] rd2_total_dwlen;
    wire [10:0] rd2_total_dwlen_next = rd2_total_dwlen - 11'h20;
    
    bit  [85:0] rd2_pkt2;
    wire [10:0] rd2_pkt2_dwlen = rd2_pkt2[73:63];
    wire        rd2_pkt2_large = (rd2_total_dwlen > 11'h20);
    
    wire        rd2_out_rden;
    
    // STATE MACHINE:
    `define S2_ENGINE_REQDATA     1'h0
    `define S2_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state2 = `S2_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            state2 <= `S2_ENGINE_REQDATA;
        end
        else case ( state2 )
            `S2_ENGINE_REQDATA: begin
                if ( rd1_out_valid && rd2_pkt1_large ) begin
                    rd2_total_dwlen <= rd2_pkt1_dwlen_next;                             // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_pkt1_dwlen_next << 2;                        // byte-count
                    rd2_pkt2[73:63] <= (rd2_pkt1_dwlen_next > 11'h20) ? 11'h20 : rd2_pkt1_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd1_out_data[62:12];                             // various data
                    rd2_pkt2[11:0]  <= rd1_out_data[11:0] + (rd2_pkt1_dwlen << 2);      // base address (within 4k page)
                    state2 <= `S2_ENGINE_PROCESSING;
                end
            end
            `S2_ENGINE_PROCESSING: begin
                if ( rd2_out_rden ) begin
                    rd2_total_dwlen <= rd2_total_dwlen_next;                                // dwlen (total remaining)
                    rd2_pkt2[85:74] <= rd2_total_dwlen_next << 2;                           // byte-count
                    rd2_pkt2[73:63] <= (rd2_total_dwlen_next > 11'h20) ? 11'h20 : rd2_total_dwlen_next;   // dwlen next
                    rd2_pkt2[62:12] <= rd2_pkt2[62:12];                                     // various data
                    rd2_pkt2[11:0]  <= rd2_pkt2[11:0] + (rd2_pkt2_dwlen << 2);              // base address (within 4k page)
                    if ( !rd2_pkt2_large ) begin
                        state2 <= `S2_ENGINE_REQDATA;
                    end
                end
            end
        endcase
    
    assign rd1_out_rden = rd2_out_rden && (((state2 == `S2_ENGINE_REQDATA) && (!rd1_out_valid || rd2_pkt1_tiny)) || ((state2 == `S2_ENGINE_PROCESSING) && !rd2_pkt2_large));

    wire [85:0] rd2_in_data  = (state2 == `S2_ENGINE_REQDATA) ? rd2_pkt1 : rd2_pkt2;
    wire        rd2_in_valid = rd1_out_valid || ((state2 == `S2_ENGINE_PROCESSING) && rd2_out_rden);
    wire [3:0]  rd2_in_be       = rd1_out_be;
    wire        rd2_in_be_valid = rd1_out_valid;

    bit  [85:0] rd2_out_data;
    bit         rd2_out_valid;
    bit  [3:0]  rd2_out_be;
    bit         rd2_out_be_valid;
    always @ ( posedge clk ) begin
        rd2_out_data    <= rd2_in_valid ? rd2_in_data : rd2_out_data;
        rd2_out_valid   <= rd2_in_valid && !rst;
        rd2_out_be       <= rd2_in_be_valid ? rd2_in_be : rd2_out_data;
        rd2_out_be_valid <= rd2_in_be_valid && !rst;  
    end

    // ------------------------------------------------------------------------
    // 3: PROCESS EACH READ REQUEST PACKAGE PER INDIVIDUAL 32-bit READ DWORDS:
    // ------------------------------------------------------------------------

    wire [4:0]  rd2_out_dwlen   = rd2_out_data[67:63];
    wire        rd2_out_last    = (rd2_out_dwlen == 1);
    wire [9:0]  rd2_out_dwaddr  = rd2_out_data[11:2];
    
    wire        rd3_enable;
    
    bit [3:0]   rd3_process_be;
    bit         rd3_process_valid;
    bit         rd3_process_first;
    bit         rd3_process_last;
    bit [4:0]   rd3_process_dwlen;
    bit [9:0]   rd3_process_dwaddr;
    bit [85:0]  rd3_process_data;
    wire        rd3_process_next_last = (rd3_process_dwlen == 2);
    wire        rd3_process_nextnext_last = (rd3_process_dwlen <= 3);
    assign rd_req_be    = rd3_process_be;
    assign rd_req_ctx   = { rd3_process_first, rd3_process_last, rd3_process_data };
    assign rd_req_bar   = rd3_process_data[62:56];
    assign rd_req_addr  = { rd3_process_data[31:12], rd3_process_dwaddr, 2'b00 };
    assign rd_req_valid = rd3_process_valid;
    
    // STATE MACHINE:
    `define S3_ENGINE_REQDATA     1'h0
    `define S3_ENGINE_PROCESSING  1'h1
    (* KEEP = "TRUE" *) bit [0:0] state3 = `S3_ENGINE_REQDATA;
    
    always @ ( posedge clk )
        if ( rst ) begin
            rd3_process_valid   <= 1'b0;
            state3              <= `S3_ENGINE_REQDATA;
        end
        else case ( state3 )
            `S3_ENGINE_REQDATA: begin
                if ( rd2_out_valid ) begin
                    rd3_process_valid       <= 1'b1;
                    rd3_process_first       <= 1'b1;                    // FIRST
                    rd3_process_last        <= rd2_out_last;            // LAST (low 5 bits of dwlen == 1, [max pktlen = 0x20))
                    rd3_process_dwlen       <= rd2_out_dwlen;           // PKT LENGTH IN DW
                    rd3_process_dwaddr      <= rd2_out_dwaddr;          // DWADDR OF THIS DWORD
                    rd3_process_data[85:0]  <= rd2_out_data[85:0];      // FORWARD / SAVE DATA
                    if ( rd2_out_be_valid ) begin
                        rd3_process_be <= rd2_out_be;
                    end else begin
                        rd3_process_be <= 4'hf;
                    end
                    if ( !rd2_out_last ) begin
                        state3 <= `S3_ENGINE_PROCESSING;
                    end
                end
                else begin
                    rd3_process_valid       <= 1'b0;
                end
            end
            `S3_ENGINE_PROCESSING: begin
                rd3_process_first           <= 1'b0;                    // FIRST
                rd3_process_last            <= rd3_process_next_last;   // LAST
                rd3_process_dwlen           <= rd3_process_dwlen - 1;   // LEN DEC
                rd3_process_dwaddr          <= rd3_process_dwaddr + 1;  // ADDR INC
                if ( rd3_process_next_last ) begin
                    state3 <= `S3_ENGINE_REQDATA;
                end
            end
        endcase

    assign rd2_out_rden = rd3_enable && (
        ((state3 == `S3_ENGINE_REQDATA) && (!rd2_out_valid || rd2_out_last)) ||
        ((state3 == `S3_ENGINE_PROCESSING) && rd3_process_nextnext_last));
    
    // ------------------------------------------------------------------------
    // 4: PROCESS RESPONSES:
    // ------------------------------------------------------------------------
    
    wire        rd_rsp_first    = rd_rsp_ctx[87];
    wire        rd_rsp_last     = rd_rsp_ctx[86];
    
    wire [9:0]  rd_rsp_dwlen    = rd_rsp_ctx[72:63];
    wire [11:0] rd_rsp_bc       = rd_rsp_ctx[85:74];
    wire [15:0] rd_rsp_reqid    = rd_rsp_ctx[47:32];
    wire [7:0]  rd_rsp_tag      = rd_rsp_ctx[55:48];
    wire [6:0]  rd_rsp_lowaddr  = rd_rsp_ctx[6:0];
    wire [31:0] rd_rsp_addr     = rd_rsp_ctx[31:0];
    wire [31:0] rd_rsp_data_bs  = { rd_rsp_data[7:0], rd_rsp_data[15:8], rd_rsp_data[23:16], rd_rsp_data[31:24] };
    
    // 1: 32-bit -> 128-bit state machine:
    bit [127:0] tdata;
    bit [3:0]   tkeepdw = 0;
    bit         tlast;
    bit         first   = 1;
    wire        tvalid  = tlast || tkeepdw[3];
    
    always @ ( posedge clk )
        if ( rst ) begin
            tkeepdw <= 0;
            tlast   <= 0;
            first   <= 0;
        end
        else if ( rd_rsp_valid && rd_rsp_first ) begin
            tkeepdw         <= 4'b1111;
            tlast           <= rd_rsp_last;
            first           <= 1'b1;
            tdata[31:0]     <= { 22'b0100101000000000000000, rd_rsp_dwlen };            // format, type, length
            tdata[63:32]    <= { pcie_id[7:0], pcie_id[15:8], 4'b0, rd_rsp_bc };        // pcie_id, byte_count
            tdata[95:64]    <= { rd_rsp_reqid, rd_rsp_tag, 1'b0, rd_rsp_lowaddr };      // req_id, tag, lower_addr
            tdata[127:96]   <= rd_rsp_data_bs;
        end
        else begin
            tlast   <= rd_rsp_valid && rd_rsp_last;
            tkeepdw <= tvalid ? (rd_rsp_valid ? 4'b0001 : 4'b0000) : (rd_rsp_valid ? ((tkeepdw << 1) | 1'b1) : tkeepdw);
            first   <= 0;
            if ( rd_rsp_valid ) begin
                if ( tvalid || !tkeepdw[0] )
                    tdata[31:0]   <= rd_rsp_data_bs;
                if ( !tkeepdw[1] )
                    tdata[63:32]  <= rd_rsp_data_bs;
                if ( !tkeepdw[2] )
                    tdata[95:64]  <= rd_rsp_data_bs;
                if ( !tkeepdw[3] )
                    tdata[127:96] <= rd_rsp_data_bs;   
            end
        end
    
    // 2.1 - submit to output fifo - will feed into mux/pcie core.
    fifo_134_134_clk1_bar_rdrsp i_fifo_134_134_clk1_bar_rdrsp(
        .srst           ( rst                       ),
        .clk            ( clk                       ),
        .din            ( { first, tlast, tkeepdw, tdata } ),
        .wr_en          ( tvalid                    ),
        .rd_en          ( tlps_out.tready           ),
        .dout           ( { tlps_out.tuser[0], tlps_out.tlast, tlps_out.tkeepdw, tlps_out.tdata } ),
        .full           (                           ),
        .empty          (                           ),
        .prog_empty     ( rd3_enable                ),
        .valid          ( tlps_out.tvalid           )
    );
    
    assign tlps_out.tuser[1] = tlps_out.tlast;
    assign tlps_out.tuser[8:2] = 0;
    
    // 2.2 - packet count:
    bit [10:0]  pkt_count       = 0;
    wire        pkt_count_dec   = tlps_out.tvalid && tlps_out.tlast;
    wire        pkt_count_inc   = tvalid && tlast;
    wire [10:0] pkt_count_next  = pkt_count + pkt_count_inc - pkt_count_dec;
    assign tlps_out.has_data    = (pkt_count_next > 0);
    
    always @ ( posedge clk ) begin
        pkt_count <= rst ? 0 : pkt_count_next;
    end

endmodule


// ------------------------------------------------------------------------
// Example BAR implementation that does nothing but drop any read/writes
// silently without generating a response.
// This is only recommended for placeholder designs.
// Latency = N/A.
// ------------------------------------------------------------------------
module pcileech_bar_impl_none(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    initial rd_rsp_ctx = 0;
    initial rd_rsp_data = 0;
    initial rd_rsp_valid = 0;

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of "address loopback" which can be useful
// for testing. Any read to a specific BAR address will result in the
// address as response.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_loopaddr(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input [87:0]        rd_req_ctx,
    input [31:0]        rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]      rd_req_ctx_1;
    bit [31:0]      rd_req_addr_1;
    bit             rd_req_valid_1;
    
    always @ ( posedge clk ) begin
        rd_req_ctx_1    <= rd_req_ctx;
        rd_req_addr_1   <= rd_req_addr;
        rd_req_valid_1  <= rd_req_valid;
        rd_rsp_ctx      <= rd_req_ctx_1;
        rd_rsp_data     <= rd_req_addr_1;
        rd_rsp_valid    <= rd_req_valid_1;
    end    

endmodule



// ------------------------------------------------------------------------
// Example BAR implementation of a 4kB writable initial-zero BAR.
// Latency = 2CLKs.
// ------------------------------------------------------------------------
module pcileech_bar_impl_zerowrite4k(
    input               rst,
    input               clk,
    // incoming BAR writes:
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // incoming BAR reads:
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    // outgoing BAR read replies:
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    bit [87:0]  drd_req_ctx;
    bit         drd_req_valid;
    wire [31:0] doutb;
    
    always @ ( posedge clk ) begin
        drd_req_ctx     <= rd_req_ctx;
        drd_req_valid   <= rd_req_valid;
        rd_rsp_ctx      <= drd_req_ctx;
        rd_rsp_valid    <= drd_req_valid;
        rd_rsp_data     <= doutb; 
    end
    
    bram_bar_zero4k i_bram_bar_zero4k(
        // Port A - write:
        .addra  ( wr_addr[11:2]     ),
        .clka   ( clk               ),
        .dina   ( wr_data           ),
        .ena    ( wr_valid          ),
        .wea    ( wr_be             ),
        // Port A - read (2 CLK latency):
        .addrb  ( rd_req_addr[11:2] ),
        .clkb   ( clk               ),
        .doutb  ( doutb             ),
        .enb    ( rd_req_valid      )
    );

endmodule



// ------------------------------------------------------------------------
// Broadcom Tigon3 BAR0 行为模型。
// 静态寄存器继续使用实卡快照；带完成位、轮询或副作用的寄存器单独建模。
// ------------------------------------------------------------------------
module pcileech_bar_impl_broadcom_tg3 #(
    parameter BAR_INIT_FILE = "broadcom_bar0_offline.mem",
    parameter NVRAM_INIT_FILE = "broadcom_nvram.mem",
    parameter integer LINK_EVENT_DELAY_BIT = 27,
    // 0=调试窗口；1=0x3E00–0x3FFC 走真卡快照。只留构建期开关。
    parameter PRODUCTION = 0
)(
    input               rst,
    input               clk,
    // BAR 写请求。
    input [31:0]        wr_addr,
    input [3:0]         wr_be,
    input [31:0]        wr_data,
    input               wr_valid,
    // BAR 读请求。
    input  [87:0]       rd_req_ctx,
    input  [31:0]       rd_req_addr,
    input               rd_req_valid,
    input  [31:0]       base_address_register,
    input               lifecycle_reset_req,
    // BME / D0：会话失效信号（禁用/启用路径）。
    input               bus_master_enable,
    input               power_state_d0,
    output              dma_reset_req,
    input               dma_quiescent,
    // 配置空间 0x68 与 BAR0 的芯片控制寄存器是同一份状态。
    output [31:0]       cfg_68_value,
    // 配置空间 0x6C 与 BAR0 的 DMA 读写控制寄存器共享状态。
    output [31:0]       cfg_6c_value,
    // 配置空间 0x70 与 BAR0 的 PCI 状态镜像共享状态。
    output [31:0]       cfg_70_value,
    // 驱动写入的主机 DMA 环、状态块和邮箱索引。
    output [63:0]       dma_tx_ring_addr,
    output [15:0]       dma_tx_ring_size,
    output [63:0]       dma_rx_std_ring_addr,
    output [15:0]       dma_rx_std_ring_size,
    output [63:0]       dma_rx_ret_ring_addr,
    output [15:0]       dma_rx_ret_ring_size,
    output [63:0]       dma_status_addr,
    output [15:0]       dma_tx_prod_idx,
    output [15:0]       dma_rx_std_prod_idx,
    output [15:0]       dma_rx_ret_cons_idx,
    output              dma_hostcc_now,
    output              dma_link_event,
    input               dma_irq_request,
    input [7:0]         dma_status_tag,
    input [703:0]       dma_debug,
    input [447:0]       dma_debug_ext,
    input               dma_tx_stat_event,
    input               dma_rx_stat_event,
    input [15:0]        dma_tx_stat_bytes,
    input [15:0]        dma_rx_stat_bytes,
    input [1:0]         dma_tx_stat_class,
    input [1:0]         dma_rx_stat_class,
    input [127:0]       irq_debug,
    // T06：DNA 派生 MAC 覆盖 NVRAM 0x7C/0x80；valid=0 时走 ROM。
    input               nvram_mac_override_valid,
    input [31:0]        nvram_mac_word_7c,
    input [31:0]        nvram_mac_word_80,
    // 配置空间写入同时更新 BAR0 镜像寄存器。
    input               cfg_wr_valid,
    input [9:0]         cfg_wr_dwaddr,
    input [3:0]         cfg_wr_be,
    input [31:0]        cfg_wr_data,
    // 设备中断源电平；配置模块负责 MSI/INTx 协议握手。
    output              interrupt_level,
    // BAR 读返回。
    output bit [87:0]   rd_rsp_ctx,
    output bit [31:0]   rd_rsp_data,
    output bit          rd_rsp_valid
);

    // BAR0 固定为 64 KiB 对齐窗口，硬核已经通过 BAR Hit 完成范围判断。
    // 直接使用请求地址低 16 位，避免依赖配置管理接口异步采集 BAR 基址。
    wire [31:0] wr_offset = {16'h0000, wr_addr[15:0]};
    wire [31:0] rd_offset = {16'h0000, rd_req_addr[15:0]};
    wire        wr_in_range = 1'b1;
    wire        rd_in_range = 1'b1;

    (* ram_style = "block" *) reg [31:0] bar_mem [0:16383];
    wire [13:0] bar_mem_wr_addr = wr_offset[15:2];
    wire [3:0]  bar_mem_wr_be = {4{!rst && wr_valid && wr_in_range}} & wr_be;
    reg [31:0]  bar_mem_wr_data;

    bit [87:0] rd_req_ctx_q;
    bit        rd_req_valid_q;
    bit        rd_in_range_q;
    bit [15:0] rd_offset_q;
    bit [31:0] rd_base_q;
    bit [31:0] rd_data_q;
    bit        rd_special_valid_q;
    bit [31:0] rd_special_data_q;

    // PCI、MAC、GRC 与片上 CPU 的动态状态。
    bit [31:0] misc_host_ctrl;
    bit [31:0] pci_state;
    bit [31:0] clock_ctrl;
    bit [31:0] mac_mode;
    bit        mac_link_event_pending;
    bit        mac_link_event_armed;
    bit        mac_link_delay_active;
    // 默认 bit27 在 62.5 MHz 下约为 2.147 秒；强制使用空闲 DSP，
    // 并让测试可缩短计数器宽度，避免增加 LUT 计数链。
    (* use_dsp = "yes" *)
    bit [LINK_EVENT_DELAY_BIT:0] mac_link_delay_count;
    bit [31:0] rx_cpu_state;
    bit [1:0]  rx_cpu_reset_writes;
    bit [31:0] grc_misc_cfg;
    bit [31:0] grc_local_ctrl;
    bit [31:0] grc_6838;
    bit        tagged_status_enabled;
    bit [31:0] cfg_6c_state;
    bit [31:0] interrupt_mailbox;
    bit        irq_force_pending;
    bit        irq_level;
    // 中断聚合 holdoff：一次中断投递后约 33us(2047 拍@62.5MHz)内的
    // 新事件只记 pending，到期后再投递，避免高负载时每包一次 MSI
    // 把主机 ISR/DPC 打满造成整机卡顿。
    bit [10:0] irq_holdoff_count;
    // BISECT 开关：IRQ_HOLDOFF_EN=0 时聚合完全透明（定位为 DHCP 回归用）。
    localparam  IRQ_HOLDOFF_EN = 1'b0;
    wire       irq_holdoff_active = IRQ_HOLDOFF_EN && (irq_holdoff_count != 0);
    bit        soft_reset_pending;
    bit        lifecycle_reset_pending;
    bit        dma_reset_req_state;
    // 静默兜底计时：软复位等待 quiescent 永远不来时强制完成，
    // 保证 0x6804 bit0 必然自清（防驱动 halt/init 轮询超时卡死）。
    bit [15:0] reset_timeout;
    bit [3:0]  soft_reset_guard;
    wire       reset_quiescent_ready =
                   dma_quiescent && !irq_level && !irq_force_pending;
    wire       reset_timeout_fired = (reset_timeout == 16'hFFF0);
    // 会话结束沿检测：只在 BME/D0 的 1→0 沿作废一次。电平式（BME=0 期间
    // 持续清空）会把驱动在 BME 置位前编程的地址立刻擦掉，属于误伤。
    bit        bme_prev;
    bit        d0_prev;
    wire       session_end = (bme_prev && !bus_master_enable) ||
                             (d0_prev && !power_state_d0);
    wire       soft_reset_write_request =
                   wr_valid && wr_in_range && (wr_be == 4'hF) &&
                   (wr_offset[15:0] == 16'h6804) &&
                   (wr_data == 32'h24000001);
    wire       reset_generation_active =
                   soft_reset_pending || lifecycle_reset_pending ||
                   lifecycle_reset_req || soft_reset_write_request;

    // 只读中断诊断：钉死"驱动是否使能过中断 / 中断是否真正投递过"。
    bit        dbg_mbox_ever_unmasked;   // 驱动写过 0x204 且 bit0==0（使能/重开）
    bit        dbg_irq_ever_delivered;   // irq_level 曾经 0->1（向硬核投递过中断）
    bit        irq_level_prev;
    bit [15:0] dbg_mbox_write_count;     // 0x204 写入次数（风暴检测）
    // 二期只读诊断：刻画 init 循环。
    bit [15:0] dbg_softreset_count;      // 0x6804=0x24000001 软复位次数
    bit [15:0] dbg_hostcc_now_count;     // hostcc_now 触发次数
    bit [31:0] dbg_last_3c00;            // 最后写入 0x3C00(HOSTCC_MODE) 的值
    bit [31:0] dbg_last_6800;            // 最后写入 0x6800(GRC_MODE) 的值

    // 中断控制寄存器写入必须优先完成确认或屏蔽，不能在同一拍重新拉高中断。
    wire irq_mailbox_write = wr_valid && wr_in_range && (wr_be == 4'hF) &&
                             (wr_offset[15:0] == 16'h0204);
    wire irq_misc_bar_write = wr_valid && wr_in_range && (wr_be == 4'hF) &&
                              (wr_offset[15:0] == 16'h0068);
    wire irq_misc_cfg_write = cfg_wr_valid && (cfg_wr_dwaddr == 10'h01A);
    wire irq_control_write = irq_mailbox_write ||
                             irq_misc_bar_write ||
                             irq_misc_cfg_write;
    wire irq_pending_delivery = irq_force_pending &&
                                !reset_generation_active &&
                                !irq_holdoff_active &&
                                !irq_level &&
                                !interrupt_mailbox[0] &&
                                !misc_host_ctrl[1] &&
                                !irq_control_write;

    // 配置空间 0x7C/0x84 是片上 SRAM 的间接访问窗口。
    // 这里只镜像数据面需要的 RCB，避免为 DMA 再复制一份完整 SRAM。
    bit [31:0] dma_cfg_sram_addr;
    bit [63:0] dma_tx_ring_addr_state;
    bit [15:0] dma_tx_ring_size_state;
    bit [63:0] dma_rx_std_ring_addr_state;
    bit [15:0] dma_rx_std_ring_size_state;
    bit [63:0] dma_rx_ret_ring_addr_state;
    bit [15:0] dma_rx_ret_ring_size_state;
    bit [63:0] dma_status_addr_state;
    bit [15:0] dma_tx_prod_idx_state;
    bit [15:0] dma_rx_std_prod_idx_state;
    bit [15:0] dma_rx_ret_cons_idx_state;
    bit        dma_hostcc_now_state;
    bit        dma_link_event_state;
    bit [31:0] dma_done_count;

    // DMA 数据面驱动的 MAC 统计。BCM5750/5751 的这些寄存器为读清零；
    // TX 位于 0x800/0x86C--0x874，RX 位于 0x880/0x88C--0x894。
    // 同一拍发生读取和新事件时，返回读取前累计值，并把新事件保留到下一次读取。
    bit [31:0] rx_stat_octets;
    bit [31:0] rx_stat_unicast;
    bit [31:0] rx_stat_multicast;
    bit [31:0] rx_stat_broadcast;
    bit [31:0] tx_stat_octets;
    bit [31:0] tx_stat_unicast;
    bit [31:0] tx_stat_multicast;
    bit [31:0] tx_stat_broadcast;
    // 板上诊断用影子计数不参与 TG3 read-clear，便于和驱动/任务管理器读数交叉核对。
    bit [31:0] tx_stat_shadow_octets;
    bit [31:0] tx_stat_shadow_packets;
    bit [31:0] tx_stat_shadow_unicast;
    bit [31:0] tx_stat_shadow_multicast;
    bit [31:0] tx_stat_shadow_broadcast;
    bit [31:0] rx_stat_shadow_octets;
    bit [31:0] rx_stat_shadow_packets;
    bit [31:0] rx_stat_shadow_unicast;
    bit [31:0] rx_stat_shadow_multicast;
    bit [31:0] rx_stat_shadow_broadcast;
    // TG3 MAC 统计窗口：TX octets 位于 0x0800，RX octets 位于 0x0880。
    localparam [15:0] MAC_TX_OCTETS_ADDR = 16'h0800;
    localparam [15:0] MAC_TX_UCAST_ADDR  = 16'h086C;
    localparam [15:0] MAC_TX_MCAST_ADDR  = 16'h0870;
    localparam [15:0] MAC_TX_BCAST_ADDR  = 16'h0874;
    localparam [15:0] MAC_RX_OCTETS_ADDR = 16'h0880;
    localparam [15:0] MAC_RX_UCAST_ADDR  = 16'h088C;
    localparam [15:0] MAC_RX_MCAST_ADDR  = 16'h0890;
    localparam [15:0] MAC_RX_BCAST_ADDR  = 16'h0894;
    // b57nd60a 的统计刷新函数会轮询完整 TG3 MAC 统计集合。
    // 当前数据面只产生 OK 包/字节；其它错误、碰撞、长度桶和扣减项必须显式返回 0，
    // 避免落回静态 BAR 镜像或驱动写入残值，影响 NDIS/任务管理器的聚合结果。
    function automatic bit tg3_stat_zero_addr(
        input [15:0] addr
    );
        begin
            // 仅对“驱动统计刷新会轮询、但当前数据面未实现”的桶显式回 0。
            // 必须是精确白名单，不能用 0x0800-0x08BC 区间：真卡在
            // 0x0804/0814/0828/0834-0868(FF) 和 0x0884(0x23) 上是功能/保留寄存器，
            // 区间清 0 会把这些真卡值喂成 0，破坏驱动 init/RX 建环 → DHCP 拿不到地址。
            // 8 个真实计数器(0x800/086C/0870/0874/0880/088C/0890/0894)不在此表，
            // 各自走 rd_clear_* 读清零路径。
            case (addr)
                16'h0808, 16'h080C, 16'h0810, 16'h0818, 16'h081C,
                16'h0820, 16'h0824, 16'h082C, 16'h0830, 16'h0878,
                16'h0888, 16'h0898, 16'h089C, 16'h08A0, 16'h08A4,
                16'h08A8, 16'h08AC, 16'h08B0, 16'h08B4, 16'h08B8,
                16'h224C, 16'h2250, 16'h2254, 16'h3C48:
                    tg3_stat_zero_addr = 1'b1;
                default:
                    tg3_stat_zero_addr = 1'b0;
            endcase
        end
    endfunction
    wire rd_clear_tx_octets = rd_req_valid && rd_in_range &&
                              (rd_offset[15:0] == MAC_TX_OCTETS_ADDR);
    wire rd_clear_tx_unicast = rd_req_valid && rd_in_range &&
                               (rd_offset[15:0] == MAC_TX_UCAST_ADDR);
    wire rd_clear_tx_multicast = rd_req_valid && rd_in_range &&
                                 (rd_offset[15:0] == MAC_TX_MCAST_ADDR);
    wire rd_clear_tx_broadcast = rd_req_valid && rd_in_range &&
                                 (rd_offset[15:0] == MAC_TX_BCAST_ADDR);
    wire rd_clear_rx_octets = rd_req_valid && rd_in_range &&
                              (rd_offset[15:0] == MAC_RX_OCTETS_ADDR);
    wire rd_clear_rx_unicast = rd_req_valid && rd_in_range &&
                               (rd_offset[15:0] == MAC_RX_UCAST_ADDR);
    wire rd_clear_rx_multicast = rd_req_valid && rd_in_range &&
                                 (rd_offset[15:0] == MAC_RX_MCAST_ADDR);
    wire rd_clear_rx_broadcast = rd_req_valid && rd_in_range &&
                                 (rd_offset[15:0] == MAC_RX_BCAST_ADDR);
    wire tx_stat_event_accepted = dma_tx_stat_event && !reset_generation_active;
    wire rx_stat_event_accepted = dma_rx_stat_event && !reset_generation_active;

    assign interrupt_level = irq_level;
    assign dma_tx_ring_addr     = dma_tx_ring_addr_state;
    assign dma_tx_ring_size     = dma_tx_ring_size_state;
    assign dma_rx_std_ring_addr = dma_rx_std_ring_addr_state;
    assign dma_rx_std_ring_size = dma_rx_std_ring_size_state;
    assign dma_rx_ret_ring_addr = dma_rx_ret_ring_addr_state;
    assign dma_rx_ret_ring_size = dma_rx_ret_ring_size_state;
    assign dma_status_addr      = dma_status_addr_state;
    assign dma_tx_prod_idx      = dma_tx_prod_idx_state;
    assign dma_rx_std_prod_idx  = dma_rx_std_prod_idx_state;
    assign dma_rx_ret_cons_idx  = dma_rx_ret_cons_idx_state;
    assign dma_hostcc_now       = dma_hostcc_now_state;
    assign dma_link_event       = dma_link_event_state;
    assign dma_reset_req        = dma_reset_req_state;

    function automatic [31:0] merge_cfg_bytes(
        input [31:0] old_value,
        input [31:0] new_value,
        input [3:0]  byte_enable
    );
        begin
            merge_cfg_bytes = old_value;
            if (byte_enable[0]) merge_cfg_bytes[7:0]   = new_value[7:0];
            if (byte_enable[1]) merge_cfg_bytes[15:8]  = new_value[15:8];
            if (byte_enable[2]) merge_cfg_bytes[23:16] = new_value[23:16];
            if (byte_enable[3]) merge_cfg_bytes[31:24] = new_value[31:24];
        end
    endfunction

    assign cfg_68_value = misc_host_ctrl |
                          (tagged_status_enabled ? 32'h00000200 :
                                                   32'h00000000);
    assign cfg_6c_value = cfg_6c_state;
    assign cfg_70_value = pci_state;
    wire [31:0] cfg_wr_68_merged = merge_cfg_bytes(
                    cfg_68_value, cfg_wr_data, cfg_wr_be);
    wire [31:0] cfg_wr_6c_merged = merge_cfg_bytes(
                    cfg_6c_state, cfg_wr_data, cfg_wr_be);
    wire [31:0] cfg_wr_70_merged = merge_cfg_bytes(
                    pci_state, cfg_wr_data, cfg_wr_be);

    // NVRAM 间接访问状态。
    bit [31:0] nvram_cmd;
    bit [31:0] nvram_addr;
    bit [31:0] nvram_access;
    bit [31:0] nvram_swarb;
    bit [1:0]  nvram_cmd_phase;
    bit        nvram_cmd_started;
    (* ram_style = "distributed" *) reg [31:0] nvram_rom [0:63];
    initial begin
        $readmemh(NVRAM_INIT_FILE, nvram_rom);
    end
    wire        nvram_in_image = (nvram_addr[31:8] == 24'h0);
    wire [31:0] nvram_rom_word = nvram_rom[nvram_addr[7:2]];
    wire        nvram_sel_mac_7c =
                    nvram_mac_override_valid && (nvram_addr == 32'h0000007C);
    wire        nvram_sel_mac_80 =
                    nvram_mac_override_valid && (nvram_addr == 32'h00000080);
    wire [31:0] nvram_rddata =
                    nvram_sel_mac_7c ? nvram_mac_word_7c :
                    nvram_sel_mac_80 ? nvram_mac_word_80 :
                    nvram_in_image   ? nvram_rom_word    :
                                       32'h00000000;

    // PHY/MDIO 命令状态。驱动通过 0x044C 的忙位轮询命令完成。
    bit [31:0] mi_command;
    bit [31:0] mi_completion;
    bit [1:0]  mi_busy_reads;
    bit [15:0] phy_reg_0;
    bit [1:0]  phy_link_phase;
    bit [15:0] phy_reg_2;
    bit [15:0] phy_reg_3;
    bit [15:0] phy_reg_4;
    bit [15:0] phy_reg_9;
    bit [15:0] phy_reg_10;
    bit [15:0] phy_reg_16;
    bit [15:0] phy_reg_24;
    bit [15:0] phy_reg_28;
    bit        phy_reg_25_first_read_pending;
    bit        phy_aux_7007_seen;

    // 当前连网实卡 trace 在 260 次等待读取后进入就绪状态。
    bit [8:0] firmware_poll_count;

    // 返回当前 PHY 寄存器值；未采集到的寄存器保持为零。
    function automatic [15:0] phy_read_value(input [4:0] reg_index);
        begin
            case (reg_index)
                5'd0:  phy_read_value = phy_reg_0;
                5'd1: begin
                    // BMSR 链路位具有锁存低语义：事件后的第一次读仅报告
                    // 自协商完成，第二次及后续读取才报告链路建立。
                    case (phy_link_phase)
                        2'd0:    phy_read_value = 16'h7949;
                        2'd1:    phy_read_value = 16'h7969;
                        default: phy_read_value = 16'h796D;
                    endcase
                end
                5'd2:  phy_read_value = phy_reg_2;
                5'd3:  phy_read_value = phy_reg_3;
                5'd4:  phy_read_value = phy_reg_4;
                // 伙伴能力和辅助状态均为只读；用链路阶段组合生成，
                // 避免为三个固定返回值保留 48 位可写寄存器。
                // 防抢网：伪造 10M 全双工伙伴（ANLPAR 只报 10BASE-T 全双工），
                // Windows 自动接口 metric 随链路速度劣化到必输真网卡，
                // 网关/DNS 存在但 0.0.0.0/0 与解析永远走真网卡。
                5'd5:  phy_read_value = (phy_link_phase != 2'd0) ?
                                         16'h0040 : 16'h0000;
                5'd9:  phy_read_value = phy_reg_9;
                5'd10: phy_read_value = phy_reg_10;
                5'd16: phy_read_value = phy_reg_16;
                5'd17: phy_read_value = (phy_link_phase != 2'd0) ?
                                         16'h0301 : 16'h0000;
                5'd24: phy_read_value = phy_reg_24;
                // 第一次返回采集到的瞬态值，后续稳定为 10M 全双工
                // （AUX_STAT SPDMASK=10FULL|FULL；伪造低速让 Windows 的
                // 接口 metric 不能作为不抢网保证，默认隔离策略见 ADVERTISE_ROUTER_DNS）。
                5'd25: begin
                    if (phy_link_phase == 2'd0)
                        phy_read_value = 16'h0000;
                    else if (phy_reg_25_first_read_pending)
                        phy_read_value = 16'hFA3F;
                    else
                        phy_read_value = 16'h821F;
                end
                5'd28: phy_read_value = phy_reg_28;
                default: phy_read_value = 16'h0000;
            endcase
        end
    endfunction

    // 所有快照内存写入共用一个写端口；特殊寄存器只修改写入数据，
    // 避免同一数组出现多个写入口而被展开成触发器或分布式 RAM。
    always @* begin
        bar_mem_wr_data = wr_data;

        if (wr_valid && (wr_be == 4'hF)) begin
            case (wr_offset[15:0])
                16'h0438: bar_mem_wr_data = 32'h0000022C;
                16'h045C: bar_mem_wr_data = wr_data & 32'hFFFFFEFF;
                16'h0C0C: bar_mem_wr_data = wr_data & 32'h00000001;
                16'h1800: bar_mem_wr_data = wr_data | 32'h00000010;
                16'h1C00: bar_mem_wr_data = wr_data & 32'h00000002;
                16'h2018: bar_mem_wr_data = wr_data & 32'h00790407;
                16'h2440: bar_mem_wr_data = 32'h00000000;
                16'h2444: bar_mem_wr_data = 32'h00000000;
                16'h2448: bar_mem_wr_data = 32'h00000000;
                16'h244C: bar_mem_wr_data = 32'h00000000;
                16'h2468: bar_mem_wr_data = 32'h00000000;
                16'h3C00: bar_mem_wr_data = wr_data & 32'hFFFFFBF7;
                // 私有控制和计数位于实卡保留区，不允许写入落到快照 BRAM。
                16'h3FF8: bar_mem_wr_data = 32'h00000000;
                16'h3FFC: bar_mem_wr_data = 32'h00000000;
                16'h3C18: bar_mem_wr_data = 32'h00000000;
                16'h3C1C: bar_mem_wr_data = 32'h00000000;
                // 连网采集中主机合并计数清零寄存器写后读零。
                16'h3C28: bar_mem_wr_data = 32'h00000000;
                16'h442C: bar_mem_wr_data = 32'h00000000;
                16'h4430: bar_mem_wr_data = 32'h00000000;
                16'h4434: bar_mem_wr_data = 32'h00000000;
                16'h4438: bar_mem_wr_data = 32'h00000000;
                default: begin
                end
            endcase
        end

        // 私有寄存器对任何字节使能都禁止写入底层快照。
        if (wr_valid &&
            (((wr_offset[15:0] >= 16'h3F80) &&
              (wr_offset[15:0] <= 16'h3FBC)) ||
             ((wr_offset[15:0] >= 16'h3FE0) &&
              (wr_offset[15:0] <= 16'h3FFC)) ||
             ((wr_offset[15:0] >= 16'h3E00) &&
              (wr_offset[15:0] <= 16'h3E7C))))
            bar_mem_wr_data = 32'h00000000;
    end

    // 文件中的每一行对应一个小端序 DWORD，共覆盖完整的 64 KiB BAR0。
    initial begin
        $readmemh(BAR_INIT_FILE, bar_mem);
    end

    // 单写端口、同步读端口符合 7 系列块 RAM 推断模板。
    // 同拍同地址读写使用非阻塞赋值，读取的是写入前的旧值。
    always @ ( posedge clk ) begin
        if (bar_mem_wr_be[0]) bar_mem[bar_mem_wr_addr][7:0]   <= bar_mem_wr_data[7:0];
        if (bar_mem_wr_be[1]) bar_mem[bar_mem_wr_addr][15:8]  <= bar_mem_wr_data[15:8];
        if (bar_mem_wr_be[2]) bar_mem[bar_mem_wr_addr][23:16] <= bar_mem_wr_data[23:16];
        if (bar_mem_wr_be[3]) bar_mem[bar_mem_wr_addr][31:24] <= bar_mem_wr_data[31:24];

        if (!rst && rd_req_valid && rd_in_range)
            rd_data_q <= bar_mem[rd_offset[15:2]];
    end

    always @ ( posedge clk ) begin
        if (rst) begin
            rd_req_ctx_q          <= 0;
            rd_req_valid_q        <= 0;
            rd_in_range_q         <= 0;
            rd_offset_q           <= 0;
            rd_base_q             <= 0;
            rd_special_valid_q    <= 0;
            rd_special_data_q     <= 0;
            rd_rsp_ctx            <= 0;
            rd_rsp_data           <= 0;
            rd_rsp_valid          <= 0;

            misc_host_ctrl        <= 32'h42000000;
            pci_state             <= 32'h000000B2;
            clock_ctrl            <= 32'h000000A0;
            mac_mode              <= 32'h00E04808;
            mac_link_event_pending <= 1'b0;
            mac_link_event_armed   <= 1'b0;
            mac_link_delay_active  <= 1'b0;
            mac_link_delay_count   <= '0;
            rx_cpu_state          <= 32'h80004000;
            rx_cpu_reset_writes   <= 0;
            grc_misc_cfg          <= 32'h3C0850FE;
            grc_local_ctrl        <= 32'h00004F01;
            grc_6838              <= 32'h003C0000;
            tagged_status_enabled <= 1'b0;
            cfg_6c_state          <= 32'h10000000;
            interrupt_mailbox     <= 32'h00000001;
            irq_force_pending     <= 1'b0;
            irq_level             <= 1'b0;
            dbg_mbox_ever_unmasked <= 1'b0;
            dbg_irq_ever_delivered <= 1'b0;
            irq_level_prev         <= 1'b0;
            dbg_mbox_write_count   <= 16'h0000;
            dbg_softreset_count    <= 16'h0000;
            dbg_hostcc_now_count   <= 16'h0000;
            dbg_last_3c00          <= 32'h00000000;
            dbg_last_6800          <= 32'h00000000;

            dma_cfg_sram_addr          <= 0;
            dma_tx_ring_addr_state     <= 0;
            dma_tx_ring_size_state     <= 0;
            dma_rx_std_ring_addr_state <= 0;
            dma_rx_std_ring_size_state <= 0;
            dma_rx_ret_ring_addr_state <= 0;
            dma_rx_ret_ring_size_state <= 0;
            dma_status_addr_state      <= 0;
            dma_tx_prod_idx_state      <= 0;
            dma_rx_std_prod_idx_state  <= 0;
            dma_rx_ret_cons_idx_state  <= 0;
            dma_hostcc_now_state       <= 1'b0;
            dma_link_event_state       <= 1'b0;
            dma_done_count             <= 0;
            rx_stat_octets             <= 0;
            rx_stat_unicast            <= 0;
            rx_stat_multicast          <= 0;
            rx_stat_broadcast          <= 0;
            tx_stat_octets             <= 0;
            tx_stat_unicast            <= 0;
            tx_stat_multicast          <= 0;
            tx_stat_broadcast          <= 0;
            tx_stat_shadow_octets      <= 0;
            tx_stat_shadow_packets     <= 0;
            tx_stat_shadow_unicast     <= 0;
            tx_stat_shadow_multicast   <= 0;
            tx_stat_shadow_broadcast   <= 0;
            rx_stat_shadow_octets      <= 0;
            rx_stat_shadow_packets     <= 0;
            rx_stat_shadow_unicast     <= 0;
            rx_stat_shadow_multicast   <= 0;
            rx_stat_shadow_broadcast   <= 0;

            nvram_cmd             <= 32'h00000108;
            nvram_addr            <= 0;
            nvram_access          <= 0;
            nvram_swarb           <= 0;
            nvram_cmd_phase       <= 0;
            nvram_cmd_started     <= 1'b0;

            mi_command            <= 0;
            mi_completion         <= 0;
            mi_busy_reads         <= 0;
            phy_reg_0             <= 16'h3100;
            phy_link_phase        <= 2'd0;
            phy_reg_2             <= 16'h0020;
            phy_reg_3             <= 16'h6180;
            phy_reg_4             <= 16'h0DE1;
            phy_reg_9             <= 16'h0300;
            phy_reg_10            <= 16'h3800;
            phy_reg_16            <= 16'h0000;
            phy_reg_24            <= 16'h7477;
            phy_reg_28            <= 16'h0000;
            phy_reg_25_first_read_pending <= 1'b0;
            phy_aux_7007_seen      <= 1'b0;

            firmware_poll_count   <= 0;
            soft_reset_pending    <= 1'b0;
            lifecycle_reset_pending <= 1'b0;
            dma_reset_req_state   <= 1'b0;
            soft_reset_guard      <= 4'd0;
            reset_timeout         <= 0;
            irq_holdoff_count     <= 0;
        end
        else begin
            dma_hostcc_now_state <= 1'b0;
            dma_link_event_state <= 1'b0;
            dma_reset_req_state  <= 1'b0;
            if (mac_link_delay_active) begin
                if (mac_link_delay_count[LINK_EVENT_DELAY_BIT]) begin
                    mac_link_delay_active   <= 1'b0;
                    mac_link_event_pending  <= 1'b1;
                    dma_link_event_state    <= 1'b1;
                    // 链路处理开始后，寄存器 25 的首次读取返回瞬态值。
                    phy_reg_25_first_read_pending <= 1'b1;
                end
                else begin
                    mac_link_delay_count <= mac_link_delay_count + 1'b1;
                end
            end
            if (lifecycle_reset_req &&
                !soft_reset_pending &&
                !lifecycle_reset_pending) begin
                lifecycle_reset_pending <= 1'b1;
                dma_reset_req_state     <= 1'b1;
                soft_reset_guard        <= 4'd8;
                reset_timeout           <= 0;
                dma_hostcc_now_state    <= 1'b0;
                dma_link_event_state    <= 1'b0;
                interrupt_mailbox       <= 32'h00000001;
                irq_force_pending       <= 1'b0;
                irq_level               <= 1'b0;
                mac_link_event_pending  <= 1'b0;
                mac_link_event_armed    <= 1'b0;
                mac_link_delay_active   <= 1'b0;
                mac_link_delay_count    <= '0;
                phy_link_phase          <= 2'd0;
                phy_reg_25_first_read_pending <= 1'b0;
            end
            if (soft_reset_pending) begin
                if (!reset_timeout_fired)
                    reset_timeout <= reset_timeout + 1'b1;
                if (!reset_quiescent_ready && !reset_timeout_fired)
                    soft_reset_guard <= 4'd8;
                else if (soft_reset_guard != 0 && !reset_timeout_fired)
                    soft_reset_guard <= soft_reset_guard - 1'b1;
                else begin
                    soft_reset_pending        <= 1'b0;
                    reset_timeout             <= 0;
                    grc_misc_cfg              <= 32'h24000000;
                    pci_state                 <= 32'h000000B2;
                    grc_local_ctrl            <= 32'h00004F01;
                    tagged_status_enabled     <= 1'b0;
                    rx_cpu_reset_writes       <= 0;
                    dma_cfg_sram_addr         <= 0;
                    dma_tx_ring_addr_state    <= 0;
                    dma_tx_ring_size_state    <= 0;
                    dma_rx_std_ring_addr_state <= 0;
                    dma_rx_std_ring_size_state <= 0;
                    dma_rx_ret_ring_addr_state <= 0;
                    dma_rx_ret_ring_size_state <= 0;
                    dma_status_addr_state     <= 0;
                    dma_tx_prod_idx_state     <= 0;
                    dma_rx_std_prod_idx_state <= 0;
                    dma_rx_ret_cons_idx_state <= 0;
                    dma_done_count            <= 0;
                    rx_stat_octets            <= 0;
                    rx_stat_unicast           <= 0;
                    rx_stat_multicast         <= 0;
                    rx_stat_broadcast         <= 0;
                    tx_stat_octets            <= 0;
                    tx_stat_unicast           <= 0;
                    tx_stat_multicast         <= 0;
                    tx_stat_broadcast         <= 0;
                    tx_stat_shadow_octets     <= 0;
                    tx_stat_shadow_packets    <= 0;
                    tx_stat_shadow_unicast    <= 0;
                    tx_stat_shadow_multicast  <= 0;
                    tx_stat_shadow_broadcast  <= 0;
                    rx_stat_shadow_octets     <= 0;
                    rx_stat_shadow_packets    <= 0;
                    rx_stat_shadow_unicast    <= 0;
                    rx_stat_shadow_multicast  <= 0;
                    rx_stat_shadow_broadcast  <= 0;
                end
            end
            else if (lifecycle_reset_pending) begin
                if (!reset_timeout_fired)
                    reset_timeout <= reset_timeout + 1'b1;
                if (!reset_quiescent_ready && !reset_timeout_fired)
                    soft_reset_guard <= 4'd8;
                else if (soft_reset_guard != 0 && !reset_timeout_fired)
                    soft_reset_guard <= soft_reset_guard - 1'b1;
                else begin
                    lifecycle_reset_pending    <= 1'b0;
                    reset_timeout              <= 0;
                    tagged_status_enabled      <= 1'b0;
                    rx_cpu_reset_writes        <= 0;
                    dma_cfg_sram_addr          <= 0;
                    dma_tx_ring_addr_state     <= 0;
                    dma_tx_ring_size_state     <= 0;
                    dma_rx_std_ring_addr_state <= 0;
                    dma_rx_std_ring_size_state <= 0;
                    dma_rx_ret_ring_addr_state <= 0;
                    dma_rx_ret_ring_size_state <= 0;
                    dma_status_addr_state      <= 0;
                    dma_tx_prod_idx_state      <= 0;
                    dma_rx_std_prod_idx_state  <= 0;
                    dma_rx_ret_cons_idx_state  <= 0;
                    dma_done_count             <= 0;
                    rx_stat_octets             <= 0;
                    rx_stat_unicast            <= 0;
                    rx_stat_multicast          <= 0;
                    rx_stat_broadcast          <= 0;
                    tx_stat_octets             <= 0;
                    tx_stat_unicast            <= 0;
                    tx_stat_multicast          <= 0;
                    tx_stat_broadcast          <= 0;
                    tx_stat_shadow_octets      <= 0;
                    tx_stat_shadow_packets     <= 0;
                    tx_stat_shadow_unicast     <= 0;
                    tx_stat_shadow_multicast   <= 0;
                    tx_stat_shadow_broadcast   <= 0;
                    rx_stat_shadow_octets      <= 0;
                    rx_stat_shadow_packets     <= 0;
                    rx_stat_shadow_unicast     <= 0;
                    rx_stat_shadow_multicast   <= 0;
                    rx_stat_shadow_broadcast   <= 0;
                end
            end
            // 会话失效（V1）：BME 掉沿或离开 D0 的沿上作废一次所有主机物理
            // 地址与中断状态，不等 DMA 静默——引擎侧排空由它自己的
            // teardown 保证（已打开的 TLP 一定发完）。作废后这些地址必须
            // 由驱动在本会话重新写入才有效，FPGA 不再可能往上一会话已
            // 释放的内存写状态块。
            bme_prev <= bus_master_enable;
            d0_prev  <= power_state_d0;
            if (session_end) begin
                tagged_status_enabled      <= 1'b0;
                dma_cfg_sram_addr          <= 0;
                dma_tx_ring_addr_state     <= 0;
                dma_tx_ring_size_state     <= 0;
                dma_rx_std_ring_addr_state <= 0;
                dma_rx_std_ring_size_state <= 0;
                dma_rx_ret_ring_addr_state <= 0;
                dma_rx_ret_ring_size_state <= 0;
                dma_status_addr_state      <= 0;
                dma_tx_prod_idx_state      <= 0;
                dma_rx_std_prod_idx_state  <= 0;
                dma_rx_ret_cons_idx_state  <= 0;
                interrupt_mailbox          <= 32'h00000001;
                irq_force_pending          <= 1'b0;
                irq_level                  <= 1'b0;
                mac_link_event_pending     <= 1'b0;
                mac_link_event_armed       <= 1'b0;
                mac_link_delay_active      <= 1'b0;
                mac_link_delay_count       <= '0;
            end
            if (!PRODUCTION &&
                dma_irq_request && !reset_generation_active)
                dma_done_count <= dma_done_count + 1'b1;

            if (rd_clear_rx_octets)
                rx_stat_octets <= rx_stat_event_accepted ?
                    {16'h0000, dma_rx_stat_bytes} : 32'h00000000;
            else if (rx_stat_event_accepted)
                rx_stat_octets <= rx_stat_octets +
                    {16'h0000, dma_rx_stat_bytes};

            if (rd_clear_rx_unicast)
                rx_stat_unicast <= (rx_stat_event_accepted &&
                    (dma_rx_stat_class == 2'd0)) ? 32'd1 : 32'd0;
            else if (rx_stat_event_accepted && (dma_rx_stat_class == 2'd0))
                rx_stat_unicast <= rx_stat_unicast + 1'b1;

            if (rd_clear_rx_multicast)
                rx_stat_multicast <= (rx_stat_event_accepted &&
                    (dma_rx_stat_class == 2'd1)) ? 32'd1 : 32'd0;
            else if (rx_stat_event_accepted && (dma_rx_stat_class == 2'd1))
                rx_stat_multicast <= rx_stat_multicast + 1'b1;

            if (rd_clear_rx_broadcast)
                rx_stat_broadcast <= (rx_stat_event_accepted &&
                    (dma_rx_stat_class == 2'd2)) ? 32'd1 : 32'd0;
            else if (rx_stat_event_accepted && (dma_rx_stat_class == 2'd2))
                rx_stat_broadcast <= rx_stat_broadcast + 1'b1;

            if (rd_clear_tx_octets)
                tx_stat_octets <= tx_stat_event_accepted ?
                    {16'h0000, dma_tx_stat_bytes} : 32'h00000000;
            else if (tx_stat_event_accepted)
                tx_stat_octets <= tx_stat_octets +
                    {16'h0000, dma_tx_stat_bytes};

            if (rd_clear_tx_unicast)
                tx_stat_unicast <= (tx_stat_event_accepted &&
                    (dma_tx_stat_class == 2'd0)) ? 32'd1 : 32'd0;
            else if (tx_stat_event_accepted && (dma_tx_stat_class == 2'd0))
                tx_stat_unicast <= tx_stat_unicast + 1'b1;

            if (rd_clear_tx_multicast)
                tx_stat_multicast <= (tx_stat_event_accepted &&
                    (dma_tx_stat_class == 2'd1)) ? 32'd1 : 32'd0;
            else if (tx_stat_event_accepted && (dma_tx_stat_class == 2'd1))
                tx_stat_multicast <= tx_stat_multicast + 1'b1;

            if (rd_clear_tx_broadcast)
                tx_stat_broadcast <= (tx_stat_event_accepted &&
                    (dma_tx_stat_class == 2'd2)) ? 32'd1 : 32'd0;
            else if (tx_stat_event_accepted && (dma_tx_stat_class == 2'd2))
                tx_stat_broadcast <= tx_stat_broadcast + 1'b1;

            if (!PRODUCTION && rx_stat_event_accepted) begin
                rx_stat_shadow_octets  <= rx_stat_shadow_octets +
                    {16'h0000, dma_rx_stat_bytes};
                rx_stat_shadow_packets <= rx_stat_shadow_packets + 1'b1;
                case (dma_rx_stat_class)
                    2'd0: rx_stat_shadow_unicast   <= rx_stat_shadow_unicast + 1'b1;
                    2'd1: rx_stat_shadow_multicast <= rx_stat_shadow_multicast + 1'b1;
                    2'd2: rx_stat_shadow_broadcast <= rx_stat_shadow_broadcast + 1'b1;
                    default: begin
                    end
                endcase
            end

            if (!PRODUCTION && tx_stat_event_accepted) begin
                tx_stat_shadow_octets  <= tx_stat_shadow_octets +
                    {16'h0000, dma_tx_stat_bytes};
                tx_stat_shadow_packets <= tx_stat_shadow_packets + 1'b1;
                case (dma_tx_stat_class)
                    2'd0: tx_stat_shadow_unicast   <= tx_stat_shadow_unicast + 1'b1;
                    2'd1: tx_stat_shadow_multicast <= tx_stat_shadow_multicast + 1'b1;
                    2'd2: tx_stat_shadow_broadcast <= tx_stat_shadow_broadcast + 1'b1;
                    default: begin
                    end
                endcase
            end

            // 只读诊断：记录中断是否真正向硬核投递过（irq_level 上升沿）。
            if (!PRODUCTION) begin
                irq_level_prev <= irq_level;
                if (irq_level && !irq_level_prev)
                    dbg_irq_ever_delivered <= 1'b1;
            end

            // DMA 完成状态块写入后再请求中断；若驱动暂时屏蔽，
            // 则保留为待处理中断，等待邮箱重新解屏蔽。
            // 待处理中断只在没有控制寄存器写入的拍次投递。
            // 因此邮箱确认后 irq_level 会完整保持一个时钟的低电平。
            // holdoff 计数：中断投递时重新装载；到期后才允许下一次投递。
            if (irq_holdoff_active)
                irq_holdoff_count <= irq_holdoff_count - 1'b1;

            if (irq_pending_delivery) begin
                irq_force_pending <= 1'b0;
                irq_level         <= 1'b1;
                irq_holdoff_count <= 11'd2047;
            end

            // 若当前中断尚未确认、正在投递旧事件或被屏蔽，
            // 新 DMA 事件必须保留到下一次投递。
            if (dma_irq_request && !reset_generation_active) begin
                if (irq_level ||
                    irq_pending_delivery ||
                    irq_holdoff_active ||
                    interrupt_mailbox[0] ||
                    misc_host_ctrl[1])
                    irq_force_pending <= 1'b1;
                else begin
                    irq_level         <= 1'b1;
                    irq_holdoff_count <= 11'd2047;
                end
            end

            // 驱动会先从配置空间写芯片控制寄存器，再立即从 BAR0 镜像读取。
            // 这里与 BAR0 写路径共用同一状态，避免两个地址空间各自保存快照。
            if (cfg_wr_valid) begin
                    case (cfg_wr_dwaddr)
                    10'h01A: begin
                        // bit0(CLEAR_INT)是自清零动作位，读回恒 0，不锁存。
                        misc_host_ctrl <= 32'h42000000 |
                                          (cfg_wr_68_merged & 32'h0000FFFE) |
                                          (tagged_status_enabled ? 32'h00000200 :
                                                                   32'h00000000);
                        if (cfg_wr_68_merged[1] || cfg_wr_68_merged[0]) begin
                            irq_level <= 1'b0;
                            // 与确认同拍到达的新事件必须留待解除屏蔽后投递。
                            if (dma_irq_request && !reset_generation_active)
                                irq_force_pending <= 1'b1;
                        end
                    end
                    10'h01B: cfg_6c_state <= cfg_wr_6c_merged;
                    10'h01C: begin
                        // 0x70 的状态位由硬件保留，驱动写 0x20 后 BAR 镜像为 0xB2。
                        pci_state <= 32'h00000092 |
                                     (cfg_wr_70_merged & 32'h00000020);
                    end
                    10'h01F: begin
                        dma_cfg_sram_addr <= merge_cfg_bytes(
                                                dma_cfg_sram_addr,
                                                cfg_wr_data,
                                                cfg_wr_be);
                    end
                    10'h021: begin
                        // RCB 均由 Windows 驱动按完整 DWORD 写入。
                        if (cfg_wr_be == 4'hF) begin
                            case (dma_cfg_sram_addr[11:0])
                                12'h100: dma_tx_ring_addr_state[63:32] <= cfg_wr_data;
                                12'h104: dma_tx_ring_addr_state[31:0]  <= cfg_wr_data;
                                12'h108: dma_tx_ring_size_state        <= cfg_wr_data[31:16];
                                12'h200: dma_rx_ret_ring_addr_state[63:32] <= cfg_wr_data;
                                12'h204: dma_rx_ret_ring_addr_state[31:0]  <= cfg_wr_data;
                                12'h208: dma_rx_ret_ring_size_state        <= cfg_wr_data[31:16];
                                default: begin
                                end
                            endcase
                        end
                    end
                    default: begin
                    end
                endcase
            end

            if (wr_valid && wr_in_range) begin
                // 实卡采集中的动态寄存器全部采用 32 位访问。
                if (wr_be == 4'hF) begin
                    case (wr_offset[15:0])
                        16'h0204: begin
                            interrupt_mailbox <= wr_data;
                            // 只读诊断：统计邮箱写次数、记录是否解屏蔽过（bit0==0）。
                            if (!PRODUCTION) begin
                                dbg_mbox_write_count <= dbg_mbox_write_count + 1'b1;
                                if (!wr_data[0])
                                    dbg_mbox_ever_unmasked <= 1'b1;
                            end
                            // 任意邮箱写先确认当前中断；bit0 清零表示重新解屏蔽。
                            // tagged 重开值的高字节等于当前状态 tag 时，旧 pending
                            // 已被驱动消费；只保留同拍新完成或尚未消费的新 tag。
                            irq_level <= 1'b0;
                            if (dma_irq_request && !reset_generation_active)
                                irq_force_pending <= 1'b1;
                            else if (!wr_data[0] &&
                                     tagged_status_enabled &&
                                     !reset_generation_active)
                                irq_force_pending <=
                                    (wr_data[31:24] !=
                                     dma_status_tag);
                            else if (!wr_data[0])
                                irq_force_pending <= 1'b0;
                        end
                        16'h026C,
                        16'h0340,
                        16'h0344: dma_rx_std_prod_idx_state <= wr_data[15:0];
                        16'h0284: dma_rx_ret_cons_idx_state  <= wr_data[15:0];
                        16'h0300,
                        16'h0304: dma_tx_prod_idx_state      <= wr_data[15:0];
                        16'h006C: cfg_6c_state <= wr_data;
                        16'h0068: begin
                            // bit0(CLEAR_INT)是自清零动作位，读回恒 0，不锁存。
                            misc_host_ctrl <= 32'h42000000 |
                                              (wr_data & 32'h0000FFFE) |
                                              (tagged_status_enabled ? 32'h00000200 : 32'h00000000);
                            if ((wr_data & 32'h00000200) != 0) begin
                                pci_state      <= 32'h000000B0;
                                grc_local_ctrl <= grc_local_ctrl & 32'hFFFFFFFE;
                            end
                            else if (wr_data[7:0] == 8'h9A &&
                                     pci_state == 32'h000000B2)
                                pci_state <= 32'h000002B2;
                            if (wr_data[1] || wr_data[0]) begin
                                irq_level <= 1'b0;
                                // 与确认同拍到达的新事件不能被后面的清零覆盖。
                                if (dma_irq_request && !reset_generation_active)
                                    irq_force_pending <= 1'b1;
                            end
                        end
                        16'h0074: begin
                            // 低字节的时钟切换请求位会在硬件完成后自动清除。
                            if (wr_data[7:0] == 8'hA0)
                                clock_ctrl <= wr_data & 32'hFFFFFFDF;
                            else
                                clock_ctrl <= wr_data | 32'h000000A0;
                        end
                        // 连网 trace 中 LINK_POLARITY 与统计 CLEAR 位均写后不保持。
                        16'h0400: mac_mode <= wr_data & 32'hFFFF6BFF;
                        16'h0404: begin
                            // MAC_STATUS.LNKSTATE_CHANGED 为 W1C，MI_COMPLETION 保持只读置位。
                            if (wr_data[12])
                                mac_link_event_pending <= 1'b0;
                        end
                        16'h0408: begin
                            // HOSTCC 后驱动重新使能链路事件；先等待约 2.147 秒，
                            // 再写回状态块并触发 MSI。等待期间 PHY 保持 Down。
                            if (wr_data[12] && mac_link_event_armed &&
                                !reset_generation_active) begin
                                mac_link_event_armed   <= 1'b0;
                                mac_link_delay_active <= 1'b1;
                                mac_link_delay_count  <= '0;
                                phy_reg_25_first_read_pending <= 1'b0;
                            end
                        end
                        16'h044C: begin
                            mi_command    <= wr_data;
                            mi_busy_reads <= 2;

                            if ((wr_data & 32'h0C000000) == 32'h08000000) begin
                                // MDIO 读：完成时清除忙位，并把 PHY 数据放入低 16 位。
                                mi_completion <= (wr_data & 32'hDFFF0000) |
                                                 {16'h0000, phy_read_value(wr_data[20:16])};
                                // 任意一次 BMSR 读都结束锁存低窗口；下一次读返回链路已建立。
                                if (wr_data[20:16] == 5'd1)
                                    phy_link_phase <= 2'd2;
                                // 辅助状态第一次返回瞬态值，随后稳定为千兆全双工。
                                if ((wr_data[20:16] == 5'd25) &&
                                    (phy_link_phase != 2'd0) &&
                                    phy_reg_25_first_read_pending)
                                    phy_reg_25_first_read_pending <= 1'b0;
                            end
                            else begin
                                // MDIO 写：完成值保留命令和数据，只清除忙位。
                                mi_completion <= wr_data & 32'hDFFFFFFF;

                                case (wr_data[20:16])
                                    5'd0: begin
                                        // 复位和重新协商位均为自清零位。
                                        if (wr_data[15]) begin
                                            phy_reg_0       <= 16'h3100;
                                            phy_link_phase  <= 2'd0;
                                            phy_reg_25_first_read_pending <= 1'b0;
                                        end
                                        else begin
                                            phy_reg_0 <= wr_data[15:0] & 16'hFDFF;
                                            if (wr_data[9]) begin
                                                phy_link_phase  <= 2'd0;
                                                phy_reg_25_first_read_pending <= 1'b0;
                                            end
                                        end
                                    end
                                    // BMSR 是只读状态寄存器，写命令仅完成握手。
                                    5'd1: begin
                                    end
                                    5'd2:  phy_reg_2  <= wr_data[15:0];
                                    5'd3:  phy_reg_3  <= wr_data[15:0];
                                    5'd4:  phy_reg_4  <= wr_data[15:0];
                                    // ANLPAR、PHY 状态和辅助状态是只读寄存器。
                                    5'd5: begin
                                    end
                                    5'd9:  phy_reg_9  <= wr_data[15:0];
                                    5'd10: phy_reg_10 <= wr_data[15:0];
                                    5'd16: phy_reg_16 <= wr_data[15:0];
                                    5'd17: begin
                                    end
                                    5'd24: begin
                                        // 辅助控制寄存器使用影子页选择，读取值不是简单写回值。
                                        if (wr_data[15:0] == 16'h0007)
                                            phy_reg_24 <= 16'h0400;
                                        else if (wr_data[15:0] == 16'h7007) begin
                                            // 实卡连网 trace 两次选页 0x7007 都回 0x7677。
                                            phy_reg_24 <= 16'h7677;
                                            phy_aux_7007_seen <= 1'b1;
                                        end
                                    end
                                    5'd25: begin
                                    end
                                    5'd28: phy_reg_28 <= wr_data[15:0];
                                    default: begin
                                    end
                                endcase
                            end
                        end
                        16'h5004: begin
                            if (wr_data == 32'hFFFFFFFF) begin
                                if (rx_cpu_reset_writes == 0)
                                    rx_cpu_state <= 32'h80008000;
                                else
                                    rx_cpu_state <= 32'h00000400;
                                if (rx_cpu_reset_writes != 2'b11)
                                    rx_cpu_reset_writes <= rx_cpu_reset_writes + 1'b1;
                            end
                            else
                                rx_cpu_state <= wr_data;
                        end
                        16'h6800: begin
                            if (!PRODUCTION)
                                dbg_last_6800 <= wr_data;
                            if (wr_data == 32'h04130034)
                                tagged_status_enabled <= 1'b1;
                        end
                        16'h6804: begin
                            if (wr_data == 32'h24000001) begin
                                if (!PRODUCTION)
                                    dbg_softreset_count <= dbg_softreset_count + 1'b1;
                                // 软件复位完成后复位位清零，并重新开始固件握手。
                                grc_misc_cfg          <= 32'h24000001;
                                soft_reset_pending    <= 1'b1;
                                lifecycle_reset_pending <= 1'b0;
                                dma_reset_req_state   <= 1'b1;
                                soft_reset_guard      <= 4'd8;
                                reset_timeout         <= 0;
                                dma_link_event_state  <= 1'b0;
                                firmware_poll_count   <= 0;
                                interrupt_mailbox     <= 32'h00000001;
                                irq_force_pending     <= 1'b0;
                                irq_level             <= 1'b0;
                                mac_link_event_pending <= 1'b0;
                                mac_link_event_armed   <= 1'b0;
                                mac_link_delay_active  <= 1'b0;
                                mac_link_delay_count   <= '0;
                                phy_link_phase         <= 2'd0;
                                phy_reg_25_first_read_pending <= 1'b0;
                            end
                            else if (wr_data == 32'h04000082)
                                grc_misc_cfg <= 32'h1C084082;
                            else
                                grc_misc_cfg <= wr_data;
                        end
                        16'h6808: begin
                            if (wr_data == 32'h01004808)
                                grc_local_ctrl <= 32'h01004F09;
                            else
                                grc_local_ctrl <= wr_data;
                        end
                        16'h6838: grc_6838 <= wr_data & 32'hDFFFFFFF;
                        16'h3C00: begin
                            if (!PRODUCTION) begin
                                dbg_last_3c00 <= wr_data;
                                if (wr_data[3])
                                    dbg_hostcc_now_count <= dbg_hostcc_now_count + 1'b1;
                            end
                            // HOSTCC.NOW 必须先让 DMA 写回 UPDATED/tag 状态块，
                            // 禁止直接制造没有状态数据的“空中断”。
                            if (wr_data[3]) begin
                                dma_hostcc_now_state <= 1'b1;
                                if (!reset_generation_active)
                                    mac_link_event_armed <= 1'b1;
                            end
                        end
                        // 0x3D98--0x3FFF 是芯片保留区；完整 DWORD 写 1
                        // 只产生一个手动数据面启动脉冲，读回始终为零。
                        16'h3FF8:
                            if (wr_data == 32'h00000001)
                                dma_hostcc_now_state <= 1'b1;
                        16'h2450: dma_rx_std_ring_addr_state[63:32] <= wr_data;
                        16'h2454: dma_rx_std_ring_addr_state[31:0]  <= wr_data;
                        16'h2458: dma_rx_std_ring_size_state        <= wr_data[31:16];
                        16'h3C38: dma_status_addr_state[63:32]      <= wr_data;
                        16'h3C3C: dma_status_addr_state[31:0]       <= wr_data;
                        16'h7C00: begin
                            // 此写入启动片上固件。
                            firmware_poll_count <= 0;
                        end
                        16'h7000: begin
                            nvram_cmd         <= wr_data;
                            nvram_cmd_phase   <= 0;
                            nvram_cmd_started <= 1'b1;
                        end
                        16'h700C: nvram_addr   <= wr_data;
                        16'h7020: begin
                            if (wr_data[1])
                                nvram_swarb <= 32'h00002200;
                            else if (wr_data[5])
                                nvram_swarb <= 32'h00000000;
                            else
                                nvram_swarb <= wr_data;
                        end
                        16'h7024: nvram_access <= wr_data;
                        default: begin
                        end
                    endcase
                end
            end

            rd_req_ctx_q       <= rd_req_ctx;
            rd_req_valid_q     <= rd_req_valid;
            rd_in_range_q      <= rd_in_range;
            rd_offset_q        <= rd_offset[15:0];
            rd_base_q          <= {rd_req_addr[31:16], 16'h0000};
            rd_special_valid_q <= 1'b0;

            // 生产模式：0x3E00–0x3FFC 不进特殊读，走 bar_mem 真卡快照。
            if (rd_req_valid &&
                !(PRODUCTION &&
                  (rd_offset[15:0] >= 16'h3E00) &&
                  (rd_offset[15:0] <= 16'h3FFC))) begin
                // 动态寄存器在请求周期锁存返回值，随后与普通 BRAM 读取统一流水输出。
                case (rd_offset[15:0])
                    16'h0204: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= interrupt_mailbox;
                    end
                    16'h026C,
                    16'h0340,
                    16'h0344: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <=
                            {16'h0000, dma_rx_std_prod_idx_state};
                    end
                    16'h0284: begin
                        // 直接返回驱动实际写入的 RX return consumer，
                        // 避免离线快照中的旧值干扰数据面诊断。
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <=
                            {16'h0000, dma_rx_ret_cons_idx_state};
                    end
                    16'h0300,
                    16'h0304: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <=
                            {16'h0000, dma_tx_prod_idx_state};
                    end
                    16'h0068: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= misc_host_ctrl |
                                              (tagged_status_enabled ?
                                               32'h00000200 : 32'h00000000);
                    end
                    16'h006C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= cfg_6c_state;
                    end
                    16'h0070: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= pci_state;
                    end
                    16'h0074: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= clock_ctrl;
                    end
                    16'h0400: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= mac_mode;
                    end
                    16'h0410: begin
                        if (nvram_mac_override_valid) begin
                            rd_special_valid_q <= 1'b1;
                            rd_special_data_q  <= nvram_mac_word_7c;
                        end
                    end
                    16'h0414: begin
                        if (nvram_mac_override_valid) begin
                            rd_special_valid_q <= 1'b1;
                            rd_special_data_q  <= nvram_mac_word_80;
                        end
                    end
                    16'h3FF8: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= 32'h00000000;
                    end
                    // 冻结帧诊断窗口：元数据、前 48 字节和 278--285。
                    // flags bit0=1 表示已改冻第一帧 DHCP OFFER。
                    16'h3F80: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[223:192];
                    end
                    16'h3F84: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[255:224];
                    end
                    16'h3F88: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[287:256];
                    end
                    16'h3F8C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[319:288];
                    end
                    16'h3F90: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[351:320];
                    end
                    16'h3F94: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[383:352];
                    end
                    16'h3F98: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[415:384];
                    end
                    16'h3F9C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[447:416];
                    end
                    16'h3FA0: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[479:448];
                    end
                    16'h3FA4: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[511:480];
                    end
                    16'h3FA8: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[543:512];
                    end
                    16'h3FAC: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[575:544];
                    end
                    16'h3FB0: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[607:576];
                    end
                    16'h3FB4: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[639:608];
                    end
                    16'h3FB8: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[671:640];
                    end
                    16'h3FBC: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[703:672];
                    end
                    // 只读中断诊断窗口。
                    // 0x3FC0 状态位：
                    //   [0]=驱动曾解屏蔽(写0x204 bit0=0)  [1]=中断曾投递(irq_level上升沿)
                    //   [2]=irq_level  [3]=irq_force_pending  [4]=当前邮箱屏蔽(mbox[0])
                    //   [5]=tagged_status_enabled  [6]=misc_host_ctrl[1]
                    16'h3FC0: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= {
                            25'h0,
                            misc_host_ctrl[1],
                            tagged_status_enabled,
                            interrupt_mailbox[0],
                            irq_force_pending,
                            irq_level,
                            dbg_irq_ever_delivered,
                            dbg_mbox_ever_unmasked
                        };
                    end
                    16'h3FC4: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= interrupt_mailbox;
                    end
                    16'h3FC8: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= {16'h0, dbg_mbox_write_count};
                    end
                    16'h3FCC: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_status_addr_state[31:0];
                    end
                    16'h3FD0: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_rx_ret_ring_addr_state[31:0];
                    end
                    16'h3FD4: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_rx_std_ring_addr_state[31:0];
                    end
                    16'h3FD8: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_tx_ring_addr_state[31:0];
                    end
                    // 二期只读诊断：init 循环刻画。
                    // 0x3E00 = {软复位次数[31:16], HOSTCC.NOW次数[15:0]}
                    16'h3E00: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= {dbg_softreset_count, dbg_hostcc_now_count};
                    end
                    16'h3E04: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dbg_last_3c00;
                    end
                    16'h3E08: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dbg_last_6800;
                    end
                    16'h3E10: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[31:0];
                    end
                    16'h3E14: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[63:32];
                    end
                    16'h3E18: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[95:64];
                    end
                    16'h3E1C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[127:96];
                    end
                    16'h3E20: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[159:128];
                    end
                    16'h3E24: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[191:160];
                    end
                    16'h3E28: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[223:192];
                    end
                    16'h3E2C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[255:224];
                    end
                    // 第一帧 DHCP OFFER 的 RX 交付冻结：地址/opaque/idx_len/status。
                    16'h3E68: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[287:256];
                    end
                    16'h3E6C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[319:288];
                    end
                    16'h3E70: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[351:320];
                    end
                    16'h3E74: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[383:352];
                    end
                    16'h3E78: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[415:384];
                    end
                    16'h3E7C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug_ext[447:416];
                    end
                    16'h3E30: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= irq_debug[31:0];
                    end
                    16'h3E34: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= irq_debug[63:32];
                    end
                    16'h3E38: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= irq_debug[95:64];
                    end
                    16'h3E3C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= irq_debug[127:96];
                    end
                    // 0x3E40--0x3E64：统计影子计数，不 read-clear，供上板对照驱动读数。
                    16'h3E40: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_shadow_octets;
                    end
                    16'h3E44: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_shadow_packets;
                    end
                    16'h3E48: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_shadow_unicast;
                    end
                    16'h3E4C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_shadow_multicast;
                    end
                    16'h3E50: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_shadow_broadcast;
                    end
                    16'h3E54: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_shadow_octets;
                    end
                    16'h3E58: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_shadow_packets;
                    end
                    16'h3E5C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_shadow_unicast;
                    end
                    16'h3E60: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_shadow_multicast;
                    end
                    16'h3E64: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_shadow_broadcast;
                    end
                    16'h3FE0: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[31:0];
                    end
                    16'h3FE4: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[63:32];
                    end
                    16'h3FE8: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[95:64];
                    end
                    16'h3FEC: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[127:96];
                    end
                    16'h3FF0: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[159:128];
                    end
                    16'h3FF4: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_debug[191:160];
                    end
                    16'h3FFC: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= dma_done_count;
                    end
                    16'h0404: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= 32'h00400000 |
                                              {19'd0, mac_link_event_pending,
                                               12'd0};
                        // 实卡在事件使能后仍报告 Down；只有驱动读到链路事件，
                        // 随后的 BMSR 命令才进入 0x7969 -> 0x796D 序列。
                        if (mac_link_event_pending &&
                            (phy_link_phase == 2'd0))
                            phy_link_phase <= 2'd1;
                    end
                    MAC_TX_OCTETS_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_octets;
                    end
                    MAC_TX_UCAST_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_unicast;
                    end
                    MAC_TX_MCAST_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_multicast;
                    end
                    MAC_TX_BCAST_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= tx_stat_broadcast;
                    end
                    MAC_RX_OCTETS_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_octets;
                    end
                    MAC_RX_UCAST_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_unicast;
                    end
                    MAC_RX_MCAST_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_multicast;
                    end
                    MAC_RX_BCAST_ADDR: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_stat_broadcast;
                    end
                    16'h0460: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= 32'h00000008;
                    end
                    16'h044C: begin
                        rd_special_valid_q <= 1'b1;
                        if (mi_busy_reads != 0) begin
                            rd_special_data_q <= mi_command;
                            mi_busy_reads     <= mi_busy_reads - 1'b1;
                        end
                        else
                            rd_special_data_q <= mi_completion;
                    end
                    16'h5004: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= rx_cpu_state;
                    end
                    16'h6804: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= grc_misc_cfg;
                    end
                    16'h6808: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= grc_local_ctrl;
                    end
                    16'h6838: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= grc_6838;
                    end
                    16'h7000: begin
                        rd_special_valid_q <= 1'b1;
                        if (!nvram_cmd_started)
                            rd_special_data_q <= 32'h00000108;
                        else begin
                            case (nvram_cmd_phase)
                                2'd0: rd_special_data_q <= 32'h00000190;
                                2'd1: rd_special_data_q <= 32'h00000180;
                                default: rd_special_data_q <= 32'h00000188;
                            endcase
                            if (nvram_cmd_phase != 2'd2)
                                nvram_cmd_phase <= nvram_cmd_phase + 1'b1;
                        end
                    end
                    16'h700C: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= nvram_addr;
                    end
                    16'h7010: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= nvram_rddata;
                    end
                    16'h7014: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= 32'h02008273;
                    end
                    16'h7020: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= nvram_swarb;
                    end
                    16'h7024: begin
                        rd_special_valid_q <= 1'b1;
                        rd_special_data_q  <= nvram_access;
                    end
                    16'h8B50: begin
                        rd_special_valid_q <= 1'b1;
                        if (firmware_poll_count < 9'd260) begin
                            rd_special_data_q   <= 32'h4B657654;
                            firmware_poll_count <= firmware_poll_count + 1'b1;
                        end
                        else
                            rd_special_data_q <= 32'hB49A89AB;
                    end
                    default: begin
                        if (tg3_stat_zero_addr(rd_offset[15:0])) begin
                            rd_special_valid_q <= 1'b1;
                            rd_special_data_q  <= 32'h00000000;
                        end
                    end
                endcase
            end

            // 与控制器内其他 BAR 实现保持相同的读取流水线。
            rd_rsp_ctx   <= rd_req_ctx_q;
            rd_rsp_valid <= rd_req_valid_q;

            if (!rd_in_range_q) begin
                rd_rsp_data <= 32'h00000000;
            end
            else if (rd_special_valid_q) begin
                rd_rsp_data <= rd_special_data_q;
            end
            else begin
                case (rd_offset_q)
                    // BAR0 前 256 字节是配置空间镜像，基址必须跟随当前枚举结果。
                    16'h0010: rd_rsp_data <= rd_base_q | 32'h00000004;
                    16'h0014: rd_rsp_data <= 32'h00000000;
                    default:  rd_rsp_data <= rd_data_q;
                endcase
            end
        end
    end

endmodule
