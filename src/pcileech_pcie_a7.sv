//
// PCILeech FPGA.
//
// PCIe module for Artix-7.
//
// (c) Ulf Frisk, 2018-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps
`include "pcileech_header.svh"

module pcileech_pcie_a7 #(
    parameter PRODUCTION = 0
)(
    input                   clk_sys,
    input                   rst,

    // PCIe fabric
    output  [0:0]           pcie_tx_p,
    output  [0:0]           pcie_tx_n,
    input   [0:0]           pcie_rx_p,
    input   [0:0]           pcie_rx_n,
    input                   pcie_clk_p,
    input                   pcie_clk_n,
    input                   pcie_perst_n,
    
    // State and Activity LEDs
    output                  led_state,
    
    // PCIe <--> FIFOs
    IfPCIeFifoCfg.mp_pcie   dfifo_cfg,
    IfPCIeFifoTlp.mp_pcie   dfifo_tlp,
    IfPCIeFifoCore.mp_pcie  dfifo_pcie,
    IfShadow2Fifo.shadow    dshadow2fifo
    );
       
    // ----------------------------------------------------------------------------
    // PCIe DEFINES AND WIRES
    // ----------------------------------------------------------------------------
    
    IfPCIeSignals           ctx();
    IfPCIeTlpRxTx           tlp_tx();
    IfPCIeTlpRxTx           tlp_rx();
    IfAXIS128               tlps_tx();
    IfAXIS128               tlps_rx();
    
    IfAXIS128               tlps_static();       // static tlp transmit from cfg->tlp
    wire [15:0]             pcie_id;
    wire                    user_lnk_up;
    
    // system interface
    wire pcie_clk_c;
    wire clk_pcie;
    wire rst_pcie_user;
    wire rst_subsys = rst || rst_pcie_user || dfifo_pcie.pcie_rst_subsys;
    wire rst_pcie = rst || ~pcie_perst_n || dfifo_pcie.pcie_rst_core;
    wire rst_lifecycle = rst_subsys;
       
    // Buffer for differential system clock
    IBUFDS_GTE2 refclk_ibuf (.O(pcie_clk_c), .ODIV2(), .I(pcie_clk_p), .CEB(1'b0), .IB(pcie_clk_n));
    
    // ----------------------------------------------------
    // TickCount64 PCIe REFCLK and LED OUTPUT
    // ----------------------------------------------------

    time tickcount64_pcie_refclk = 0;
    always @ ( posedge pcie_clk_c )
        tickcount64_pcie_refclk <= tickcount64_pcie_refclk + 1;
    assign led_state = user_lnk_up || tickcount64_pcie_refclk[25];
    
    // ----------------------------------------------------------------------------
    // PCIe CFG RX/TX <--> FIFO below
    // ----------------------------------------------------------------------------
    wire [31:0] base_address_register;
    wire [31:0] base_address_register_1;
    wire [31:0] base_address_register_2;
    wire [31:0] base_address_register_3;
    wire [31:0] base_address_register_4;
    wire [31:0] base_address_register_5;
    wire [127:0] irq_debug;

    wire        int_enable;     // 中断触发器(Interrupt trigger signal)
    wire        msix_vaild;     // msix生效(MSIX signal takes effect)
    wire        msix_send_done; // msix发送完成(MSIX send done)
    wire [31:0] msix_address;   // msix地址(MSIX address)
    wire [31:0] msix_vector;    // msix中断向量(MSIX interrupt vector)
    
    wire        dst64_idle;
    wire        tlp_data_plane_quiescent;
    wire        cfg_trn_pending_i = !tlp_data_plane_quiescent;
    wire        cfg_turnoff_ok_i  = ctx.cfg_to_turnoff &&
                                    tlp_data_plane_quiescent;
    pcileech_pcie_cfg_a7 i_pcileech_pcie_cfg_a7(
        .rst                        ( rst_subsys                ),
        .clk_sys                    ( clk_sys                   ),
        .clk_pcie                   ( clk_pcie                  ),
        .dfifo                      ( dfifo_cfg                 ),        
        .ctx                        ( ctx                       ),
        .tlps_static                ( tlps_static.source        ),
        .cfg_trn_pending_i          ( cfg_trn_pending_i         ),
        .cfg_turnoff_ok_i           ( cfg_turnoff_ok_i          ),
        .pcie_id                    ( pcie_id                   ),  // -> [15:0]
        .int_enable                 ( int_enable                ),
        .msix_vaild                 ( msix_vaild                ),
        .msix_address               ( msix_address              ),
        .msix_vector                ( msix_vector               ),
        .msix_send_done             ( msix_send_done            ),
        .irq_debug                  ( irq_debug                 ),
        .base_address_register      ( base_address_register     ),  // bar0 base register 基地址
        .base_address_register_1    ( base_address_register_1   ),  // bar1 base register 基地址
        .base_address_register_2    ( base_address_register_2   ),  // bar2 base register 基地址
        .base_address_register_3    ( base_address_register_3   ),  // bar3 base register 基地址
        .base_address_register_4    ( base_address_register_4   ),  // bar4 base register 基地址
        .base_address_register_5    ( base_address_register_5   )   // bar5 base register 基地址
    );
    
    // ----------------------------------------------------------------------------
    // PCIe TLP RX/TX <--> FIFO below
    // ----------------------------------------------------------------------------
    
    pcileech_tlps128_src64 i_pcileech_tlps128_src64(
        .rst                        ( rst_subsys                ),
        .clk_pcie                   ( clk_pcie                  ),
        .tlp_rx                     ( tlp_rx.sink               ),
        .tlps_out                   ( tlps_rx.source_lite       )
    );

    wire        tlp_tx_packet_done;
    wire [2:0]  tlp_tx_packet_src;

    // 只把去抖后的 FLR/热复位做成单拍清环。BME/PM 抖动不是复位源
    // （PMCSR 已由 i_pmcsr_d0_filter 处理）。PERST# 已在 rst_pcie 里。
    // 7 系列硬核 FLR 端口按 PG054 不实现，仍接线以免将来/仿真注入漏掉。
    wire        lifecycle_reset_req;
    pcileech_lifecycle_reset_filter i_lifecycle_reset_filter(
        .clk                        ( clk_pcie                          ),
        .rst                        ( rst_subsys                        ),
        .flr                        ( ctx.cfg_received_func_lvl_rst     ),
        .hot_rst                    ( ctx.pl_received_hot_rst           ),
        .ltssm_state                ( ctx.pl_ltssm_state                ),
        .lifecycle_reset_req        ( lifecycle_reset_req               )
    );

    // 硬核 cfg_pmcsr_powerstate 在 CfgWr PMCSR / 内部更新时可能出现数拍
    // 非 D0 毛刺。tg3_dma 把 power_state_d0 编进 tx_config_ready，一拍 0
    // 就会清 dma_runtime_armed，必须再写 HOSTCC.NOW 才能恢复。
    wire        power_state_d0;
    pcileech_pmcsr_d0_filter i_pmcsr_d0_filter(
        .clk                        ( clk_pcie                  ),
        .rst                        ( rst_subsys                ),
        .pmcsr_powerstate           ( ctx.cfg_pmcsr_powerstate  ),
        .power_state_d0             ( power_state_d0            )
    );
    
    pcileech_pcie_tlp_a7 #(
        .PRODUCTION                 ( PRODUCTION                )
    ) i_pcileech_pcie_tlp_a7(
        .rst                        ( rst_subsys                ),
        .clk_pcie                   ( clk_pcie                  ),
        .clk_sys                    ( clk_sys                   ),
        .dfifo                      ( dfifo_tlp                 ),
        .tlps_tx                    ( tlps_tx.source            ),       
        .tlps_rx                    ( tlps_rx.sink_lite         ),
        .tlps_static                ( tlps_static.sink          ),
        .dshadow2fifo               ( dshadow2fifo              ),
        .pcie_id                    ( pcie_id                   ),  // <- [15:0]
        .bus_master_enable          ( ctx.cfg_command[2]        ),
        .lifecycle_reset_req        ( lifecycle_reset_req       ),
        .pl_ltssm_state             ( ctx.pl_ltssm_state        ),  // <- [5:0]
        .power_state_d0             ( power_state_d0            ),
        .dst64_idle                 ( dst64_idle                ),
        .tlp_tx_packet_done         ( tlp_tx_packet_done        ),
        .tlp_tx_packet_src          ( tlp_tx_packet_src         ),
        .data_plane_quiescent       ( tlp_data_plane_quiescent  ),
        .irq_debug                  ( irq_debug                 ),
        .int_enable                 ( int_enable                ),
        .msix_vaild                 ( msix_vaild                ),
        .msix_send_done             ( msix_send_done            ),
        .msix_address               ( msix_address              ),
        .msix_vector                ( msix_vector               ),
        .base_address_register      ( base_address_register     ),  // bar0 base register 基地址
        .base_address_register_1    ( base_address_register_1   ),  // bar1 base register 基地址
        .base_address_register_2    ( base_address_register_2   ),  // bar2 base register 基地址
        .base_address_register_3    ( base_address_register_3   ),  // bar3 base register 基地址
        .base_address_register_4    ( base_address_register_4   ),  // bar4 base register 基地址
        .base_address_register_5    ( base_address_register_5   )   // bar5 base register 基地址
    );
    
    pcileech_tlps128_dst64 i_pcileech_tlps128_dst64(
        .rst                        ( rst_lifecycle             ),
        .clk_pcie                   ( clk_pcie                  ),
        .tlp_tx                     ( tlp_tx.source             ),
        .tlps_in                    ( tlps_tx.sink              ),
        .idle                       ( dst64_idle                ),
        .packet_done                ( tlp_tx_packet_done        ),
        .packet_src                 ( tlp_tx_packet_src         )
    );
    
    // ----------------------------------------------------------------------------
    // PCIe CORE BELOW
    // ---------------------------------------------------------------------------- 
      
    pcie_7x_0 i_pcie_7x_0 (
        // pcie_7x_mgt
        .pci_exp_txp                ( pcie_tx_p                 ),  // ->
        .pci_exp_txn                ( pcie_tx_n                 ),  // ->
        .pci_exp_rxp                ( pcie_rx_p                 ),  // <-
        .pci_exp_rxn                ( pcie_rx_n                 ),  // <-
        .sys_clk                    ( pcie_clk_c                ),  // <-
        .sys_rst_n                  ( ~rst_pcie                 ),  // <-
    
        // s_axis_tx (transmit data)
        .s_axis_tx_tdata            ( tlp_tx.data               ),  // <- [63:0]
        .s_axis_tx_tkeep            ( tlp_tx.keep               ),  // <- [7:0]
        .s_axis_tx_tlast            ( tlp_tx.last               ),  // <-
        .s_axis_tx_tready           ( tlp_tx.ready              ),  // ->
        .s_axis_tx_tuser            ( 4'b0                      ),  // <- [3:0]
        .s_axis_tx_tvalid           ( tlp_tx.valid              ),  // <-
    
        // s_axis_rx (receive data)
        .m_axis_rx_tdata            ( tlp_rx.data               ),  // -> [63:0]
        .m_axis_rx_tkeep            ( tlp_rx.keep               ),  // -> [7:0]
        .m_axis_rx_tlast            ( tlp_rx.last               ),  // -> 
        .m_axis_rx_tready           ( tlp_rx.ready              ),  // <-
        .m_axis_rx_tuser            ( tlp_rx.user               ),  // -> [21:0]
        .m_axis_rx_tvalid           ( tlp_rx.valid              ),  // ->
    
        // pcie_cfg_mgmt
        .cfg_mgmt_dwaddr            ( ctx.cfg_mgmt_dwaddr       ),  // <- [9:0]
        .cfg_mgmt_byte_en           ( ctx.cfg_mgmt_byte_en      ),  // <- [3:0]
        .cfg_mgmt_do                ( ctx.cfg_mgmt_do           ),  // -> [31:0]
        .cfg_mgmt_rd_en             ( ctx.cfg_mgmt_rd_en        ),  // <-
        .cfg_mgmt_rd_wr_done        ( ctx.cfg_mgmt_rd_wr_done   ),  // ->
        .cfg_mgmt_wr_readonly       ( ctx.cfg_mgmt_wr_readonly  ),  // <-
        .cfg_mgmt_wr_rw1c_as_rw     ( ctx.cfg_mgmt_wr_rw1c_as_rw ), // <-
        .cfg_mgmt_di                ( ctx.cfg_mgmt_di           ),  // <- [31:0]
        .cfg_mgmt_wr_en             ( ctx.cfg_mgmt_wr_en        ),  // <-
    
        // special core config
        //.pcie_cfg_vend_id           ( dfifo_pcie.pcie_cfg_vend_id       ),  // <- [15:0]
        //.pcie_cfg_dev_id            ( dfifo_pcie.pcie_cfg_dev_id        ),  // <- [15:0]
        //.pcie_cfg_rev_id            ( dfifo_pcie.pcie_cfg_rev_id        ),  // <- [7:0]
        //.pcie_cfg_subsys_vend_id    ( dfifo_pcie.pcie_cfg_subsys_vend_id ), // <- [15:0]
        //.pcie_cfg_subsys_id         ( dfifo_pcie.pcie_cfg_subsys_id     ),  // <- [15:0]
    
        // pcie2_cfg_interrupt
        .cfg_interrupt_assert       ( ctx.cfg_interrupt_assert          ),  // <-
        .cfg_interrupt              ( ctx.cfg_interrupt                 ),  // <-
        .cfg_interrupt_mmenable     ( ctx.cfg_interrupt_mmenable        ),  // -> [2:0]
        .cfg_interrupt_msienable    ( ctx.cfg_interrupt_msienable       ),  // ->
        .cfg_interrupt_msixenable   ( ctx.cfg_interrupt_msixenable      ),  // ->
        .cfg_interrupt_msixfm       ( ctx.cfg_interrupt_msixfm          ),  // ->
        .cfg_pciecap_interrupt_msgnum ( ctx.cfg_pciecap_interrupt_msgnum ), // <- [4:0]
        .cfg_interrupt_rdy          ( ctx.cfg_interrupt_rdy             ),  // ->
        .cfg_interrupt_do           ( ctx.cfg_interrupt_do              ),  // -> [7:0]
        .cfg_interrupt_stat         ( ctx.cfg_interrupt_stat            ),  // <-
        .cfg_interrupt_di           ( ctx.cfg_interrupt_di              ),  // <- [7:0]
        
        // pcie2_cfg_control
        .cfg_ds_bus_number          ( ctx.cfg_bus_number                ),  // <- [7:0]
        .cfg_ds_device_number       ( ctx.cfg_device_number             ),  // <- [4:0]
        .cfg_ds_function_number     ( ctx.cfg_function_number           ),  // <- [2:0]
        .cfg_dsn                    ( ctx.cfg_dsn                       ),  // <- [63:0]
        .cfg_pm_force_state         ( ctx.cfg_pm_force_state            ),  // <- [1:0]
        .cfg_pm_force_state_en      ( ctx.cfg_pm_force_state_en         ),  // <-
        .cfg_pm_halt_aspm_l0s       ( ctx.cfg_pm_halt_aspm_l0s          ),  // <-
        .cfg_pm_halt_aspm_l1        ( ctx.cfg_pm_halt_aspm_l1           ),  // <-
        .cfg_pm_send_pme_to         ( ctx.cfg_pm_send_pme_to            ),  // <-
        .cfg_pm_wake                ( ctx.cfg_pm_wake                   ),  // <-
        .rx_np_ok                   ( ctx.rx_np_ok                      ),  // <-
        .rx_np_req                  ( ctx.rx_np_req                     ),  // <-
        .cfg_trn_pending            ( ctx.cfg_trn_pending               ),  // <-
        .cfg_turnoff_ok             ( ctx.cfg_turnoff_ok                ),  // <-
        .tx_cfg_gnt                 ( ctx.tx_cfg_gnt                    ),  // <-
        
        // pcie2_cfg_status
        .cfg_command                ( ctx.cfg_command                   ),  // -> [15:0]
        .cfg_bus_number             ( ctx.cfg_bus_number                ),  // -> [7:0]
        .cfg_device_number          ( ctx.cfg_device_number             ),  // -> [4:0]
        .cfg_function_number        ( ctx.cfg_function_number           ),  // -> [2:0]
        .cfg_root_control_pme_int_en( ctx.cfg_root_control_pme_int_en   ),  // ->
        .cfg_bridge_serr_en         ( ctx.cfg_bridge_serr_en            ),  // ->
        .cfg_dcommand               ( ctx.cfg_dcommand                  ),  // -> [15:0]
        .cfg_dcommand2              ( ctx.cfg_dcommand2                 ),  // -> [15:0]
        .cfg_dstatus                ( ctx.cfg_dstatus                   ),  // -> [15:0]
        .cfg_lcommand               ( ctx.cfg_lcommand                  ),  // -> [15:0]
        .cfg_lstatus                ( ctx.cfg_lstatus                   ),  // -> [15:0]
        .cfg_pcie_link_state        ( ctx.cfg_pcie_link_state           ),  // -> [2:0]
        .cfg_pmcsr_pme_en           ( ctx.cfg_pmcsr_pme_en              ),  // ->
        .cfg_pmcsr_pme_status       ( ctx.cfg_pmcsr_pme_status          ),  // ->
        .cfg_pmcsr_powerstate       ( ctx.cfg_pmcsr_powerstate          ),  // -> [1:0]
        .cfg_received_func_lvl_rst  ( ctx.cfg_received_func_lvl_rst     ),  // ->
        .cfg_status                 ( ctx.cfg_status                    ),  // -> [15:0]
        .cfg_to_turnoff             ( ctx.cfg_to_turnoff                ),  // ->
        .tx_buf_av                  ( ctx.tx_buf_av                     ),  // -> [5:0]
        .tx_cfg_req                 ( ctx.tx_cfg_req                    ),  // ->
        .tx_err_drop                ( ctx.tx_err_drop                   ),  // ->
        .cfg_vc_tcvc_map            ( ctx.cfg_vc_tcvc_map               ),  // -> [6:0]
        .cfg_aer_rooterr_corr_err_received          ( ctx.cfg_aer_rooterr_corr_err_received             ),  // ->
        .cfg_aer_rooterr_corr_err_reporting_en      ( ctx.cfg_aer_rooterr_corr_err_reporting_en         ),  // ->
        .cfg_aer_rooterr_fatal_err_received         ( ctx.cfg_aer_rooterr_fatal_err_received            ),  // ->
        .cfg_aer_rooterr_fatal_err_reporting_en     ( ctx.cfg_aer_rooterr_fatal_err_reporting_en        ),  // ->
        .cfg_aer_rooterr_non_fatal_err_received     ( ctx.cfg_aer_rooterr_non_fatal_err_received        ),  // ->
        .cfg_aer_rooterr_non_fatal_err_reporting_en ( ctx.cfg_aer_rooterr_non_fatal_err_reporting_en    ),  // ->
        .cfg_root_control_syserr_corr_err_en        ( ctx.cfg_root_control_syserr_corr_err_en           ),  // ->
        .cfg_root_control_syserr_fatal_err_en       ( ctx.cfg_root_control_syserr_fatal_err_en          ),  // ->
        .cfg_root_control_syserr_non_fatal_err_en   ( ctx.cfg_root_control_syserr_non_fatal_err_en      ),  // ->
        .cfg_slot_control_electromech_il_ctl_pulse  ( ctx.cfg_slot_control_electromech_il_ctl_pulse     ),  // ->
        
        // PCIe core PHY
        .pl_initial_link_width      ( ctx.pl_initial_link_width         ),  // -> [2:0]
        .pl_phy_lnk_up              ( ctx.pl_phy_lnk_up                 ),  // ->
        .pl_lane_reversal_mode      ( ctx.pl_lane_reversal_mode         ),  // -> [1:0]
        .pl_link_gen2_cap           ( ctx.pl_link_gen2_cap              ),  // ->
        .pl_link_partner_gen2_supported ( ctx.pl_link_partner_gen2_supported ),  // ->
        .pl_link_upcfg_cap          ( ctx.pl_link_upcfg_cap             ),  // ->
        .pl_sel_lnk_rate            ( ctx.pl_sel_lnk_rate               ),  // ->
        .pl_sel_lnk_width           ( ctx.pl_sel_lnk_width              ),  // -> [1:0]
        .pl_ltssm_state             ( ctx.pl_ltssm_state                ),  // -> [5:0]
        .pl_rx_pm_state             ( ctx.pl_rx_pm_state                ),  // -> [1:0]
        .pl_tx_pm_state             ( ctx.pl_tx_pm_state                ),  // -> [2:0]
        .pl_directed_change_done    ( ctx.pl_directed_change_done       ),  // ->
        .pl_received_hot_rst        ( ctx.pl_received_hot_rst           ),  // ->
        .pl_directed_link_auton     ( ctx.pl_directed_link_auton        ),  // <-
        .pl_directed_link_change    ( ctx.pl_directed_link_change       ),  // <- [1:0]
        .pl_directed_link_speed     ( ctx.pl_directed_link_speed        ),  // <-
        .pl_directed_link_width     ( ctx.pl_directed_link_width        ),  // <- [1:0]
        .pl_upstream_prefer_deemph  ( ctx.pl_upstream_prefer_deemph     ),  // <-
        .pl_transmit_hot_rst        ( ctx.pl_transmit_hot_rst           ),  // <-
        .pl_downstream_deemph_source( ctx.pl_downstream_deemph_source   ),  // <-
        
        // DRP - clock domain clk_100 - write should only happen when core is in reset state ...
        .pcie_drp_clk               ( clk_sys                           ),  // <-
        .pcie_drp_en                ( dfifo_pcie.drp_en                 ),  // <-
        .pcie_drp_we                ( dfifo_pcie.drp_we                 ),  // <-
        .pcie_drp_addr              ( dfifo_pcie.drp_addr               ),  // <- [8:0]
        .pcie_drp_di                ( dfifo_pcie.drp_di                 ),  // <- [15:0]
        .pcie_drp_rdy               ( dfifo_pcie.drp_rdy                ),  // ->
        .pcie_drp_do                ( dfifo_pcie.drp_do                 ),  // -> [15:0]
    
        // user interface
        .user_clk_out               ( clk_pcie                          ),  // ->
        .user_reset_out             ( rst_pcie_user                     ),  // ->
        .user_lnk_up                ( user_lnk_up                       ),  // ->
        .user_app_rdy               (                                   )   // ->
    );

endmodule


// ------------------------------------------------------------------------
// PMCSR D0 滤波。
// 离 D0 需连续 LEAVE_STABLE 拍非 00；回 D0 需连续 ENTER_STABLE 拍 00。
// 复位按 PCI 约定视为 D0。默认 64 拍 @62.5 MHz ≈ 1.0 µs，吞毛刺但远短于
// 主机真实 D3 驻留。
// ------------------------------------------------------------------------
module pcileech_pmcsr_d0_filter #(
    parameter [7:0] LEAVE_STABLE = 8'd64,
    parameter [7:0] ENTER_STABLE = 8'd4
)(
    input                   clk,
    input                   rst,
    input  [1:0]            pmcsr_powerstate,
    output bit              power_state_d0
);
    bit [7:0] stable_cnt;
    wire      is_d0 = (pmcsr_powerstate == 2'b00);

    always @ ( posedge clk ) begin
        if ( rst ) begin
            power_state_d0 <= 1'b1;
            stable_cnt     <= 8'h00;
        end
        else if ( power_state_d0 ) begin
            if ( is_d0 )
                stable_cnt <= 8'h00;
            else if ( stable_cnt == (LEAVE_STABLE - 8'd1) ) begin
                power_state_d0 <= 1'b0;
                stable_cnt     <= 8'h00;
            end
            else
                stable_cnt <= stable_cnt + 8'd1;
        end
        else begin
            if ( !is_d0 )
                stable_cnt <= 8'h00;
            else if ( stable_cnt == (ENTER_STABLE - 8'd1) ) begin
                power_state_d0 <= 1'b1;
                stable_cnt     <= 8'h00;
            end
            else
                stable_cnt <= stable_cnt + 8'd1;
        end
    end
endmodule


// ------------------------------------------------------------------------
// FLR / 热复位去抖。
// 只认硬核 cfg_received_func_lvl_rst 与 pl_received_hot_rst，不认 BME/PM。
// FLR 需连续 STABLE 拍且 LTSSM=L0；热复位需连续 STABLE 拍且 LTSSM=Hot Reset
// （PG054：L0=0x16，Hot Reset=0x21）。合格后发单拍脉冲，源保持高不重发。
// 默认 16 拍 @62.5 MHz ≈ 256 ns，吞初始化毛刺，远短于真实 FLR/热复位驻留。
// ------------------------------------------------------------------------
module pcileech_lifecycle_reset_filter #(
    parameter [7:0] STABLE_CYCLES = 8'd16
)(
    input                   clk,
    input                   rst,
    input                   flr,
    input                   hot_rst,
    input  [5:0]            ltssm_state,
    output bit              lifecycle_reset_req
);
    localparam [5:0] LTSSM_L0        = 6'h16;
    localparam [5:0] LTSSM_HOT_RESET = 6'h21;

    bit [7:0] stable_cnt;
    bit       fired;
    wire      flr_qual  = flr && (ltssm_state == LTSSM_L0);
    wire      hot_qual  = hot_rst && (ltssm_state == LTSSM_HOT_RESET);
    wire      event_qual = flr_qual || hot_qual;

    always @ ( posedge clk ) begin
        if ( rst ) begin
            lifecycle_reset_req <= 1'b0;
            stable_cnt          <= 8'h00;
            fired               <= 1'b0;
        end
        else begin
            lifecycle_reset_req <= 1'b0;
            if ( !event_qual ) begin
                stable_cnt <= 8'h00;
                fired      <= 1'b0;
            end
            else if ( !fired ) begin
                if ( stable_cnt == (STABLE_CYCLES - 8'd1) ) begin
                    lifecycle_reset_req <= 1'b1;
                    fired               <= 1'b1;
                    stable_cnt          <= 8'h00;
                end
                else
                    stable_cnt <= stable_cnt + 8'd1;
            end
        end
    end
endmodule


// ------------------------------------------------------------------------
// TLP STREAM SINK:
// Convert a 128-bit TLP-AXI-STREAM to a 64-bit PCIe core AXI-STREAM.
// ------------------------------------------------------------------------
module pcileech_tlps128_dst64(
    input                   rst,
    input                   clk_pcie,
    IfPCIeTlpRxTx.source    tlp_tx,
    IfAXIS128.sink          tlps_in,
    output wire             idle,
    output wire             packet_done,
    output wire [2:0]       packet_src
);
    // 整拍缓存的标准 128→64 转换：缓冲一个 128 位拍，分低/高 64 位两拍发出，
    // 各拍保持到 tlp_tx.ready。电平式 tready(不依赖 tvalid)，既能接持续拉高
    // tvalid 的多拍源(完成通路)，也不会死锁。
    bit [127:0] buf_data;
    bit [3:0]   buf_keepdw;
    bit         buf_last;
    bit [2:0]   buf_src;
    bit         buf_valid  = 0;
    bit         emit_upper = 0;

    wire has_upper     = buf_keepdw[2];
    wire emitting_last = buf_valid && (emit_upper || !has_upper);
    wire final_accept  = emitting_last && tlp_tx.ready;
    assign idle = !buf_valid && !tlps_in.tvalid && !tlps_in.has_data;
    assign tlps_in.tready = !rst && (!buf_valid || final_accept);

    assign tlp_tx.valid = buf_valid;
    assign tlp_tx.data  =
        (buf_data[127:64] & {64{emit_upper}}) |
        (buf_data[63:0] & {64{!emit_upper}});
    assign tlp_tx.keep  = emit_upper ? (buf_keepdw[3] ? 8'hff : 8'h0f)
                                     : (buf_keepdw[1] ? 8'hff : 8'h0f);
    assign tlp_tx.last  = buf_last && (emit_upper || !has_upper);
    assign tlp_tx.user  = 22'h0;
    assign packet_done  = final_accept;
    assign packet_src   = buf_src;

    always @ ( posedge clk_pcie ) begin
        if (rst) begin
            buf_valid  <= 1'b0;
            emit_upper <= 1'b0;
            buf_src    <= 3'd0;
        end
        else begin
            if (tlps_in.tvalid && (!buf_valid || final_accept)) begin
                buf_data   <= tlps_in.tdata;
                buf_keepdw <= tlps_in.tkeepdw;
                buf_last   <= tlps_in.tlast;
                buf_src    <= tlps_in.tuser[8:6];
                buf_valid  <= 1'b1;
                emit_upper <= 1'b0;
            end
            else if (buf_valid && tlp_tx.ready) begin
                if (!emit_upper && has_upper)
                    emit_upper <= 1'b1;
                else
                    buf_valid <= 1'b0;
            end
        end
    end

endmodule


// ------------------------------------------------------------------------
// TLP STREAM SOURCE:
// Convert a 64-bit PCIe core AXIS to a 128-bit TLP-AXI-STREAM 
// ------------------------------------------------------------------------
module pcileech_tlps128_src64(
    input                   rst,
    input                   clk_pcie,
    IfPCIeTlpRxTx.sink      tlp_rx,
    IfAXIS128.source_lite   tlps_out
);

    bit [127:0] tdata;
    bit         first       = 1;
    bit         tlast       = 0;
    bit [3:0]   len         = 0;
    bit [6:0]   bar_hit     = 0;
    wire        tvalid      = tlast || (len>2);
    
    assign tlp_rx.ready     = 1'b1;
    assign tlps_out.tdata   = tdata;
    assign tlps_out.tkeepdw = {(len>3), (len>2), (len>1), 1'b1};
    assign tlps_out.tlast   = tlast;   
    assign tlps_out.tvalid  = tvalid; 
    assign tlps_out.tuser[0]    = first;
    assign tlps_out.tuser[1]    = tlast;
    assign tlps_out.tuser[8:2]  = bar_hit;
    
    wire [3:0]  next_base   = (tlast || tvalid) ? 0 : len;
    wire [3:0]  next_len    = next_base + 1 + tlp_rx.keep[4];

    always @ ( posedge clk_pcie )
        if ( rst ) begin
            first   <= 1;
            tlast   <= 0;
            len     <= 0;
            bar_hit <= 0;
        end
        // rx_err_fwd(tuser[1])在整个毒化包期间由硬核拉高：整包丢弃不组帧，
        // 避免毒化数据进入 BAR/配置/完成包通路。（PG054：AXI 接口无截断
        // 指示位；链路中断时硬核会按 Length 补齐并正常给 tlast，不会产生
        // 半包，因此无需额外的半包冲刷逻辑。）
        else if ( tlp_rx.valid && !tlp_rx.user[1] ) begin
            tdata[(32*next_base)+:64] <= tlp_rx.data;
            first   <= tvalid ? tlast : first;
            tlast   <= tlp_rx.last;
            len     <= next_len;
            bar_hit <= tlp_rx.user[8:2];
        end
        else if ( tvalid ) begin 
            first   <= tlast;
            tlast   <= 0;
            len     <= 0;
            bar_hit <= 0;
        end
    
endmodule
