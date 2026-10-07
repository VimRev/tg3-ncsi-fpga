//
// PCILeech FPGA.
//
// PCIe custom shadow configuration space.
// Xilinx PCIe core will take configuration space priority; if Xilinx PCIe core
// is configured to forward configuration requests to user application such TLP
// will end up being processed by this module.
//
// (c) Ulf Frisk, 2021-2024
// Author: Ulf Frisk, pcileech@frizk.net
//

`timescale 1ns / 1ps

module pcileech_tlps128_cfgspace_shadow(
    input                   rst,
    input                   clk_pcie,
    input                   clk_sys,
    IfAXIS128.sink_lite     tlps_in,
    input [15:0]            pcie_id,
    input [31:0]            broadcom_cfg_68_value,
    input [31:0]            broadcom_cfg_6c_value,
    input [31:0]            broadcom_cfg_70_value,
    IfAXIS128.source        tlps_cfg_rsp,
    IfShadow2Fifo.shadow    dshadow2fifo
);
    // ----------------------------------------------------------------------------
    // PCIe RECEIVE:
    // ----------------------------------------------------------------------------
    wire                pcie_rx_rden    = tlps_in.tvalid && tlps_in.tuser[0] && (tlps_in.tdata[31:25] == 7'b0000010);   // CfgRd: Fmt[2:0]=000b (3 DW header, no data), CfgRd0/CfgRd1=0010xb
    wire                pcie_rx_wren    = tlps_in.tvalid && tlps_in.tuser[0] && (tlps_in.tdata[31:25] == 7'b0100010);   // CfgWr: Fmt[2:0]=010b (3 DW header, data),    CfgWr0/CfgWr1=0010xb
    wire [9:0]          pcie_rx_addr    = tlps_in.tdata[75:66];
    wire [31:0]         pcie_rx_data_raw = tlps_in.tdata[127:96];
    // PCIe TLP 数据在 AXI DWORD 内按字节反序，内部配置状态统一保存为主机字节序。
    wire [31:0]         pcie_rx_data    = {
                            pcie_rx_data_raw[7:0],
                            pcie_rx_data_raw[15:8],
                            pcie_rx_data_raw[23:16],
                            pcie_rx_data_raw[31:24]
                        };
    wire [7:0]          pcie_rx_tag     = tlps_in.tdata[47:40];
    wire [3:0]          pcie_rx_be      = tlps_in.tdata[35:32];
    wire [15:0]         pcie_rx_reqid   = tlps_in.tdata[63:48];

    // ----------------------------------------------------------------------------
    // USB RECEIVE (clock domain crossing):
    // ----------------------------------------------------------------------------
    wire                usb_rx_rden_out;
    wire                usb_rx_wren_out;
    wire                usb_rx_valid;
    wire                usb_rx_rden = usb_rx_valid && usb_rx_rden_out;
    wire                usb_rx_wren = usb_rx_valid && usb_rx_wren_out;
    wire    [3:0]       usb_rx_be;
    wire    [31:0]      usb_rx_data;
    wire    [9:0]       usb_rx_addr;
    wire                usb_rx_addr_lo;
    fifo_49_49_clk2 i_fifo_49_49_clk2(
        .rst            ( rst                       ),
        .wr_clk         ( clk_sys                   ),
        .rd_clk         ( clk_pcie                  ),
        .wr_en          ( dshadow2fifo.rx_rden || dshadow2fifo.rx_wren ),
        .din            ( {dshadow2fifo.rx_rden, dshadow2fifo.rx_wren, dshadow2fifo.rx_addr_lo, dshadow2fifo.rx_addr, dshadow2fifo.rx_be, dshadow2fifo.rx_data} ),
        .full           (                           ),
        .rd_en          ( 1'b1                      ),
        .dout           ( {usb_rx_rden_out, usb_rx_wren_out, usb_rx_addr_lo, usb_rx_addr, usb_rx_be, usb_rx_data} ),    
        .empty          (                           ),
        .valid          ( usb_rx_valid              )
    );

    // ----------------------------------------------------------------------------
    // WRITE multiplexor: simple naive multiplexor which will prioritize in order:
    // (1) PCIe (if enabled), (2) USB, (3) INTERNAL.
    // Collisions will be discarded (it's assumed that they'll be very rare)
    // ----------------------------------------------------------------------------
    // cfgtlp_en/cfgtlp_wren/cfgtlp_zero 来自 clk_sys 域的准静态配置，
    // 进入 clk_pcie 域组合逻辑前打两拍，消除多位异步翻转被采出
    // 不一致组合的窗口。
    (* ASYNC_REG = "TRUE" *) bit [2:0] cfgbits_sync1 = 0;
    (* ASYNC_REG = "TRUE" *) bit [2:0] cfgbits_sync2 = 0;
    always @ ( posedge clk_pcie ) begin
        cfgbits_sync1 <= {dshadow2fifo.cfgtlp_en, dshadow2fifo.cfgtlp_wren,
                          dshadow2fifo.cfgtlp_zero};
        cfgbits_sync2 <= cfgbits_sync1;
    end
    wire            cfgtlp_en_sync   = cfgbits_sync2[2];
    wire            cfgtlp_wren_sync = cfgbits_sync2[1];
    wire            cfgtlp_zero_sync = cfgbits_sync2[0];

    wire            bram_wr_1_tlp = pcie_rx_wren & cfgtlp_en_sync;
    wire            bram_wr_2_usb = ~bram_wr_1_tlp & usb_rx_wren;
    wire [3:0]      bram_wr_be = bram_wr_1_tlp ? (cfgtlp_wren_sync ? pcie_rx_be : 4'b0000) : (bram_wr_2_usb ? usb_rx_be : 4'b0000);
    wire [31:0]     bram_wr_data = bram_wr_1_tlp ? pcie_rx_data : (bram_wr_2_usb ? usb_rx_data : 32'h00000000);

    // ----------------------------------------------------------------------------
    // WRITE multiplexor and state machine: simple naive multiplexor which will prioritize in order:
    // (1) PCIe (if enabled), (2) USB, (3) INTERNAL.
    // Collisions will be discarded (it's assumed that they'll be very rare)
    // ----------------------------------------------------------------------------
    `define S_SHADOW_CFGSPACE_IDLE  2'b00
    `define S_SHADOW_CFGSPACE_TLP   2'b01
    `define S_SHADOW_CFGSPACE_USB   2'b10
    
    wire [15:0]     bram_rd_reqid;
    wire [1:0]      bram_rd_tp;
    wire [7:0]      bram_rd_tag;
    wire [9:0]      bram_rd_addr;
    wire [31:0]     bram_rd_data;
    wire            broadcom_special_valid;
    wire [31:0]     broadcom_special_data;
    wire [31:0]     bram_rd_data_z  = cfgtlp_zero_sync ?
                                      32'h00000000 :
                                      (broadcom_special_valid ?
                                       broadcom_special_data : bram_rd_data);
    wire            bram_rd_valid   = (bram_rd_tp == `S_SHADOW_CFGSPACE_TLP);
    wire            bram_rd_tlpwr;
    
    wire            bram_rd_1_tlp   = pcie_rx_rden & cfgtlp_en_sync;
    wire            bram_tlp        = bram_rd_1_tlp | bram_wr_1_tlp;
    wire            bram_rd_2_usb   = ~bram_tlp & usb_rx_rden;
    wire [1:0]      bram_rdreq_tp   = bram_tlp ? `S_SHADOW_CFGSPACE_TLP : (bram_rd_2_usb ? `S_SHADOW_CFGSPACE_USB : `S_SHADOW_CFGSPACE_IDLE);
    wire [9:0]      bram_rdreq_addr = bram_tlp ? pcie_rx_addr : usb_rx_addr;
    wire [7:0]      bram_rdreq_tag  = bram_tlp ? pcie_rx_tag : {7'h00, usb_rx_addr_lo};
    wire [15:0]     bram_rdreq_reqid= bram_tlp ? pcie_rx_reqid : 16'h0000;

    // Broadcom 的 0x68–0x84 区域包含 BAR 镜像和间接 SRAM 窗口，
    // 不能只依赖静态配置空间 BRAM。
    pcileech_broadcom_cfg_window i_pcileech_broadcom_cfg_window(
        .clk                    ( clk_pcie                  ),
        .rst                    ( rst                       ),
        .cfg_wr_valid           ( |bram_wr_be               ),
        .cfg_wr_dwaddr          ( bram_rdreq_addr           ),
        .cfg_wr_be              ( bram_wr_be                ),
        .cfg_wr_data            ( bram_wr_data              ),
        .cfg_rd_valid           ( bram_rd_1_tlp | bram_rd_2_usb ),
        .cfg_rd_dwaddr          ( bram_rdreq_addr           ),
        .broadcom_cfg_68_value  ( broadcom_cfg_68_value     ),
        .broadcom_cfg_6c_value  ( broadcom_cfg_6c_value     ),
        .broadcom_cfg_70_value  ( broadcom_cfg_70_value     ),
        .special_valid          ( broadcom_special_valid    ),
        .special_data           ( broadcom_special_data     )
    );
    
    // BRAM MEMORY ACCESS for the 4kB / 0x1000 byte shadow configuration space.    
    pcileech_mem_wrap i_pcileech_mem_wrap(
        .clk_pcie       ( clk_pcie                 ), // <-
        .rdwr_addr      ( bram_rdreq_addr          ), // <-
        .wr_be          ( bram_wr_be               ), // <-
        .wr_data        ( bram_wr_data             ), // <-
        .wr_from_host   ( bram_wr_1_tlp            ), // <-
        .rdreq_tag      ( bram_rdreq_tag           ), // <-
        .rdreq_tp       ( bram_rdreq_tp            ), // <-
        .rdreq_reqid    ( bram_rdreq_reqid         ), // <-
        .rdreq_tlpwr    ( bram_wr_1_tlp            ), // <-
        .rd_data        ( bram_rd_data             ), // ->
        .rd_addr        ( bram_rd_addr             ), // ->
        .rd_tag         ( bram_rd_tag              ), // ->
        .rd_tp          ( bram_rd_tp               ), // ->
        .rd_reqid       ( bram_rd_reqid            ), // ->
        .rd_tlpwr       ( bram_rd_tlpwr            )  // ->
    );
    
    // PCIe REPLY:
    pcileech_cfgspace_pcie_tx i_pcileech_cfgspace_pcie_tx(
        .rst            ( rst                       ),  // <-
        .clk_pcie       ( clk_pcie                  ),  // <-
        .pcie_id        ( pcie_id                   ),  // <- [15:0]
        .tlps_cfg_rsp   ( tlps_cfg_rsp              ),
        // cfgspace:
        .cfg_wren       ( bram_rd_valid             ),  // <-
        .cfg_tlpwr      ( bram_rd_tlpwr             ),  // <-
        .cfg_tag        ( bram_rd_tag               ),  // <- [7:0]
        .cfg_data       ( bram_rd_data_z            ),  // <- [32:0]
        .cfg_reqid      ( bram_rd_reqid             )   // <- [15:0]
    );
    
    // USB REPLY:
    fifo_43_43_clk2 i_fifo_43_43_clk2(
        .rst            ( rst                       ),
        .wr_clk         ( clk_pcie                  ),
        .rd_clk         ( clk_sys                   ),
        .wr_en          ( (bram_rd_tp == `S_SHADOW_CFGSPACE_USB) ),
        .din            ( {bram_rd_tag[0], bram_rd_addr, bram_rd_data_z} ),
        .full           (                           ),
        .rd_en          ( 1'b1                      ),
        .dout           ( {dshadow2fifo.tx_addr_lo, dshadow2fifo.tx_addr, dshadow2fifo.tx_data} ),    
        .empty          (                           ),
        .valid          ( dshadow2fifo.tx_valid     )
    );
    
endmodule



// PCIe TLP cfg reply module:
module pcileech_cfgspace_pcie_tx(
    input                   rst,
    input                   clk_pcie,
    input   [15:0]          pcie_id,        // PCIe id of this core
    IfAXIS128.source        tlps_cfg_rsp,
    // cfgspace:
    input                   cfg_wren,
    input                   cfg_tlpwr,
    input [7:0]             cfg_tag,
    input [31:0]            cfg_data,
    input [15:0]            cfg_reqid
    );
    
    wire [31:0]     cpl_tlp_data_dw0_rd  = 32'b01001010000000000000000000000001;
    wire [31:0]     cpl_tlp_data_dw0_wr  = 32'b00001010000000000000000000000000;
    wire [31:0]     cpl_tlp_data_dw1     = {
                            pcie_id[7:0], pcie_id[15:8], 16'h0004
                        };
    wire [31:0]     cpl_tlp_data_dw2     = { cfg_reqid, cfg_tag, 8'h00 };
    // 完成包的数据 DWORD 必须转换回 PCIe TLP 的字节排列。
    wire [31:0]     cpl_tlp_data_dw3     = {
                            cfg_data[7:0],
                            cfg_data[15:8],
                            cfg_data[23:16],
                            cfg_data[31:24]
                        };
    wire [127:0]    cpl_tlp_rd           = { cpl_tlp_data_dw3, cpl_tlp_data_dw2, cpl_tlp_data_dw1, cpl_tlp_data_dw0_rd };
    wire [127:0]    cpl_tlp_wr           = { 32'h00000000,     cpl_tlp_data_dw2, cpl_tlp_data_dw1, cpl_tlp_data_dw0_wr };
    wire [128:0]    cpl_tlps             = cfg_tlpwr ? {1'b0, cpl_tlp_wr} : {1'b1, cpl_tlp_rd};

    wire tx_tp;
    wire tx_empty;
    fifo_129_129_clk1 i_fifo_129_129_clk1 (
        .srst           ( rst                       ),
        .clk            ( clk_pcie                  ),
        // data in
        .wr_en          ( cfg_wren                  ),
        .din            ( cpl_tlps                  ),
        .full           (                           ),
        // data out
        .rd_en          ( tlps_cfg_rsp.tready       ),
        .dout           ( {tx_tp, tlps_cfg_rsp.tdata} ), 
        .empty          ( tx_empty                  ),
        .valid          ( tlps_cfg_rsp.tvalid       )
    );
    
    assign tlps_cfg_rsp.tkeepdw = (tx_tp ? 4'b1111 : 4'b0111);
    assign tlps_cfg_rsp.tlast = 1;
    assign tlps_cfg_rsp.tuser = 0;
    assign tlps_cfg_rsp.has_data = ~tx_empty;    
endmodule



// Wrapper module for the BRAM-backed configuration space.
module pcileech_mem_wrap(
    input               clk_pcie,
    
    // Address common to Read/Write:
    input   [9:0]       rdwr_addr,
    
    // Write to 'configuration/action space':
    input   [3:0]       wr_be,
    input   [31:0]      wr_data,
    // 1 = 主机 TLP 写（套 ROM/硬编码掩码）；0 = USB 写（全通）。
    input               wr_from_host,
       
    // Read from 'configuration space':
    input   [7:0]       rdreq_tag,
    input   [1:0]       rdreq_tp,
    input   [15:0]      rdreq_reqid,
    input               rdreq_tlpwr,
    
    output bit  [9:0]   rd_addr,
    output      [31:0]  rd_data,
    output bit  [7:0]   rd_tag,
    output bit  [1:0]   rd_tp,
    output bit  [15:0]  rd_reqid,
    output bit          rd_tlpwr
    );
    
    bit [3:0]  wr_be_d;
    bit [31:0] wr_data_d;
    bit        wr_from_host_d;
    
    wire [31:0] wr_mask_rom;
    wire [31:0] wr_mask;
    wire [31:0] wr_dina;
    
    wire [31:0] wr1c_mask_rom;
    wire [31:0] wr1c_mask;

    // DELAY TO FOLLOW BRAM DELAY
    always @ ( posedge clk_pcie )
        begin
            wr_be_d        <= wr_be;
            wr_data_d      <= wr_data;
            wr_from_host_d <= wr_from_host;
            rd_addr        <= rdwr_addr;
            rd_tag      <= rdreq_tag;
            rd_tp       <= rdreq_tp;
            rd_reqid    <= rdreq_reqid;
            rd_tlpwr    <= rdreq_tlpwr;
        end

    // BRAM: 'configuration space' - 4kB / 0x1000 bytes:
    bram_pcie_cfgspace i_bram_pcie_cfgspace(
        .clka           ( clk_pcie                  ),
        .clkb           ( clk_pcie                  ),
        .wea            ( wr_be_d                   ),
        .addra          ( rd_addr                   ),
        .dina           ( wr_dina                   ),
        .addrb          ( rdwr_addr                 ),
        .doutb          ( rd_data                   )
    );
    
    // DROM: 'configuration space' - 4kB / 0x1000 bytes write mask:
    drom_pcie_cfgspace_writemask i_drom_pcie_cfgspace_writemask(
        .a              ( rd_addr                   ),
        .spo            ( wr_mask_rom               )
    );    

    // DROM: 'configuration space' - 4kB / 0x1000 bytes RW1C mask - the write bit must be 1 :
    drom_pcie_cfgspace_rw1c i_drom_pcie_cfgspace_rw1c(
        .a              ( rd_addr                   ),
        .spo            ( wr1c_mask_rom             )
    );

    pcileech_cfgspace_write_filter i_pcileech_cfgspace_write_filter(
        .dwaddr          ( rd_addr                   ),
        .current_value   ( rd_data                   ),
        .write_value     ( wr_data_d                 ),
        .rom_write_mask  ( wr_mask_rom               ),
        .rom_rw1c_mask   ( wr1c_mask_rom             ),
        .apply_host_mask ( wr_from_host_d            ),
        .write_mask      ( wr_mask                   ),
        .rw1c_mask       ( wr1c_mask                 ),
        .next_value      ( wr_dina                   )
    );

endmodule

module pcileech_cfgspace_write_filter(
    input  [9:0]  dwaddr,
    input  [31:0] current_value,
    input  [31:0] write_value,
    input  [31:0] rom_write_mask,
    input  [31:0] rom_rw1c_mask,
    input         apply_host_mask,
    output [31:0] write_mask,
    output [31:0] rw1c_mask,
    output [31:0] next_value
);

    wire [31:0] host_write_mask =
        (dwaddr == 10'h034) ? 32'h00000000 :
        (dwaddr == 10'h035) ? 32'h00000000 :
        (dwaddr == 10'h036) ? 32'h000F7DFF :
        (dwaddr == 10'h037) ? 32'h00000000 :
        (dwaddr == 10'h038) ? 32'h000000C3 :
                              rom_write_mask;

    wire [31:0] host_rw1c_mask =
        (dwaddr == 10'h036) ? 32'h000F0000 :
                              rom_rw1c_mask;

    assign write_mask = apply_host_mask ? host_write_mask : 32'hFFFFFFFF;
    assign rw1c_mask  = apply_host_mask ? host_rw1c_mask  : 32'h00000000;

    assign next_value =
        (current_value & ~write_mask) |
        (write_value & write_mask & ~rw1c_mask) |
        (current_value & ~write_value & write_mask & rw1c_mask);

endmodule
