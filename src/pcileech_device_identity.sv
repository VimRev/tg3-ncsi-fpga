`timescale 1ns / 1ps

// 每卡身份：7 系列 Device DNA（DNA_PORT，57 位）作种子。
// Artix-7 没有 DNA_PORT2（那是 UltraScale）；全片只有一个 DNA_PORT，
// 由本模块独占。fifo 的 dna_check 在 dna_enable=0 时不再例化该原语。
//
// MAC：Broadcom OUI 池 4 选 1 + DNA 派生低 24 位（单播、全球管理位已由 OUI 保证）。
// 打包与 tg3 / T05 一致：0x410/0x7C = {16'h0, mac0, mac1}，0x414/0x80 = {mac2..mac5}。
// 子网 192.168.x.0/24，x 避开 0/255。网关 MAC：家用路由器 OUI 池 + 00:00 + DNA 低字节。
module pcileech_device_identity #(
    parameter bit        USE_SIM_DNA   = 1'b0,
    parameter [56:0]     SIM_DNA_VALUE = 57'h1555AAA33331111
)(
    input               clk,
    input               rst,
    output              identity_valid,
    output [56:0]       dna_value,
    output [47:0]       nic_mac,
    output [31:0]       mac_word_7c,
    output [31:0]       mac_word_80,
    output [7:0]        subnet_octet,
    output [31:0]       gw_ipv4,
    output [31:0]       lease_ipv4,
    output [47:0]       gw_mac
);

    localparam [23:0] OUI0 = 24'h001018;
    localparam [23:0] OUI1 = 24'h000AF7;
    localparam [23:0] OUI2 = 24'h0014A4;
    localparam [23:0] OUI3 = 24'h001BE9;
    // 家用路由器 OUI：TP-Link / Netgear / ASUS / D-Link（单播、全球管理位）。
    localparam [23:0] GW_OUI0 = 24'h50C7BF;
    localparam [23:0] GW_OUI1 = 24'h20E52A;
    localparam [23:0] GW_OUI2 = 24'h2C56DC;
    localparam [23:0] GW_OUI3 = 24'h14D64D;

    bit         ident_valid_r;
    bit         ident_valid_d;
    bit [56:0]  dna_latched;
    bit         dna_read;
    bit         dna_shift;
    bit [6:0]   dna_bit_cnt;
    wire        dna_dout;
    bit [47:0]  nic_mac_r;
    bit [7:0]   subnet_r;
    bit [47:0]  gw_mac_r;

    assign identity_valid = ident_valid_d;
    assign dna_value      = dna_latched;

    function automatic [47:0] derive_nic_mac(input [56:0] dna);
        reg [1:0]  oui_sel;
        reg [23:0] oui;
        reg [23:0] nic;
        begin
            oui_sel = dna[1:0] ^ dna[9:8];
            case (oui_sel)
                2'd0: oui = OUI0;
                2'd1: oui = OUI1;
                2'd2: oui = OUI2;
                default: oui = OUI3;
            endcase
            nic = dna[32:9] ^ dna[56:33];
            if (nic == 24'h0)
                nic = 24'h000001;
            derive_nic_mac = {oui, nic};
        end
    endfunction

    function automatic [7:0] derive_subnet(input [56:0] dna);
        reg [7:0] x;
        begin
            x = dna[15:8] ^ dna[23:16] ^ dna[47:40];
            if ((x == 8'h00) || (x == 8'hFF))
                x = 8'd77;
            derive_subnet = x;
        end
    endfunction

    function automatic [7:0] derive_gw_mac_low(input [56:0] dna);
        begin
            derive_gw_mac_low = (dna[7:0] == 8'h00) ? 8'h01 : dna[7:0];
        end
    endfunction

    function automatic [23:0] derive_gw_oui(input [56:0] dna);
        reg [1:0] oui_sel;
        begin
            oui_sel = dna[3:2] ^ dna[11:10];
            case (oui_sel)
                2'd0: derive_gw_oui = GW_OUI0;
                2'd1: derive_gw_oui = GW_OUI1;
                2'd2: derive_gw_oui = GW_OUI2;
                default: derive_gw_oui = GW_OUI3;
            endcase
        end
    endfunction

    wire [47:0] nic_mac_c = derive_nic_mac(dna_latched);
    wire [7:0]  subnet_c  = derive_subnet(dna_latched);
    wire [7:0]  gw_low_c  = derive_gw_mac_low(dna_latched);
    wire [23:0] gw_oui_c  = derive_gw_oui(dna_latched);

    // 锁存派生结果，切断 DNA→reply_byte 的单周期组合云。
    always @(posedge clk) begin
        if (rst) begin
            ident_valid_d <= 1'b0;
            nic_mac_r     <= 48'h0;
            subnet_r      <= 8'h0;
            gw_mac_r      <= 48'h0;
        end
        else begin
            ident_valid_d <= ident_valid_r;
            nic_mac_r     <= nic_mac_c;
            subnet_r      <= subnet_c;
            gw_mac_r      <= {gw_oui_c, 16'h0000, gw_low_c};
        end
    end

    assign nic_mac      = nic_mac_r;
    assign mac_word_7c  = {16'h0000, nic_mac_r[47:32]};
    assign mac_word_80  = nic_mac_r[31:0];
    assign subnet_octet = subnet_r;
    assign gw_ipv4      = {16'hC0A8, subnet_r, 8'h01};
    assign lease_ipv4   = {16'hC0A8, subnet_r, 8'h02};
    assign gw_mac       = gw_mac_r;

    generate
        if (USE_SIM_DNA) begin : gen_sim_dna
            assign dna_dout = 1'b0;
            always @(posedge clk) begin
                if (rst) begin
                    ident_valid_r <= 1'b0;
                    dna_latched   <= 57'h0;
                    dna_read      <= 1'b0;
                    dna_shift     <= 1'b0;
                    dna_bit_cnt   <= 7'h0;
                end
                else begin
                    dna_latched   <= SIM_DNA_VALUE;
                    ident_valid_r <= 1'b1;
                end
            end
        end
        else begin : gen_hw_dna
            DNA_PORT #(
                .SIM_DNA_VALUE ( SIM_DNA_VALUE )
            ) u_dna (
                .DOUT  ( dna_dout ),
                .CLK   ( clk ),
                .DIN   ( 1'b0 ),
                .READ  ( dna_read ),
                .SHIFT ( dna_shift )
            );

            always @(posedge clk) begin
                if (rst) begin
                    ident_valid_r <= 1'b0;
                    dna_latched   <= 57'h0;
                    dna_read      <= 1'b1;
                    dna_shift     <= 1'b0;
                    dna_bit_cnt   <= 7'h0;
                end
                else if (!ident_valid_r) begin
                    if (dna_read) begin
                        dna_read    <= 1'b0;
                        dna_shift   <= 1'b1;
                        dna_latched <= {56'h0, dna_dout};
                        dna_bit_cnt <= 7'd1;
                    end
                    else if (dna_bit_cnt < 7'd57) begin
                        dna_shift   <= 1'b1;
                        dna_latched <= {dna_latched[55:0], dna_dout};
                        dna_bit_cnt <= dna_bit_cnt + 1'b1;
                    end
                    else begin
                        dna_shift     <= 1'b0;
                        ident_valid_r <= 1'b1;
                    end
                end
                else begin
                    dna_read  <= 1'b0;
                    dna_shift <= 1'b0;
                end
            end
        end
    endgenerate

endmodule
