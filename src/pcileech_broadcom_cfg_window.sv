//
// Broadcom 配置空间动态窗口。
//
// cfg_*_dwaddr 使用 PCIe 配置 TLP 中的 10 位 DWORD 地址：
// 例如字节偏移 0x68 对应 DWORD 地址 0x1A。
//

`timescale 1ns / 1ps

module pcileech_broadcom_cfg_window(
    input               clk,
    input               rst,

    input               cfg_wr_valid,
    input       [9:0]   cfg_wr_dwaddr,
    input       [3:0]   cfg_wr_be,
    input       [31:0]  cfg_wr_data,

    input               cfg_rd_valid,
    input       [9:0]   cfg_rd_dwaddr,

    input       [31:0]  broadcom_cfg_68_value,
    input       [31:0]  broadcom_cfg_6c_value,
    input       [31:0]  broadcom_cfg_70_value,

    output reg          special_valid,
    output reg  [31:0]  special_data
);

    localparam [9:0] CFG_DW_68 = 10'h01A;
    localparam [9:0] CFG_DW_6C = 10'h01B;
    localparam [9:0] CFG_DW_70 = 10'h01C;
    localparam [9:0] CFG_DW_7C = 10'h01F;
    localparam [9:0] CFG_DW_84 = 10'h021;

    reg [31:0] sram_byte_address;

    // 4 KiB 内部 SRAM；同步读和分字节写有利于推断单端口块 RAM。
    (* ram_style = "block" *) reg [31:0] internal_sram [0:1023];
    integer init_index;

    initial begin
        for (init_index = 0; init_index < 1024; init_index = init_index + 1)
            internal_sram[init_index] = 32'h00000000;

        // 实卡初始化阶段通过配置空间窗口读取的芯片身份数据。
        internal_sram[12'hB54 >> 2] = 32'h4B657654;
        internal_sram[12'hB58 >> 2] = 32'h00010015;
        internal_sram[12'hB5C >> 2] = 32'h00040000;
        internal_sram[12'hB74 >> 2] = 32'h00206180;
        internal_sram[12'hC14 >> 2] = 32'h484B1C86;
        internal_sram[12'hD34 >> 2] = 32'h01800002;
        internal_sram[12'hD38 >> 2] = 32'h00000015;
        internal_sram[12'hD3C >> 2] = 32'h00000030;
    end

    always @(posedge clk) begin
        if (rst) begin
            sram_byte_address <= 32'h00000000;
            special_valid    <= 1'b0;
            special_data     <= 32'h00000000;
        end
        else begin
            special_valid <= 1'b0;

            // 0x7C 保留配置写入结果，并支持 PCIe 字节使能。
            if (cfg_wr_valid && (cfg_wr_dwaddr == CFG_DW_7C)) begin
                if (cfg_wr_be[0]) sram_byte_address[7:0]   <= cfg_wr_data[7:0];
                if (cfg_wr_be[1]) sram_byte_address[15:8]  <= cfg_wr_data[15:8];
                if (cfg_wr_be[2]) sram_byte_address[23:16] <= cfg_wr_data[23:16];
                if (cfg_wr_be[3]) sram_byte_address[31:24] <= cfg_wr_data[31:24];
            end

            // SRAM 端口采用写优先仲裁；正常配置 TLP 流不会同拍读写 0x84。
            if (cfg_wr_valid && (cfg_wr_dwaddr == CFG_DW_84)) begin
                if (cfg_wr_be[0])
                    internal_sram[sram_byte_address[11:2]][7:0]
                        <= cfg_wr_data[7:0];
                if (cfg_wr_be[1])
                    internal_sram[sram_byte_address[11:2]][15:8]
                        <= cfg_wr_data[15:8];
                if (cfg_wr_be[2])
                    internal_sram[sram_byte_address[11:2]][23:16]
                        <= cfg_wr_data[23:16];
                if (cfg_wr_be[3])
                    internal_sram[sram_byte_address[11:2]][31:24]
                        <= cfg_wr_data[31:24];
            end
            else if (cfg_rd_valid && (cfg_rd_dwaddr == CFG_DW_84)) begin
                special_valid <= 1'b1;
                special_data  <= internal_sram[sram_byte_address[11:2]];
            end

            // 非 SRAM 特殊寄存器在读请求后一拍给出响应。
            if (cfg_rd_valid) begin
                case (cfg_rd_dwaddr)
                    CFG_DW_68: begin
                        special_valid <= 1'b1;
                        special_data  <= broadcom_cfg_68_value;
                    end
                    CFG_DW_6C: begin
                        special_valid <= 1'b1;
                        special_data  <= broadcom_cfg_6c_value;
                    end
                    CFG_DW_70: begin
                        special_valid <= 1'b1;
                        special_data  <= broadcom_cfg_70_value;
                    end
                    CFG_DW_7C: begin
                        special_valid <= 1'b1;
                        special_data  <= sram_byte_address;
                    end
                    CFG_DW_84: begin
                        // SRAM 数据由上面的同步存储器端口返回。
                    end
                    default: begin
                        special_valid <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule
