`timescale 1ns / 1ps

// 中断控制器：按主机配置选择 MSI 或 legacy INTx。
//   MSI  : 每个中断电平上升沿发一次（电平未清除前不重复发）。
//   INTx : 电平语义——电平拉高发 Assert_INTx，电平落低发 Deassert_INTx；
//          断言期间若 MSI 使能或主机置 Interrupt Disable，先去断言再按
//          新模式处理，保证主机侧不会残留一条永远清除不掉的中断线。
module pcileech_interrupt_controller(
    input       rst,
    input       clk,
    input       interrupt_level,
    input       msi_enabled,
    input       int_disabled,       // 配置空间 Command.IntDisable (bit 10)
    input       request_ready,
    output reg  request_valid,
    output reg  request_assert,
    output wire [2:0] debug_state
);

    // INTX_IMPLEMENTED=1：恢复 legacy INTx（与 INTERRUPT_PIN=1 / 真卡一致，
    // b57nd60a 在本机对这颗芯片实际走传统中断）。引导期安全性由两点保证：
    // BAR 侧 irq_level 只由驱动触发的事件产生（无驱动无事件），且本控制器
    // host_irq_ready 未就绪前一律不断言——INTA 不会在 BIOS/引导期尖叫。
    localparam      INTX_IMPLEMENTED         = 1'b1;

    localparam [2:0] STATE_IDLE                  = 3'd0;
    localparam [2:0] STATE_WAIT_ENABLE           = 3'd1;
    localparam [2:0] STATE_SEND_MSI              = 3'd2;
    localparam [2:0] STATE_WAIT_MSI_SOURCE_CLEAR = 3'd3;
    localparam [2:0] STATE_SEND_ASSERT           = 3'd4;
    localparam [2:0] STATE_INTX_ASSERTED         = 3'd5;
    localparam [2:0] STATE_SEND_DEASSERT         = 3'd6;

    reg [2:0] state;
    reg       interrupt_seen;
    reg       interrupt_armed;
    reg       interrupt_level_d;

    // 方案1：延迟中断使能。
    // 复位后到驱动首次真正配置出可用的投递机制（MSI 使能，或在实现了
    // INTx 的硬核上主机未置 Interrupt Disable）之前，屏蔽所有中断源。
    // 这样 BIOS/引导阶段由 PHY 链路事件、后台 ARP 等提前产生的中断，
    // 不会在驱动 ISR 就绪前拉高中断线——既避免状态机在 STATE_WAIT_ENABLE
    // 上长期空等，也避免主机侧出现无人应答的早期中断导致引导卡死。
    // host_irq_ready 一旦置起就保持到下次复位，之后按正常电平/边沿语义投递。
    reg       host_irq_ready;
    wire      deliver_possible    = msi_enabled || (!int_disabled && INTX_IMPLEMENTED);
    wire      interrupt_level_eff = interrupt_level && host_irq_ready;
    wire      interrupt_rise      = interrupt_level_eff && !interrupt_level_d;
    assign debug_state = state;

    always @(posedge clk) begin
        if (rst) begin
            state          <= STATE_IDLE;
            request_valid  <= 1'b0;
            request_assert <= 1'b0;
            interrupt_seen <= 1'b0;
            interrupt_armed <= 1'b0;
            interrupt_level_d <= 1'b0;
            host_irq_ready <= 1'b0;
        end
        else begin
            // 主机一旦配置出可用的投递机制即视为 ISR 就绪，之后保持。
            if (deliver_possible)
                host_irq_ready <= 1'b1;
            interrupt_level_d <= interrupt_level_eff;
            if (!interrupt_level_eff) begin
                interrupt_seen <= 1'b0;
                interrupt_armed <= 1'b0;
            end
            else begin
                if (interrupt_rise)
                    interrupt_seen <= 1'b1;
                if (interrupt_rise || interrupt_armed)
                    interrupt_armed <= 1'b1;
                if ((state == STATE_SEND_MSI) && request_ready)
                    interrupt_armed <= 1'b0;
            end
            case (state)
                STATE_IDLE: begin
                    request_valid  <= 1'b0;
                    request_assert <= 1'b0;
                    if (interrupt_level_eff && interrupt_armed) begin
                        if (msi_enabled) begin
                            request_valid <= 1'b1;
                            state         <= STATE_SEND_MSI;
                        end
                        else if (!int_disabled && INTX_IMPLEMENTED) begin
                            request_valid  <= 1'b1;
                            request_assert <= 1'b1;
                            state          <= STATE_SEND_ASSERT;
                        end
                        else
                            state <= STATE_WAIT_ENABLE;
                    end
                end

                STATE_WAIT_ENABLE: begin
                    request_valid  <= 1'b0;
                    request_assert <= 1'b0;
                    if (!interrupt_level_eff)
                        state <= STATE_IDLE;
                    else if (msi_enabled && interrupt_armed) begin
                        request_valid <= 1'b1;
                        state         <= STATE_SEND_MSI;
                    end
                    else if (!int_disabled && INTX_IMPLEMENTED && interrupt_armed) begin
                        request_valid  <= 1'b1;
                        request_assert <= 1'b1;
                        state          <= STATE_SEND_ASSERT;
                    end
                end

                STATE_SEND_MSI: begin
                    request_assert <= 1'b0;
                    if (!msi_enabled) begin
                        // 禁用时主机可能在半握手中途关掉 MSI：撤回请求，
                        // 不让 cfg_interrupt 滞留堵死后续中断。
                        request_valid <= 1'b0;
                        state         <= STATE_IDLE;
                    end else if (request_ready) begin
                        request_valid <= 1'b0;
                        state         <= STATE_WAIT_MSI_SOURCE_CLEAR;
                    end
                end

                STATE_WAIT_MSI_SOURCE_CLEAR: begin
                    request_valid <= 1'b0;
                    if (!interrupt_level_eff || !msi_enabled)
                        state <= STATE_IDLE;
                end

                STATE_SEND_ASSERT: begin
                    // Assert_INTx 保持到硬核接收；硬核经 rdy 握手保证
                    // Assert/Deassert 成对按序上线。
                    if (request_ready) begin
                        request_valid  <= 1'b0;
                        request_assert <= 1'b0;
                        state          <= STATE_INTX_ASSERTED;
                    end
                end

                STATE_INTX_ASSERTED: begin
                    request_valid  <= 1'b0;
                    request_assert <= 1'b0;
                    // 电平清除、切换 MSI 或主机关中断时都需要去断言。
                    if (!interrupt_level_eff || msi_enabled || int_disabled) begin
                        request_valid <= 1'b1;
                        state         <= STATE_SEND_DEASSERT;
                    end
                end

                STATE_SEND_DEASSERT: begin
                    if (request_ready) begin
                        request_valid <= 1'b0;
                        state         <= STATE_IDLE;
                    end
                end

                default: begin
                    state          <= STATE_IDLE;
                    request_valid  <= 1'b0;
                    request_assert <= 1'b0;
                end
            endcase
        end
    end

endmodule
