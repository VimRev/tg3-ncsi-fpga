# Enigma X1 PCIe 7-series core constraints.
# The checked-in core is imported as RTL, so Vivado does not load the XDC that
# would normally accompany an XCI. Paths below are rooted at the project top.

# Hard-block placement for the xc7a75t-fgg484 Enigma X1 lane.
set_property LOC GTPE2_CHANNEL_X0Y7 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gtp_channel.gtpe2_channel_i}]
set_property LOC GTPE2_COMMON_X0Y1 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].pipe_quad.gt_common_enabled.gt_common_int.gt_common_i/qpll_wrapper_i/gtp_common.gtpe2_common_i}]
set_property LOC PCIE_X0Y0 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_block_i}]

# Core BRAM placement follows the Series-7 PCIe x1 generated-core layout.
set_property LOC RAMB36_X2Y36 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_bram_top/pcie_brams_rx/brams[3].ram/use_tdp.ramb36/genblk*.bram36_tdp_bl.bram36_tdp_bl}]
set_property LOC RAMB36_X1Y37 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_bram_top/pcie_brams_rx/brams[2].ram/use_tdp.ramb36/genblk*.bram36_tdp_bl.bram36_tdp_bl}]
set_property LOC RAMB36_X1Y36 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_bram_top/pcie_brams_rx/brams[1].ram/use_tdp.ramb36/genblk*.bram36_tdp_bl.bram36_tdp_bl}]
set_property LOC RAMB36_X1Y35 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_bram_top/pcie_brams_rx/brams[0].ram/use_tdp.ramb36/genblk*.bram36_tdp_bl.bram36_tdp_bl}]
set_property LOC RAMB36_X1Y34 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_bram_top/pcie_brams_tx/brams[0].ram/use_tdp.ramb36/genblk*.bram36_tdp_bl.bram36_tdp_bl}]
set_property LOC RAMB36_X1Y33 [get_cells {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/pcie_top_i/pcie_7x_i/pcie_bram_top/pcie_brams_tx/brams[1].ram/use_tdp.ramb36/genblk*.bram36_tdp_bl.bram36_tdp_bl}]

# The Gen2 x1 core derives its 125 MHz user clock from lane 0 TXOUTCLK.
create_clock -name pcie_txoutclk_x0y0 -period 10.000 [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gtp_channel.gtpe2_channel_i/TXOUTCLK}]

set_false_path -to [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/S0}]
set_false_path -to [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/S1}]

create_generated_clock -name pcie_clk_125mhz_x0y0 [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/mmcm_i/CLKOUT0}]
create_generated_clock -name pcie_clk_250mhz_x0y0 [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/mmcm_i/CLKOUT1}]
create_generated_clock -name pcie_clk_125mhz_mux_x0y0 \
    -source [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/I0}] \
    -divide_by 1 \
    [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/O}]
create_generated_clock -name pcie_clk_250mhz_mux_x0y0 \
    -source [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/I1}] \
    -divide_by 1 -add -master_clock [get_clocks -of_objects [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/I1}]] \
    [get_pins {i_pcileech_pcie_a7/i_pcie_7x_0/inst/inst/gt_top_i/pipe_wrapper_i/pipe_clock_int.pipe_clock_i/pclk_i1_bufgctrl.pclk_i1/O}]

set_clock_groups -name pcieclkmux -physically_exclusive \
    -group pcie_clk_125mhz_mux_x0y0 -group pcie_clk_250mhz_mux_x0y0

# These are the asynchronous status paths documented by the generated core.
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ PLPHYLNKUPN} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ PLRECEIVEDHOTRST} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ RXELECIDLE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ TXPHINITDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ TXPHALIGNDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ TXDLYSRESETDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ RXDLYSRESETDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ RXPHALIGNDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ RXCDRLOCK} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ CFGMSGRECEIVEDPMETO} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ PLL0LOCK} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ RXPMARESETDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ RXSYNCDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
set_false_path -through [get_pins -filter {REF_PIN_NAME =~ TXSYNCDONE} -of_objects [get_cells -hierarchical -filter {NAME =~ i_pcileech_pcie_a7/i_pcie_7x_0/* && PRIMITIVE_TYPE =~ IO.gt.*}]]
