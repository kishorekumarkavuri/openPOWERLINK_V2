
module mnSingleDualProcDrv (
	clk_clk,
	reset_reset_n,
	clk_100_clk,
	sdram_addr,
	sdram_ba,
	sdram_cas_n,
	sdram_cke,
	sdram_cs_n,
	sdram_dq,
	sdram_dqm,
	sdram_ras_n,
	sdram_we_n,
	led_external_connection_export,
	button_external_connection_export,
	pcie_ip_reconfig_togxb_data,
	pcie_ip_refclk_export,
	pcie_ip_test_in_test_in,
	pcie_ip_pcie_rstn_export,
	pcie_ip_clocks_sim_clk250_export,
	pcie_ip_clocks_sim_clk500_export,
	pcie_ip_clocks_sim_clk125_export,
	pcie_ip_reconfig_busy_busy_altgxb_reconfig,
	pcie_ip_pipe_ext_pipe_mode,
	pcie_ip_pipe_ext_phystatus_ext,
	pcie_ip_pipe_ext_rate_ext,
	pcie_ip_pipe_ext_powerdown_ext,
	pcie_ip_pipe_ext_txdetectrx_ext,
	pcie_ip_pipe_ext_rxelecidle0_ext,
	pcie_ip_pipe_ext_rxdata0_ext,
	pcie_ip_pipe_ext_rxstatus0_ext,
	pcie_ip_pipe_ext_rxvalid0_ext,
	pcie_ip_pipe_ext_rxdatak0_ext,
	pcie_ip_pipe_ext_txdata0_ext,
	pcie_ip_pipe_ext_txdatak0_ext,
	pcie_ip_pipe_ext_rxpolarity0_ext,
	pcie_ip_pipe_ext_txcompl0_ext,
	pcie_ip_pipe_ext_txelecidle0_ext,
	pcie_ip_rx_in_rx_datain_0,
	pcie_ip_tx_out_tx_dataout_0,
	pcie_ip_reconfig_fromgxb_0_data,
	tristate_conduit_bridge_0_out_SSRAM_1_CS_N,
	tristate_conduit_bridge_0_out_DATA,
	tristate_conduit_bridge_0_out_SSRAM_BE_N,
	tristate_conduit_bridge_0_out_CFI_0_WR_N,
	tristate_conduit_bridge_0_out_ADDR,
	tristate_conduit_bridge_0_out_SSRAM_BT_N,
	tristate_conduit_bridge_0_out_SSRAM_WR_N,
	tristate_conduit_bridge_0_out_CFI_0_OE_N,
	tristate_conduit_bridge_0_out_SSRAM_0_CS_N,
	tristate_conduit_bridge_0_out_CFI_0_CS_N,
	tristate_conduit_bridge_0_out_SSRAM_OE_N,
	tristate_conduit_bridge_0_out_CFI_0_RST_N,
	flash_reset_n_export,
	openmac_mii_txEnable,
	openmac_mii_txData,
	openmac_mii_txClk,
	openmac_mii_rxError,
	openmac_mii_rxDataValid,
	openmac_mii_rxData,
	openmac_mii_rxClk,
	openmac_smi_nPhyRst,
	openmac_smi_clk,
	openmac_smi_dio,
	clk_150_clk,
	benchmark_pio_export,
	host_benchmark_pio_export);	

	input		clk_clk;
	input		reset_reset_n;
	input		clk_100_clk;
	output	[12:0]	sdram_addr;
	output	[1:0]	sdram_ba;
	output		sdram_cas_n;
	output		sdram_cke;
	output		sdram_cs_n;
	inout	[31:0]	sdram_dq;
	output	[3:0]	sdram_dqm;
	output		sdram_ras_n;
	output		sdram_we_n;
	output	[20:0]	led_external_connection_export;
	input	[3:0]	button_external_connection_export;
	input	[3:0]	pcie_ip_reconfig_togxb_data;
	input		pcie_ip_refclk_export;
	input	[39:0]	pcie_ip_test_in_test_in;
	input		pcie_ip_pcie_rstn_export;
	output		pcie_ip_clocks_sim_clk250_export;
	output		pcie_ip_clocks_sim_clk500_export;
	output		pcie_ip_clocks_sim_clk125_export;
	input		pcie_ip_reconfig_busy_busy_altgxb_reconfig;
	input		pcie_ip_pipe_ext_pipe_mode;
	input		pcie_ip_pipe_ext_phystatus_ext;
	output		pcie_ip_pipe_ext_rate_ext;
	output	[1:0]	pcie_ip_pipe_ext_powerdown_ext;
	output		pcie_ip_pipe_ext_txdetectrx_ext;
	input		pcie_ip_pipe_ext_rxelecidle0_ext;
	input	[7:0]	pcie_ip_pipe_ext_rxdata0_ext;
	input	[2:0]	pcie_ip_pipe_ext_rxstatus0_ext;
	input		pcie_ip_pipe_ext_rxvalid0_ext;
	input		pcie_ip_pipe_ext_rxdatak0_ext;
	output	[7:0]	pcie_ip_pipe_ext_txdata0_ext;
	output		pcie_ip_pipe_ext_txdatak0_ext;
	output		pcie_ip_pipe_ext_rxpolarity0_ext;
	output		pcie_ip_pipe_ext_txcompl0_ext;
	output		pcie_ip_pipe_ext_txelecidle0_ext;
	input		pcie_ip_rx_in_rx_datain_0;
	output		pcie_ip_tx_out_tx_dataout_0;
	output	[4:0]	pcie_ip_reconfig_fromgxb_0_data;
	output	[0:0]	tristate_conduit_bridge_0_out_SSRAM_1_CS_N;
	inout	[31:0]	tristate_conduit_bridge_0_out_DATA;
	output	[3:0]	tristate_conduit_bridge_0_out_SSRAM_BE_N;
	output	[0:0]	tristate_conduit_bridge_0_out_CFI_0_WR_N;
	output	[25:0]	tristate_conduit_bridge_0_out_ADDR;
	output	[0:0]	tristate_conduit_bridge_0_out_SSRAM_BT_N;
	output	[0:0]	tristate_conduit_bridge_0_out_SSRAM_WR_N;
	output	[0:0]	tristate_conduit_bridge_0_out_CFI_0_OE_N;
	output	[0:0]	tristate_conduit_bridge_0_out_SSRAM_0_CS_N;
	output	[0:0]	tristate_conduit_bridge_0_out_CFI_0_CS_N;
	output	[0:0]	tristate_conduit_bridge_0_out_SSRAM_OE_N;
	output	[0:0]	tristate_conduit_bridge_0_out_CFI_0_RST_N;
	output		flash_reset_n_export;
	output	[0:0]	openmac_mii_txEnable;
	output	[3:0]	openmac_mii_txData;
	input	[0:0]	openmac_mii_txClk;
	input	[0:0]	openmac_mii_rxError;
	input	[0:0]	openmac_mii_rxDataValid;
	input	[3:0]	openmac_mii_rxData;
	input	[0:0]	openmac_mii_rxClk;
	output	[0:0]	openmac_smi_nPhyRst;
	output	[0:0]	openmac_smi_clk;
	inout	[0:0]	openmac_smi_dio;
	input		clk_150_clk;
	output	[7:0]	benchmark_pio_export;
	output	[7:0]	host_benchmark_pio_export;
endmodule
