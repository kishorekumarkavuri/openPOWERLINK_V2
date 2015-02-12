	mnSingleDualProcDrv u0 (
		.clk_clk                                    (<connected-to-clk_clk>),                                    //                           clk.clk
		.reset_reset_n                              (<connected-to-reset_reset_n>),                              //                         reset.reset_n
		.clk_100_clk                                (<connected-to-clk_100_clk>),                                //                       clk_100.clk
		.sdram_addr                                 (<connected-to-sdram_addr>),                                 //                         sdram.addr
		.sdram_ba                                   (<connected-to-sdram_ba>),                                   //                              .ba
		.sdram_cas_n                                (<connected-to-sdram_cas_n>),                                //                              .cas_n
		.sdram_cke                                  (<connected-to-sdram_cke>),                                  //                              .cke
		.sdram_cs_n                                 (<connected-to-sdram_cs_n>),                                 //                              .cs_n
		.sdram_dq                                   (<connected-to-sdram_dq>),                                   //                              .dq
		.sdram_dqm                                  (<connected-to-sdram_dqm>),                                  //                              .dqm
		.sdram_ras_n                                (<connected-to-sdram_ras_n>),                                //                              .ras_n
		.sdram_we_n                                 (<connected-to-sdram_we_n>),                                 //                              .we_n
		.led_external_connection_export             (<connected-to-led_external_connection_export>),             //       led_external_connection.export
		.button_external_connection_export          (<connected-to-button_external_connection_export>),          //    button_external_connection.export
		.pcie_ip_reconfig_togxb_data                (<connected-to-pcie_ip_reconfig_togxb_data>),                //        pcie_ip_reconfig_togxb.data
		.pcie_ip_refclk_export                      (<connected-to-pcie_ip_refclk_export>),                      //                pcie_ip_refclk.export
		.pcie_ip_test_in_test_in                    (<connected-to-pcie_ip_test_in_test_in>),                    //               pcie_ip_test_in.test_in
		.pcie_ip_pcie_rstn_export                   (<connected-to-pcie_ip_pcie_rstn_export>),                   //             pcie_ip_pcie_rstn.export
		.pcie_ip_clocks_sim_clk250_export           (<connected-to-pcie_ip_clocks_sim_clk250_export>),           //            pcie_ip_clocks_sim.clk250_export
		.pcie_ip_clocks_sim_clk500_export           (<connected-to-pcie_ip_clocks_sim_clk500_export>),           //                              .clk500_export
		.pcie_ip_clocks_sim_clk125_export           (<connected-to-pcie_ip_clocks_sim_clk125_export>),           //                              .clk125_export
		.pcie_ip_reconfig_busy_busy_altgxb_reconfig (<connected-to-pcie_ip_reconfig_busy_busy_altgxb_reconfig>), //         pcie_ip_reconfig_busy.busy_altgxb_reconfig
		.pcie_ip_pipe_ext_pipe_mode                 (<connected-to-pcie_ip_pipe_ext_pipe_mode>),                 //              pcie_ip_pipe_ext.pipe_mode
		.pcie_ip_pipe_ext_phystatus_ext             (<connected-to-pcie_ip_pipe_ext_phystatus_ext>),             //                              .phystatus_ext
		.pcie_ip_pipe_ext_rate_ext                  (<connected-to-pcie_ip_pipe_ext_rate_ext>),                  //                              .rate_ext
		.pcie_ip_pipe_ext_powerdown_ext             (<connected-to-pcie_ip_pipe_ext_powerdown_ext>),             //                              .powerdown_ext
		.pcie_ip_pipe_ext_txdetectrx_ext            (<connected-to-pcie_ip_pipe_ext_txdetectrx_ext>),            //                              .txdetectrx_ext
		.pcie_ip_pipe_ext_rxelecidle0_ext           (<connected-to-pcie_ip_pipe_ext_rxelecidle0_ext>),           //                              .rxelecidle0_ext
		.pcie_ip_pipe_ext_rxdata0_ext               (<connected-to-pcie_ip_pipe_ext_rxdata0_ext>),               //                              .rxdata0_ext
		.pcie_ip_pipe_ext_rxstatus0_ext             (<connected-to-pcie_ip_pipe_ext_rxstatus0_ext>),             //                              .rxstatus0_ext
		.pcie_ip_pipe_ext_rxvalid0_ext              (<connected-to-pcie_ip_pipe_ext_rxvalid0_ext>),              //                              .rxvalid0_ext
		.pcie_ip_pipe_ext_rxdatak0_ext              (<connected-to-pcie_ip_pipe_ext_rxdatak0_ext>),              //                              .rxdatak0_ext
		.pcie_ip_pipe_ext_txdata0_ext               (<connected-to-pcie_ip_pipe_ext_txdata0_ext>),               //                              .txdata0_ext
		.pcie_ip_pipe_ext_txdatak0_ext              (<connected-to-pcie_ip_pipe_ext_txdatak0_ext>),              //                              .txdatak0_ext
		.pcie_ip_pipe_ext_rxpolarity0_ext           (<connected-to-pcie_ip_pipe_ext_rxpolarity0_ext>),           //                              .rxpolarity0_ext
		.pcie_ip_pipe_ext_txcompl0_ext              (<connected-to-pcie_ip_pipe_ext_txcompl0_ext>),              //                              .txcompl0_ext
		.pcie_ip_pipe_ext_txelecidle0_ext           (<connected-to-pcie_ip_pipe_ext_txelecidle0_ext>),           //                              .txelecidle0_ext
		.pcie_ip_rx_in_rx_datain_0                  (<connected-to-pcie_ip_rx_in_rx_datain_0>),                  //                 pcie_ip_rx_in.rx_datain_0
		.pcie_ip_tx_out_tx_dataout_0                (<connected-to-pcie_ip_tx_out_tx_dataout_0>),                //                pcie_ip_tx_out.tx_dataout_0
		.pcie_ip_reconfig_fromgxb_0_data            (<connected-to-pcie_ip_reconfig_fromgxb_0_data>),            //    pcie_ip_reconfig_fromgxb_0.data
		.tristate_conduit_bridge_0_out_SSRAM_1_CS_N (<connected-to-tristate_conduit_bridge_0_out_SSRAM_1_CS_N>), // tristate_conduit_bridge_0_out.SSRAM_1_CS_N
		.tristate_conduit_bridge_0_out_DATA         (<connected-to-tristate_conduit_bridge_0_out_DATA>),         //                              .DATA
		.tristate_conduit_bridge_0_out_SSRAM_BE_N   (<connected-to-tristate_conduit_bridge_0_out_SSRAM_BE_N>),   //                              .SSRAM_BE_N
		.tristate_conduit_bridge_0_out_CFI_0_WR_N   (<connected-to-tristate_conduit_bridge_0_out_CFI_0_WR_N>),   //                              .CFI_0_WR_N
		.tristate_conduit_bridge_0_out_ADDR         (<connected-to-tristate_conduit_bridge_0_out_ADDR>),         //                              .ADDR
		.tristate_conduit_bridge_0_out_SSRAM_BT_N   (<connected-to-tristate_conduit_bridge_0_out_SSRAM_BT_N>),   //                              .SSRAM_BT_N
		.tristate_conduit_bridge_0_out_SSRAM_WR_N   (<connected-to-tristate_conduit_bridge_0_out_SSRAM_WR_N>),   //                              .SSRAM_WR_N
		.tristate_conduit_bridge_0_out_CFI_0_OE_N   (<connected-to-tristate_conduit_bridge_0_out_CFI_0_OE_N>),   //                              .CFI_0_OE_N
		.tristate_conduit_bridge_0_out_SSRAM_0_CS_N (<connected-to-tristate_conduit_bridge_0_out_SSRAM_0_CS_N>), //                              .SSRAM_0_CS_N
		.tristate_conduit_bridge_0_out_CFI_0_CS_N   (<connected-to-tristate_conduit_bridge_0_out_CFI_0_CS_N>),   //                              .CFI_0_CS_N
		.tristate_conduit_bridge_0_out_SSRAM_OE_N   (<connected-to-tristate_conduit_bridge_0_out_SSRAM_OE_N>),   //                              .SSRAM_OE_N
		.tristate_conduit_bridge_0_out_CFI_0_RST_N  (<connected-to-tristate_conduit_bridge_0_out_CFI_0_RST_N>),  //                              .CFI_0_RST_N
		.flash_reset_n_export                       (<connected-to-flash_reset_n_export>),                       //                 flash_reset_n.export
		.openmac_mii_txEnable                       (<connected-to-openmac_mii_txEnable>),                       //                   openmac_mii.txEnable
		.openmac_mii_txData                         (<connected-to-openmac_mii_txData>),                         //                              .txData
		.openmac_mii_txClk                          (<connected-to-openmac_mii_txClk>),                          //                              .txClk
		.openmac_mii_rxError                        (<connected-to-openmac_mii_rxError>),                        //                              .rxError
		.openmac_mii_rxDataValid                    (<connected-to-openmac_mii_rxDataValid>),                    //                              .rxDataValid
		.openmac_mii_rxData                         (<connected-to-openmac_mii_rxData>),                         //                              .rxData
		.openmac_mii_rxClk                          (<connected-to-openmac_mii_rxClk>),                          //                              .rxClk
		.openmac_smi_nPhyRst                        (<connected-to-openmac_smi_nPhyRst>),                        //                   openmac_smi.nPhyRst
		.openmac_smi_clk                            (<connected-to-openmac_smi_clk>),                            //                              .clk
		.openmac_smi_dio                            (<connected-to-openmac_smi_dio>),                            //                              .dio
		.clk_150_clk                                (<connected-to-clk_150_clk>),                                //                       clk_150.clk
		.benchmark_pio_export                       (<connected-to-benchmark_pio_export>),                       //                 benchmark_pio.export
		.host_benchmark_pio_export                  (<connected-to-host_benchmark_pio_export>)                   //            host_benchmark_pio.export
	);

