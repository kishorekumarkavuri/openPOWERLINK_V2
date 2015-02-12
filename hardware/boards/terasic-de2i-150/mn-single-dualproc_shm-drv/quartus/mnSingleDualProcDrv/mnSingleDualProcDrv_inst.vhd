	component mnSingleDualProcDrv is
		port (
			clk_clk                                    : in    std_logic                     := 'X';             -- clk
			reset_reset_n                              : in    std_logic                     := 'X';             -- reset_n
			clk_100_clk                                : in    std_logic                     := 'X';             -- clk
			sdram_addr                                 : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_ba                                   : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_cas_n                                : out   std_logic;                                        -- cas_n
			sdram_cke                                  : out   std_logic;                                        -- cke
			sdram_cs_n                                 : out   std_logic;                                        -- cs_n
			sdram_dq                                   : inout std_logic_vector(31 downto 0) := (others => 'X'); -- dq
			sdram_dqm                                  : out   std_logic_vector(3 downto 0);                     -- dqm
			sdram_ras_n                                : out   std_logic;                                        -- ras_n
			sdram_we_n                                 : out   std_logic;                                        -- we_n
			led_external_connection_export             : out   std_logic_vector(20 downto 0);                    -- export
			button_external_connection_export          : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- export
			pcie_ip_reconfig_togxb_data                : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- data
			pcie_ip_refclk_export                      : in    std_logic                     := 'X';             -- export
			pcie_ip_test_in_test_in                    : in    std_logic_vector(39 downto 0) := (others => 'X'); -- test_in
			pcie_ip_pcie_rstn_export                   : in    std_logic                     := 'X';             -- export
			pcie_ip_clocks_sim_clk250_export           : out   std_logic;                                        -- clk250_export
			pcie_ip_clocks_sim_clk500_export           : out   std_logic;                                        -- clk500_export
			pcie_ip_clocks_sim_clk125_export           : out   std_logic;                                        -- clk125_export
			pcie_ip_reconfig_busy_busy_altgxb_reconfig : in    std_logic                     := 'X';             -- busy_altgxb_reconfig
			pcie_ip_pipe_ext_pipe_mode                 : in    std_logic                     := 'X';             -- pipe_mode
			pcie_ip_pipe_ext_phystatus_ext             : in    std_logic                     := 'X';             -- phystatus_ext
			pcie_ip_pipe_ext_rate_ext                  : out   std_logic;                                        -- rate_ext
			pcie_ip_pipe_ext_powerdown_ext             : out   std_logic_vector(1 downto 0);                     -- powerdown_ext
			pcie_ip_pipe_ext_txdetectrx_ext            : out   std_logic;                                        -- txdetectrx_ext
			pcie_ip_pipe_ext_rxelecidle0_ext           : in    std_logic                     := 'X';             -- rxelecidle0_ext
			pcie_ip_pipe_ext_rxdata0_ext               : in    std_logic_vector(7 downto 0)  := (others => 'X'); -- rxdata0_ext
			pcie_ip_pipe_ext_rxstatus0_ext             : in    std_logic_vector(2 downto 0)  := (others => 'X'); -- rxstatus0_ext
			pcie_ip_pipe_ext_rxvalid0_ext              : in    std_logic                     := 'X';             -- rxvalid0_ext
			pcie_ip_pipe_ext_rxdatak0_ext              : in    std_logic                     := 'X';             -- rxdatak0_ext
			pcie_ip_pipe_ext_txdata0_ext               : out   std_logic_vector(7 downto 0);                     -- txdata0_ext
			pcie_ip_pipe_ext_txdatak0_ext              : out   std_logic;                                        -- txdatak0_ext
			pcie_ip_pipe_ext_rxpolarity0_ext           : out   std_logic;                                        -- rxpolarity0_ext
			pcie_ip_pipe_ext_txcompl0_ext              : out   std_logic;                                        -- txcompl0_ext
			pcie_ip_pipe_ext_txelecidle0_ext           : out   std_logic;                                        -- txelecidle0_ext
			pcie_ip_rx_in_rx_datain_0                  : in    std_logic                     := 'X';             -- rx_datain_0
			pcie_ip_tx_out_tx_dataout_0                : out   std_logic;                                        -- tx_dataout_0
			pcie_ip_reconfig_fromgxb_0_data            : out   std_logic_vector(4 downto 0);                     -- data
			tristate_conduit_bridge_0_out_SSRAM_1_CS_N : out   std_logic_vector(0 downto 0);                     -- SSRAM_1_CS_N
			tristate_conduit_bridge_0_out_DATA         : inout std_logic_vector(31 downto 0) := (others => 'X'); -- DATA
			tristate_conduit_bridge_0_out_SSRAM_BE_N   : out   std_logic_vector(3 downto 0);                     -- SSRAM_BE_N
			tristate_conduit_bridge_0_out_CFI_0_WR_N   : out   std_logic_vector(0 downto 0);                     -- CFI_0_WR_N
			tristate_conduit_bridge_0_out_ADDR         : out   std_logic_vector(25 downto 0);                    -- ADDR
			tristate_conduit_bridge_0_out_SSRAM_BT_N   : out   std_logic_vector(0 downto 0);                     -- SSRAM_BT_N
			tristate_conduit_bridge_0_out_SSRAM_WR_N   : out   std_logic_vector(0 downto 0);                     -- SSRAM_WR_N
			tristate_conduit_bridge_0_out_CFI_0_OE_N   : out   std_logic_vector(0 downto 0);                     -- CFI_0_OE_N
			tristate_conduit_bridge_0_out_SSRAM_0_CS_N : out   std_logic_vector(0 downto 0);                     -- SSRAM_0_CS_N
			tristate_conduit_bridge_0_out_CFI_0_CS_N   : out   std_logic_vector(0 downto 0);                     -- CFI_0_CS_N
			tristate_conduit_bridge_0_out_SSRAM_OE_N   : out   std_logic_vector(0 downto 0);                     -- SSRAM_OE_N
			tristate_conduit_bridge_0_out_CFI_0_RST_N  : out   std_logic_vector(0 downto 0);                     -- CFI_0_RST_N
			flash_reset_n_export                       : out   std_logic;                                        -- export
			openmac_mii_txEnable                       : out   std_logic_vector(0 downto 0);                     -- txEnable
			openmac_mii_txData                         : out   std_logic_vector(3 downto 0);                     -- txData
			openmac_mii_txClk                          : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- txClk
			openmac_mii_rxError                        : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- rxError
			openmac_mii_rxDataValid                    : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- rxDataValid
			openmac_mii_rxData                         : in    std_logic_vector(3 downto 0)  := (others => 'X'); -- rxData
			openmac_mii_rxClk                          : in    std_logic_vector(0 downto 0)  := (others => 'X'); -- rxClk
			openmac_smi_nPhyRst                        : out   std_logic_vector(0 downto 0);                     -- nPhyRst
			openmac_smi_clk                            : out   std_logic_vector(0 downto 0);                     -- clk
			openmac_smi_dio                            : inout std_logic_vector(0 downto 0)  := (others => 'X'); -- dio
			clk_150_clk                                : in    std_logic                     := 'X';             -- clk
			benchmark_pio_export                       : out   std_logic_vector(7 downto 0);                     -- export
			host_benchmark_pio_export                  : out   std_logic_vector(7 downto 0)                      -- export
		);
	end component mnSingleDualProcDrv;

	u0 : component mnSingleDualProcDrv
		port map (
			clk_clk                                    => CONNECTED_TO_clk_clk,                                    --                           clk.clk
			reset_reset_n                              => CONNECTED_TO_reset_reset_n,                              --                         reset.reset_n
			clk_100_clk                                => CONNECTED_TO_clk_100_clk,                                --                       clk_100.clk
			sdram_addr                                 => CONNECTED_TO_sdram_addr,                                 --                         sdram.addr
			sdram_ba                                   => CONNECTED_TO_sdram_ba,                                   --                              .ba
			sdram_cas_n                                => CONNECTED_TO_sdram_cas_n,                                --                              .cas_n
			sdram_cke                                  => CONNECTED_TO_sdram_cke,                                  --                              .cke
			sdram_cs_n                                 => CONNECTED_TO_sdram_cs_n,                                 --                              .cs_n
			sdram_dq                                   => CONNECTED_TO_sdram_dq,                                   --                              .dq
			sdram_dqm                                  => CONNECTED_TO_sdram_dqm,                                  --                              .dqm
			sdram_ras_n                                => CONNECTED_TO_sdram_ras_n,                                --                              .ras_n
			sdram_we_n                                 => CONNECTED_TO_sdram_we_n,                                 --                              .we_n
			led_external_connection_export             => CONNECTED_TO_led_external_connection_export,             --       led_external_connection.export
			button_external_connection_export          => CONNECTED_TO_button_external_connection_export,          --    button_external_connection.export
			pcie_ip_reconfig_togxb_data                => CONNECTED_TO_pcie_ip_reconfig_togxb_data,                --        pcie_ip_reconfig_togxb.data
			pcie_ip_refclk_export                      => CONNECTED_TO_pcie_ip_refclk_export,                      --                pcie_ip_refclk.export
			pcie_ip_test_in_test_in                    => CONNECTED_TO_pcie_ip_test_in_test_in,                    --               pcie_ip_test_in.test_in
			pcie_ip_pcie_rstn_export                   => CONNECTED_TO_pcie_ip_pcie_rstn_export,                   --             pcie_ip_pcie_rstn.export
			pcie_ip_clocks_sim_clk250_export           => CONNECTED_TO_pcie_ip_clocks_sim_clk250_export,           --            pcie_ip_clocks_sim.clk250_export
			pcie_ip_clocks_sim_clk500_export           => CONNECTED_TO_pcie_ip_clocks_sim_clk500_export,           --                              .clk500_export
			pcie_ip_clocks_sim_clk125_export           => CONNECTED_TO_pcie_ip_clocks_sim_clk125_export,           --                              .clk125_export
			pcie_ip_reconfig_busy_busy_altgxb_reconfig => CONNECTED_TO_pcie_ip_reconfig_busy_busy_altgxb_reconfig, --         pcie_ip_reconfig_busy.busy_altgxb_reconfig
			pcie_ip_pipe_ext_pipe_mode                 => CONNECTED_TO_pcie_ip_pipe_ext_pipe_mode,                 --              pcie_ip_pipe_ext.pipe_mode
			pcie_ip_pipe_ext_phystatus_ext             => CONNECTED_TO_pcie_ip_pipe_ext_phystatus_ext,             --                              .phystatus_ext
			pcie_ip_pipe_ext_rate_ext                  => CONNECTED_TO_pcie_ip_pipe_ext_rate_ext,                  --                              .rate_ext
			pcie_ip_pipe_ext_powerdown_ext             => CONNECTED_TO_pcie_ip_pipe_ext_powerdown_ext,             --                              .powerdown_ext
			pcie_ip_pipe_ext_txdetectrx_ext            => CONNECTED_TO_pcie_ip_pipe_ext_txdetectrx_ext,            --                              .txdetectrx_ext
			pcie_ip_pipe_ext_rxelecidle0_ext           => CONNECTED_TO_pcie_ip_pipe_ext_rxelecidle0_ext,           --                              .rxelecidle0_ext
			pcie_ip_pipe_ext_rxdata0_ext               => CONNECTED_TO_pcie_ip_pipe_ext_rxdata0_ext,               --                              .rxdata0_ext
			pcie_ip_pipe_ext_rxstatus0_ext             => CONNECTED_TO_pcie_ip_pipe_ext_rxstatus0_ext,             --                              .rxstatus0_ext
			pcie_ip_pipe_ext_rxvalid0_ext              => CONNECTED_TO_pcie_ip_pipe_ext_rxvalid0_ext,              --                              .rxvalid0_ext
			pcie_ip_pipe_ext_rxdatak0_ext              => CONNECTED_TO_pcie_ip_pipe_ext_rxdatak0_ext,              --                              .rxdatak0_ext
			pcie_ip_pipe_ext_txdata0_ext               => CONNECTED_TO_pcie_ip_pipe_ext_txdata0_ext,               --                              .txdata0_ext
			pcie_ip_pipe_ext_txdatak0_ext              => CONNECTED_TO_pcie_ip_pipe_ext_txdatak0_ext,              --                              .txdatak0_ext
			pcie_ip_pipe_ext_rxpolarity0_ext           => CONNECTED_TO_pcie_ip_pipe_ext_rxpolarity0_ext,           --                              .rxpolarity0_ext
			pcie_ip_pipe_ext_txcompl0_ext              => CONNECTED_TO_pcie_ip_pipe_ext_txcompl0_ext,              --                              .txcompl0_ext
			pcie_ip_pipe_ext_txelecidle0_ext           => CONNECTED_TO_pcie_ip_pipe_ext_txelecidle0_ext,           --                              .txelecidle0_ext
			pcie_ip_rx_in_rx_datain_0                  => CONNECTED_TO_pcie_ip_rx_in_rx_datain_0,                  --                 pcie_ip_rx_in.rx_datain_0
			pcie_ip_tx_out_tx_dataout_0                => CONNECTED_TO_pcie_ip_tx_out_tx_dataout_0,                --                pcie_ip_tx_out.tx_dataout_0
			pcie_ip_reconfig_fromgxb_0_data            => CONNECTED_TO_pcie_ip_reconfig_fromgxb_0_data,            --    pcie_ip_reconfig_fromgxb_0.data
			tristate_conduit_bridge_0_out_SSRAM_1_CS_N => CONNECTED_TO_tristate_conduit_bridge_0_out_SSRAM_1_CS_N, -- tristate_conduit_bridge_0_out.SSRAM_1_CS_N
			tristate_conduit_bridge_0_out_DATA         => CONNECTED_TO_tristate_conduit_bridge_0_out_DATA,         --                              .DATA
			tristate_conduit_bridge_0_out_SSRAM_BE_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_SSRAM_BE_N,   --                              .SSRAM_BE_N
			tristate_conduit_bridge_0_out_CFI_0_WR_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_CFI_0_WR_N,   --                              .CFI_0_WR_N
			tristate_conduit_bridge_0_out_ADDR         => CONNECTED_TO_tristate_conduit_bridge_0_out_ADDR,         --                              .ADDR
			tristate_conduit_bridge_0_out_SSRAM_BT_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_SSRAM_BT_N,   --                              .SSRAM_BT_N
			tristate_conduit_bridge_0_out_SSRAM_WR_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_SSRAM_WR_N,   --                              .SSRAM_WR_N
			tristate_conduit_bridge_0_out_CFI_0_OE_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_CFI_0_OE_N,   --                              .CFI_0_OE_N
			tristate_conduit_bridge_0_out_SSRAM_0_CS_N => CONNECTED_TO_tristate_conduit_bridge_0_out_SSRAM_0_CS_N, --                              .SSRAM_0_CS_N
			tristate_conduit_bridge_0_out_CFI_0_CS_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_CFI_0_CS_N,   --                              .CFI_0_CS_N
			tristate_conduit_bridge_0_out_SSRAM_OE_N   => CONNECTED_TO_tristate_conduit_bridge_0_out_SSRAM_OE_N,   --                              .SSRAM_OE_N
			tristate_conduit_bridge_0_out_CFI_0_RST_N  => CONNECTED_TO_tristate_conduit_bridge_0_out_CFI_0_RST_N,  --                              .CFI_0_RST_N
			flash_reset_n_export                       => CONNECTED_TO_flash_reset_n_export,                       --                 flash_reset_n.export
			openmac_mii_txEnable                       => CONNECTED_TO_openmac_mii_txEnable,                       --                   openmac_mii.txEnable
			openmac_mii_txData                         => CONNECTED_TO_openmac_mii_txData,                         --                              .txData
			openmac_mii_txClk                          => CONNECTED_TO_openmac_mii_txClk,                          --                              .txClk
			openmac_mii_rxError                        => CONNECTED_TO_openmac_mii_rxError,                        --                              .rxError
			openmac_mii_rxDataValid                    => CONNECTED_TO_openmac_mii_rxDataValid,                    --                              .rxDataValid
			openmac_mii_rxData                         => CONNECTED_TO_openmac_mii_rxData,                         --                              .rxData
			openmac_mii_rxClk                          => CONNECTED_TO_openmac_mii_rxClk,                          --                              .rxClk
			openmac_smi_nPhyRst                        => CONNECTED_TO_openmac_smi_nPhyRst,                        --                   openmac_smi.nPhyRst
			openmac_smi_clk                            => CONNECTED_TO_openmac_smi_clk,                            --                              .clk
			openmac_smi_dio                            => CONNECTED_TO_openmac_smi_dio,                            --                              .dio
			clk_150_clk                                => CONNECTED_TO_clk_150_clk,                                --                       clk_150.clk
			benchmark_pio_export                       => CONNECTED_TO_benchmark_pio_export,                       --                 benchmark_pio.export
			host_benchmark_pio_export                  => CONNECTED_TO_host_benchmark_pio_export                   --            host_benchmark_pio.export
		);

