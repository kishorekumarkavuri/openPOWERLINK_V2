module My_First_NiosII(
    ////////////CLOCK//////////
    CLOCK_50,
    ////////////FLASH//////
	 oFlash_Clk,
    oFlash_nCS,
    oFlash_DI,
    iFlash_DO,

);
input          CLOCK_50;
output  			oFlash_Clk;
output 			oFlash_nCS;
output 			oFlash_DI;
input          iFlash_DO;

DE2i_150_QSYS  u0(
							.clk_clk  (CLOCK_50),
							.reset_reset_n  (1'b1),
							.epcs_flash_controller_0_external_dclk  (oFlash_Clk),  // epcs_flash_controller_0_external.dclk
							.epcs_flash_controller_0_external_sce   (oFlash_nCS),   //                                 .sce
							.epcs_flash_controller_0_external_sdo   (oFlash_DI),   //                                 .sdo
							.epcs_flash_controller_0_external_data0 (iFlash_DO),  //                                 .data0
							//.nios2_qsys_cpu_resetrequest_conduit_resetrequest (1'b0),
							//.nios2_qsys_cpu_resetrequest_conduit_resettaken ()
							);  
endmodule
