module wb_sram8(
    // Generic synchronous single-port RAM interface
    clk_i, nrst_i, wb_adr_i, wb_dat_o, wb_dat_i, wb_sel_i, wb_we_i,
						 wb_stb_i, wb_cyc_i, wb_ack_o
);

    //
    // Default address and data buses width
    //
    parameter aw = 15; //number of address-bits
    parameter dw = 32; //number of data-bits

    //
    // Generic synchronous single-port RAM interface
    //

	input		clk_i;
	input 	nrst_i;
	input 	[aw-1:0] wb_adr_i;
	output   [dw-1:0] wb_dat_o;
  	input 	[dw-1:0] wb_dat_i;
	input 	[3:0] wb_sel_i;
  	input 	wb_we_i;
	input 	wb_stb_i;
	input 	wb_cyc_i;
	output reg wb_ack_o;

	//
	// generate wishbone register bank writes
	wire wb_acc = wb_cyc_i & wb_stb_i;    // WISHBONE access
	wire wb_wr  = wb_acc & wb_we_i;       // WISHBONE write access
	wire [3:0] xram_we;

   // generate ack_o
   always @(posedge clk_i)
		wb_ack_o <= #1 wb_acc & !wb_ack_o;

    wire    [1:0]    adr_low;
    wire    [7:0]    sram_dat_o;
    wire    [7:0]    sram_dat_i;

    assign adr_low = wb_sel_i == 4'b0001 ? 2'b00 : wb_sel_i == 4'b0010 ? 2'b01 : wb_sel_i == 4'b0100 ? 2'b10 : 2'b11;
    assign wb_dat_o = {sram_dat_o, sram_dat_o, sram_dat_o, sram_dat_o};
    assign sram_dat_i = wb_sel_i == 4'b0001 ? wb_dat_i[7:0] : wb_sel_i == 4'b0010 ? wb_dat_i[15:8] : wb_sel_i == 4'b0100 ? wb_dat_i[23:16] : wb_dat_i[31:24];


xram16 xram0(
	.addr({wb_adr_i[13:2],adr_low}),
	.clk(clk_i),
	.din(sram_dat_i),
	.dout(sram_dat_o),
	.we(wb_wr)
	);

endmodule
