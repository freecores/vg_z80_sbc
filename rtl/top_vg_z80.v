//////////////////////////////////////////////////////////////////////
////                                                              ////
////  $Id: top_vg_z80.v,v 1.1 2008-12-01 02:00:10 hharte Exp $        ////
////  top_sk_z80.v - Z80 SBC Based on Xilinx S3E Starter Kit      ////
////                 Top-Level                                    ////
////                                                              ////
////  This file is part of the Vector Graphic Z80 SBC Project     ////
////  http://www.opencores.org/projects/vg_z80_sbc/               ////
////                                                              ////
////  Author:                                                     ////
////      - Howard M. Harte (hharte@opencores.org)                ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2008 Howard M. Harte                           ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////

//+---------------------------------------------------------------------------+
//| 16 1MB Address Regions:
//| 
//| 0:00000 - Wishbone I/O
//| 1:00000 - SRAM (32k)
//| 2:00000 - FLASH (512K)
//| 6:00000 - VGA Controller
//| 8:00000 - DDR SDRAM
//| E:00000 - PS2 Keyboard
//| F:00000 - MMU
//| 
//| 16 4K Entries:
//| 
//| 0:00xxx - Lower <xxx> 12 address bits are passed through unchanged.
//| 
//| Mapping register (MMR) is divided into two fields:
//| 
//| Upper 4-bits = MMR_H  <h>
//| Lower 8-bits = MMR_L  <ll>
//| 
//| forms the final 24-bit address as follows:
//| 
//| <h>:<llxxx>
//| 
//| This provides for 16MB of address space from 64K,
//+---------------------------------------------------------------------------+

`include "ddr_include.v"

`define USE_INTERNAL_RAM

module vg_z80_sbc
(
    CLK,
    RST, // Active Low

    UART0_BR_CLK,
    UART0_TXD,
    UART0_RXD,

    UART1_TXD,
    UART1_RXD,

    FLASH_A,
    FLASH_D,
    FLASH_CE,
    FLASH_OE,
    FLASH_WE,
    FLASH_BYTE,
     
    hsync,
    vsync,
    R,
    G,
    B,
     
    PS2_KBD_CLK,
    PS2_KBD_DAT,
     
    SD_A,
    SD_DQ,
    SD_BA,
    SD_CAS,
    SD_CK_N,
    SD_CK_P,
    SD_CKE,
    SD_CS,
     
    SD_DM,
    SD_DQS,
    SD_RAS,
    SD_WE,
    SD_CK_FB,

    rot
);

input         CLK ;
input         RST ;

// UARTS
output        UART0_BR_CLK;
output        UART0_TXD;
input         UART0_RXD;
output        UART1_TXD;
input         UART1_RXD;

// FLASH Memory Interface
output [23:0] FLASH_A;
inout   [7:0] FLASH_D;
output        FLASH_CE;
output        FLASH_OE;
output        FLASH_WE;
output        FLASH_BYTE;

// VGA
output             hsync, vsync, R, G, B;

// PS/2
inout              PS2_KBD_CLK;
inout              PS2_KBD_DAT;

// DDR Interface
//output [12:0]  SD_A;
//output  [1:0]  SD_BA;
//inout  [15:0]  SD_DQ;
//inout   [1:0]  SD_DQS;
//output  [1:0]  SD_DM;
output             SD_CAS;
output             SD_CK_N;
output             SD_CK_P;
output             SD_CKE;
output             SD_CS;
     
output             SD_RAS;
output             SD_WE;
input              SD_CK_FB;

output [  `A_RNG]  SD_A;
output [ `BA_RNG]  SD_BA;
inout  [ `DQ_RNG]  SD_DQ;
inout  [`DQS_RNG]  SD_DQS;
output [ `DM_RNG]  SD_DM;

input  [2:0]       rot;

wire    NRST = !RST;

wire  [7:0] flash_dat_i;
wire  [7:0] flash_dat_o;
wire [18:0] flash_adr_o;
wire        flash_ce_o, flash_oe_o, flash_we_o;

reg clk25mhz;

// Generate 25MHz Clock from 50MHz clock input
always @(posedge CLK or posedge RST)
    if (RST)
        begin
            clk25mhz <= 1'b0;
        end else begin
            clk25mhz <= !clk25mhz;
        end

assign FLASH_A = {5'b0,flash_adr_o};
assign FLASH_D = (!flash_we_o) ? flash_dat_o : 8'bZZZZZZZZ;
assign flash_dat_i = FLASH_D;
assign FLASH_CE = flash_ce_o;
assign FLASH_OE = flash_oe_o;
assign FLASH_WE = flash_we_o;
assign FLASH_BYTE = 1'b0;

wire [15:0] rgb_int ;
// WISHBONE slave interface
wire    [31:0]  ADR_I = 32'h00000000;
wire    [31:0]  SDAT_I = 32'hffffffff;
wire    [31:0]  SDAT_O ;
wire    [3:0]   SEL_I = 1'b0;
wire            CYC_I = 1'b0;
wire            STB_I = 1'b0;
wire            WE_I  = 1'b0;
wire            CAB_I = 1'b0;
wire            ACK_O ;
wire            RTY_O ;
wire            ERR_O ;

// WISHBONE master interface
wire [31:0]  ADR_O ;
wire [31:0]  MDAT_I; 
wire [31:0]  MDAT_O = 32'h00000000;
wire [3:0]   SEL_O = 4'b0000;
wire         CYC_O = 1'b0;
wire         STB_O = 1'b0;
wire         WE_O  = 1'b0;
wire         CAB_O = 1'b0;
wire         ACK_I;
wire         RTY_I;
wire         ERR_I;

wire         PCI_CLK = clk25mhz;

wire [31:0] wb_z80_dat_o;
wire        wb_z80_stb_o;
wire        wb_z80_cyc_o;
wire        wb_z80_we_o;
wire [15:0] wb_z80_adr_o;
wire [1:0]  wb_z80_tga_o;
wire        wb_z80_ack_i;
wire [31:0] wb_z80_dat_i;
wire        z80_int_req_i;
wire        wb_z80_err_i;
wire [3:0]  wb_z80_sel_o;
wire        z80_cpu_rst;

wire [7:0]  wb_z80_be_dat_i;    // dat moved to correct byte lane depending on sel lines.
wire [7:0]  wb_z80_final_dat_i;

wire        z80_mem_hit;
assign z80_mem_hit = wb_z80_tga_o == 2'b00;

assign wb_z80_int_req_i = 1'h0;
assign wb_z80_sel_o = wb_z80_adr_o[1:0] == 2'b00 ? 4'b0001 :
                      wb_z80_adr_o[1:0] == 2'b01 ? 4'b0010 : 
                      wb_z80_adr_o[1:0] == 2'b10 ? 4'b0100 : 4'b1000;

assign wb_z80_dat_o[15:8]  = wb_z80_dat_o[7:0];
assign wb_z80_dat_o[23:16] = wb_z80_dat_o[7:0];
assign wb_z80_dat_o[31:24] = wb_z80_dat_o[7:0];

assign wb_z80_be_dat_i = wb_z80_adr_o[1:0] == 2'b00 ? wb_z80_dat_i[7:0] :
                         wb_z80_adr_o[1:0] == 2'b01 ? wb_z80_dat_i[15:8] : 
                         wb_z80_adr_o[1:0] == 2'b10 ? wb_z80_dat_i[23:16] : wb_z80_dat_i[31:24];

assign wb_z80_final_dat_i = z80_mem_hit ? wb_z80_be_dat_i : wb_z80_dat_i[7:0]; 

// Instantiate the Wishbone Z80 Core
z80_core_top z80cpu (
    .wb_dat_o(wb_z80_dat_o[7:0]), 
    .wb_stb_o(wb_z80_stb_o), 
    .wb_cyc_o(wb_z80_cyc_o), 
    .wb_we_o(wb_z80_we_o), 
    .wb_adr_o(wb_z80_adr_o), 
    .wb_tga_o(wb_z80_tga_o), 
    .wb_ack_i(wb_z80_ack_i), 
    .wb_clk_i(PCI_CLK), 
    .wb_dat_i(wb_z80_final_dat_i),
    .wb_rst_i(RST), 
    .int_req_i(z80_int_req_i)
    );

// Instantiate the CPU Controller
wire [31:0] wb_cpu_ctrl_dat_o;
wire [31:0] wb_cpu_ctrl_dat_i;
wire  [3:0] wb_cpu_ctrl_sel_i;
wire        wb_cpu_ctrl_we_i;
wire        wb_cpu_ctrl_stb_i;
wire        wb_cpu_ctrl_cyc_i;
wire        wb_cpu_ctrl_ack_o;
wire  [5:0] wb_cpu_ctrl_adr_i;
wire [31:0] cpu_ctrl_reg0;
wire [31:0] cpu_ctrl_reg1;

wb_cpu_ctrl cpu_ctrl0 (
    .clk_i(PCI_CLK), 
    .nrst_i(NRST), 
    .wb_adr_i(wb_cpu_ctrl_adr_i[2:0]), 
    .wb_dat_o(wb_cpu_ctrl_dat_o), 
    .wb_dat_i(wb_cpu_ctrl_dat_i), 
    .wb_sel_i(wb_cpu_ctrl_sel_i), 
    .wb_we_i(wb_cpu_ctrl_we_i), 
    .wb_stb_i(wb_cpu_ctrl_stb_i), 
    .wb_cyc_i(wb_cpu_ctrl_cyc_i), 
    .wb_ack_o(wb_cpu_ctrl_ack_o),
    .datareg0(cpu_ctrl_reg0),
    .datareg1(cpu_ctrl_reg1)
    );

assign z80_cpu_rst = cpu_ctrl_reg0[0];

`ifdef USE_INTERNAL_RAM
// Instantiate the SRAM
wire [31:0] wb_ram_dat_o;
wire [31:0] wb_ram_dat_i;
wire [3:0]  wb_ram_sel_i;
wire        wb_ram_we_i;
wire        wb_ram_stb_i;
wire        wb_ram_cyc_i;
wire        wb_ram_ack_o;
wire [14:0] wb_ram_adr_i;

wb_sram8 sram0 (
    .clk_i(PCI_CLK), 
    .nrst_i(NRST), 
    .wb_adr_i(wb_ram_adr_i), 
    .wb_dat_o(wb_ram_dat_o), 
    .wb_dat_i(wb_ram_dat_i), 
    .wb_sel_i(wb_ram_sel_i), 
    .wb_we_i(wb_ram_we_i), 
    .wb_stb_i(wb_ram_stb_i), 
    .wb_cyc_i(wb_ram_cyc_i), 
    .wb_ack_o(wb_ram_ack_o)
    );
`endif // USE_INTERNAL_RAM

wire [31:0] wbs_vga_dat_o; 
wire        wbs_vga_ack_o; 
wire [31:0] wbs_vga_dat_i; 
wire        wbs_vga_we_i; 
wire  [3:0] wbs_vga_sel_i;
wire [13:0] wbs_vga_adr_i;
wire        wbs_vga_cyc_i; 
wire        wbs_vga_stb_i; 

// Instantiate the VGA Controller (Emulating Vector Graphic FlashWriter2)
wb_vga #(
    .font_height(10),
    .text_height(2))
vga0 (
    .clk_i(PCI_CLK), 
    .clk_50mhz_i(CLK), 
    .nrst_i(NRST), 
    .wb_adr_i(wbs_vga_adr_i), 
    .wb_dat_o(wbs_vga_dat_o), 
    .wb_dat_i(wbs_vga_dat_i), 
    .wb_sel_i(wbs_vga_sel_i), 
    .wb_we_i(wbs_vga_we_i), 
    .wb_stb_i(wbs_vga_stb_i), 
    .wb_cyc_i(wbs_vga_cyc_i), 
    .wb_ack_o(wbs_vga_ack_o), 
    .vga_hsync_o(hsync), 
    .vga_vsync_o(vsync), 
    .vga_r_o(R), 
    .vga_g_o(G), 
    .vga_b_o(B)
    );

wire  [2:0] wb_uart0_adr_i;
wire [31:0] wb_uart0_dat_i;
wire [31:0] wb_uart0_dat_o;
wire        wb_uart0_we_i;
wire  [3:0] wb_uart0_sel_i;
wire        wb_uart0_cyc_i;
wire        wb_uart0_stb_i;
wire        wb_uart0_ack_o;
wire        uart0_int_o;
wire        uart0_stx_pad_o;
wire        uart0_rts_pad_o;
wire        uart0_cts_pad_i;
wire        uart0_dtr_pad_o;
wire        uart0_dsr_pad_i;
wire        uart0_ri_pad_i;
wire        uart0_dcd_pad_i;
wire        uart0_baud_o;

assign wb_uart0_dat_o[31:8] = 24'h000000;
// Instantiate the Console UART
uart_top uart0 (
    .wb_clk_i(PCI_CLK), 
    .wb_rst_i(RST), 
    .wb_adr_i(wb_uart0_adr_i), 
    .wb_dat_i(wb_uart0_dat_i[7:0]), 
    .wb_dat_o(wb_uart0_dat_o[7:0]), 
    .wb_we_i(wb_uart0_we_i), 
    .wb_stb_i(wb_uart0_stb_i), 
    .wb_cyc_i(wb_uart0_cyc_i), 
    .wb_ack_o(wb_uart0_ack_o), 
    .wb_sel_i(wb_uart0_sel_i), 
    .int_o(uart0_int_o), 
    .stx_pad_o(uart0_stx_pad_o), 
    .srx_pad_i(UART0_RXD), 
    .rts_pad_o(uart0_rts_pad_o), 
    .cts_pad_i(uart0_cts_pad_i), 
    .dtr_pad_o(uart0_dtr_pad_o), 
    .dsr_pad_i(uart0_dsr_pad_i), 
    .ri_pad_i(uart0_ri_pad_i), 
    .dcd_pad_i(uart0_dcd_pad_i),
    .baud_o(uart0_baud_o)
    );

assign UART0_BR_CLK = uart0_baud_o;
assign UART0_TXD = uart0_stx_pad_o;

wire  [4:0] wb_uart1_adr_i;
wire [31:0] wb_uart1_dat_i;
wire [31:0] wb_uart1_dat_o;
wire        wb_uart1_we_i;
wire  [3:0] wb_uart1_sel_i;
wire        wb_uart1_cyc_i;
wire        wb_uart1_stb_i;
wire        wb_uart1_ack_o;

wire        uart_lb_tx;
wire        uart_lb_rx;
wire        uart_lb2_tx;
wire        uart_lb2_rx;

// Instantiate more UARTs and most importantly, the PS/2 Keyboard
// 
wb_uart #(
    .clk_freq(25000000),
    .baud(115200)
) wb_uart1 (
    .clk(PCI_CLK), 
    .reset(RST), 
    .wb_stb_i(wb_uart1_stb_i), 
    .wb_cyc_i(wb_uart1_cyc_i), 
    .wb_ack_o(wb_uart1_ack_o), 
    .wb_we_i(wb_uart1_we_i), 
    .wb_adr_i(wb_uart1_adr_i), 
    .wb_sel_i(wb_uart1_sel_i), 
    .wb_dat_i(wb_uart1_dat_i), 
    .wb_dat_o(wb_uart1_dat_o), 
    .ps2_clk(PS2_KBD_CLK), 
    .ps2_data(PS2_KBD_DAT),
    .uart1_rxd(UART1_RXD), 
    .uart1_txd(UART1_TXD),
    .uart2_rxd(uart_lb2_rx), 
    .uart2_txd(uart_lb2_tx),
    .uart3_rxd(uart_lb2_tx), 
    .uart3_txd(uart_lb2_rx)
    );

wire [31:0] wbs_flash_dat_o; 
wire        wbs_flash_ack_o; 
wire [31:0] wbs_flash_dat_i; 
wire                wbs_flash_we_i; 
wire  [3:0] wbs_flash_sel_i;
wire [18:0] wbs_flash_adr_i;
wire        wbs_flash_cyc_i; 
wire        wbs_flash_stb_i; 

// Instantiate the FLASH Memory Interface
wb_flash flash0 (
    .clk_i(PCI_CLK), 
    .nrst_i(NRST), 
    .wb_adr_i(wbs_flash_adr_i), 
    .wb_dat_o(wbs_flash_dat_o), 
    .wb_dat_i(wbs_flash_dat_i), 
    .wb_sel_i(wbs_flash_sel_i), 
    .wb_we_i(wbs_flash_we_i), 
    .wb_stb_i(wbs_flash_stb_i), 
    .wb_cyc_i(wbs_flash_cyc_i), 
    .wb_ack_o(wbs_flash_ack_o), 
    .flash_adr_o(flash_adr_o), 
    .flash_dat_o(flash_dat_o), 
    .flash_dat_i(flash_dat_i), 
    .flash_oe(flash_oe_o), 
    .flash_ce(flash_ce_o), 
    .flash_we(flash_we_o)
    );

wire [31:0] wbs_ddr_dat_o; 
wire        wbs_ddr_ack_o; 
wire [31:0] wbs_ddr_dat_i; 
wire        wbs_ddr_we_i; 
wire  [3:0] wbs_ddr_sel_i;
wire [19:0] wbs_ddr_adr_i;
wire        wbs_ddr_cyc_i; 
wire        wbs_ddr_stb_i; 

// Instantiate the DDR SDRAM Controller
// (This is not working properly at the moment... not sure why.)
wb_ddr #(
    .phase_shift(0),
    .clk_multiply(12), //15), //13),
    .clk_divide(3),
    .wait200_init(26)
) ddr0 (
    .clk(PCI_CLK), 
    .reset(RST), 
    .rot(rot), 
    .ddr_clk(SD_CK_P), 
    .ddr_clk_n(SD_CK_N), 
    .ddr_clk_fb(SD_CK_FB), 
    .ddr_ras_n(SD_RAS), 
    .ddr_cas_n(SD_CAS), 
    .ddr_we_n(SD_WE), 
    .ddr_cke(SD_CKE), 
    .ddr_cs_n(SD_CS), 
    .ddr_a(SD_A), 
    .ddr_ba(SD_BA), 
    .ddr_dq(SD_DQ), 
    .ddr_dqs(SD_DQS), 
    .ddr_dm(SD_DM), 
    .wb_adr_i({12'b0, wbs_ddr_adr_i}), 
    .wb_dat_i(wbs_ddr_dat_i), 
    .wb_dat_o(wbs_ddr_dat_o), 
    .wb_sel_i(wbs_ddr_sel_i), 
    .wb_cyc_i(wbs_ddr_cyc_i), 
    .wb_stb_i(wbs_ddr_stb_i), 
    .wb_we_i(wbs_ddr_we_i), 
    .wb_ack_o(wbs_ddr_ack_o) 
    );

wire [31:0] wbs_mmu_dat_o; 
wire        wbs_mmu_ack_o; 
wire [31:0] wbs_mmu_dat_i; 
wire        wbs_mmu_we_i; 
wire  [3:0] wbs_mmu_sel_i;
wire  [9:0] wbs_mmu_adr_i;
wire        wbs_mmu_cyc_i; 
wire        wbs_mmu_stb_i; 

wire [15:0] mmu_adr_i = wb_z80_adr_o[15:0];
wire [23:0] mmu_adr_o;

// Instantiate the Memory Management Unit
wb_mmu mmu0 (
    .clk_i(PCI_CLK), 
    .nrst_i(NRST), 
    .wbs_adr_i(wbs_mmu_adr_i), 
    .wbs_dat_o(wbs_mmu_dat_o), 
    .wbs_dat_i(wbs_mmu_dat_i), 
    .wbs_sel_i(wbs_mmu_sel_i), 
    .wbs_we_i(wbs_mmu_we_i), 
    .wbs_stb_i(wbs_mmu_stb_i), 
    .wbs_cyc_i(wbs_mmu_cyc_i), 
    .wbs_ack_o(wbs_mmu_ack_o), 
    .mmu_adr_i(mmu_adr_i), 
    .mmu_adr_o(mmu_adr_o)
    );

// Instantiate the Wishbone Backplane
intercon wb_intercon (
    .wb32_pci_master_dat_i(MDAT_I), 
    .wb32_pci_master_ack_i(ACK_I), 
    .wb32_pci_master_err_i(ERR_I), 
    .wb32_pci_master_dat_o(MDAT_O), 
    .wb32_pci_master_we_o(WE_O), 
    .wb32_pci_master_sel_o(SEL_O), 
    .wb32_pci_master_adr_o(ADR_O[23:0]), 
    .wb32_pci_master_cyc_o(CYC_O), 
    .wb32_pci_master_stb_o(STB_O),
    .wbm_z80_dat_i(wb_z80_dat_i), 
    .wbm_z80_ack_i(wb_z80_ack_i), 
    .wbm_z80_dat_o(wb_z80_dat_o), 
    .wbm_z80_we_o(wb_z80_we_o), 
    .wbm_z80_sel_o(wb_z80_sel_o), 
    .wbm_z80_adr_o((wb_z80_tga_o & 2'b01) ? {14'h0000,wb_z80_adr_o[7:0],2'b00} : mmu_adr_o),
    .wbm_z80_cyc_o(wb_z80_cyc_o), 
    .wbm_z80_stb_o(wb_z80_stb_o),
    .wb_cpu_ctrl_dat_o(wb_cpu_ctrl_dat_o), 
    .wb_cpu_ctrl_ack_o(wb_cpu_ctrl_ack_o), 
    .wb_cpu_ctrl_dat_i(wb_cpu_ctrl_dat_i), 
    .wb_cpu_ctrl_we_i(wb_cpu_ctrl_we_i), 
    .wb_cpu_ctrl_sel_i(wb_cpu_ctrl_sel_i), 
    .wb_cpu_ctrl_adr_i(wb_cpu_ctrl_adr_i), 
    .wb_cpu_ctrl_cyc_i(wb_cpu_ctrl_cyc_i), 
    .wb_cpu_ctrl_stb_i(wb_cpu_ctrl_stb_i),
`ifdef USE_INTERNAL_RAM
    .wb_sram_dat_o(wb_ram_dat_o), 
    .wb_sram_ack_o(wb_ram_ack_o), 
    .wb_sram_dat_i(wb_ram_dat_i), 
    .wb_sram_we_i(wb_ram_we_i), 
    .wb_sram_sel_i(wb_ram_sel_i), 
    .wb_sram_adr_i(wb_ram_adr_i), 
    .wb_sram_cyc_i(wb_ram_cyc_i), 
    .wb_sram_stb_i(wb_ram_stb_i), 
`endif // USE_INTERNAL_RAM
    .wb_uart0_dat_o(wb_uart0_dat_o), 
    .wb_uart0_ack_o(wb_uart0_ack_o), 
    .wb_uart0_dat_i(wb_uart0_dat_i), 
    .wb_uart0_we_i(wb_uart0_we_i), 
    .wb_uart0_sel_i(wb_uart0_sel_i), 
    .wb_uart0_adr_i(wb_uart0_adr_i), 
    .wb_uart0_cyc_i(wb_uart0_cyc_i), 
    .wb_uart0_stb_i(wb_uart0_stb_i),

    .wb_uart1_dat_o(wb_uart1_dat_o), 
    .wb_uart1_ack_o(wb_uart1_ack_o), 
    .wb_uart1_dat_i(wb_uart1_dat_i), 
    .wb_uart1_we_i(wb_uart1_we_i), 
    .wb_uart1_sel_i(wb_uart1_sel_i), 
    .wb_uart1_adr_i(wb_uart1_adr_i), 
    .wb_uart1_cyc_i(wb_uart1_cyc_i), 
    .wb_uart1_stb_i(wb_uart1_stb_i),

    .wbs_flash_dat_o(wbs_flash_dat_o), 
    .wbs_flash_ack_o(wbs_flash_ack_o), 
    .wbs_flash_dat_i(wbs_flash_dat_i), 
    .wbs_flash_we_i (wbs_flash_we_i), 
    .wbs_flash_sel_i(wbs_flash_sel_i), 
    .wbs_flash_adr_i(wbs_flash_adr_i), 
    .wbs_flash_cyc_i(wbs_flash_cyc_i), 
    .wbs_flash_stb_i(wbs_flash_stb_i), 

    .wbs_ddr_dat_o(wbs_ddr_dat_o), 
    .wbs_ddr_ack_o(wbs_ddr_ack_o), 
    .wbs_ddr_dat_i(wbs_ddr_dat_i), 
    .wbs_ddr_we_i (wbs_ddr_we_i), 
    .wbs_ddr_sel_i(wbs_ddr_sel_i), 
    .wbs_ddr_adr_i(wbs_ddr_adr_i), 
    .wbs_ddr_cyc_i(wbs_ddr_cyc_i), 
    .wbs_ddr_stb_i(wbs_ddr_stb_i), 

    .wbs_mmu_dat_o(wbs_mmu_dat_o), 
    .wbs_mmu_ack_o(wbs_mmu_ack_o), 
    .wbs_mmu_dat_i(wbs_mmu_dat_i), 
    .wbs_mmu_we_i (wbs_mmu_we_i), 
    .wbs_mmu_sel_i(wbs_mmu_sel_i), 
    .wbs_mmu_adr_i(wbs_mmu_adr_i), 
    .wbs_mmu_cyc_i(wbs_mmu_cyc_i), 
    .wbs_mmu_stb_i(wbs_mmu_stb_i), 

    .wbs_vga_dat_o(wbs_vga_dat_o), 
    .wbs_vga_ack_o(wbs_vga_ack_o), 
    .wbs_vga_dat_i(wbs_vga_dat_i), 
    .wbs_vga_we_i (wbs_vga_we_i), 
    .wbs_vga_sel_i(wbs_vga_sel_i), 
    .wbs_vga_adr_i(wbs_vga_adr_i), 
    .wbs_vga_cyc_i(wbs_vga_cyc_i), 
    .wbs_vga_stb_i(wbs_vga_stb_i), 

    .clk(PCI_CLK), 
    .reset(RST)
    );

endmodule
