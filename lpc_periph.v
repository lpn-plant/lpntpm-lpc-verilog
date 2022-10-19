// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Copyright (C) 2008 Howard M. Harte <hharte@opencores.org>
// Copyright (C) 2021 LPN Plant
//
// This source file may be used and distributed without
// restriction provided that this copyright statement is not
// removed from the file and that any derivative work contains
// the original copyright notice and the associated disclaimer.
//
// This source file is free software; you can redistribute it
// and/or modify it under the terms of the GNU Lesser General
// Public License as published by the Free Software Foundation;
// either version 2.1 of the License, or (at your option) any
// later version.
//
// This source is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
// PURPOSE.  See the GNU Lesser General Public License for more
// details.
//
// You should have received a copy of the GNU Lesser General
// Public License along with this source; if not, download it
// from http://www.opencores.org/lgpl.shtml

`timescale 1 ns / 1 ps

`include "lpc_defines.v"

module lpc_periph (clk_i, nrst_i, lframe_i, lad_bus, addr_hit_i, current_state_o,
                   din_i, lpc_data_in_o, lpc_data_out_o, lpc_addr_o, lpc_en_o,
                   io_rden_sm_o, io_wren_sm_o, TDATA, READY
);

    // Master Interface
    input  wire        clk_i; // LPC clock
    input  wire        nrst_i; // LPC rese (active low)

    // LPC Slave Interface
    input  wire        lframe_i; // LPC frame input (active low)
    inout  wire [ 3:0] lad_bus; // LPC data bus

    // Helper signals
    input  wire        addr_hit_i;
    output reg  [ 4:0] current_state_o;  //Current peripheral state (FSM)
    input  wire [ 7:0] din_i;            //Data sent when host requests a read
    output reg  [ 7:0] lpc_data_in_o;    //Data received by peripheral for writing
    output wire [ 3:0] lpc_data_out_o;   //Data sent to host when a read is requested
    output wire [15:0] lpc_addr_o;       //16-bit LPC Peripheral Address
    output wire        lpc_en_o;         //Active-high status signal indicating the peripheral is ready for next operation.
    output wire        io_rden_sm_o;     //Active-high read status
    output wire        io_wren_sm_o;     //Active-high write status
    output reg  [31:0] TDATA;            //32-bit register with LPC cycle: Address, Data(8-bit) and type of opertion
    output reg         READY;            //Active-high status signal indicating that new cycle data is on TDATA

    // Internal signals
    reg         sync_en;
    reg   [3:0] rd_addr_en;
    reg   [3:0] lad;
    wire  [1:0] wr_data_en;
    reg   [1:0] rd_data_en;
    reg         tar_F;
    reg  [15:0] lpc_addr_o_reg;   //16-bit internal LPC address register

    reg   [4:0] fsm_next_state;   //State: next state of FSM
    reg   [4:0] previous_state;   //State: previous state of FSM

    reg   [1:0] cycle_type = 2'b00; //"00" none, "01" write, "11" read
    integer cycle_cnt = 0;          //auxiliary clock periods counter
    reg  [31:0] dinAbuf = 32'b00000000000000000000000000000000; //32-bit register buffer for LPC cycle data (encoded)

    reg  [31:0] memoryLPC [0:2]; //memory array 2x32bit
    reg wasLframeLow = 1'b0;     //indicates that new LPC cycle started
    reg wasLpc_enHigh = 1'b0;    //indicates that all current cycle data is ready
    reg newValuedata = 1'b0;     //indicates that data on TDATA had been changed
    reg skipCycle;               // 1 -indicates that this cycle is not I/O or TPM cycle, 0 - indicates I/O or TPM cycle
    
    assign lpc_addr_o = lpc_addr_o_reg;

//    always @ (negedge nrst_i) begin
//        current_state_o <= `LPC_ST_IDLE;
//        tar_F <= 1'b0;
//        sync_en <= 1'b0;
//        rd_data_en <= 2'b00;
//    end
    
//       always @ (negedge lframe_i) begin
//        current_state_o <= `LPC_ST_START;
//        tar_F <= 1'b0;
//        sync_en <= 1'b0;
//        rd_data_en <= 2'b00;
//        lad <= `LPC_STOP;    // abort if there is no posedge clk_i while lframe_i is low
//    end

//    always @ (posedge lframe_i) begin
//        if (lad == 4'h0) current_state_o <= `LPC_ST_CYCTYPE;
//        else current_state_o <= `LPC_ST_IDLE;
//    end


    always @ (posedge clk_i) begin
	lad <= lad_bus;
	if (nrst_i==1'b0)
	begin
	  current_state_o <= `LPC_ST_IDLE;
	  tar_F <= 1'b0;
	  sync_en <= 1'b0;
      rd_data_en <= 2'b00;
	end
	 if (lframe_i==1'b0) 
	 begin
	     current_state_o <= `LPC_ST_START;
        tar_F <= 1'b0;
        sync_en <= 1'b0;
        rd_data_en <= 2'b00;
        lad <= `LPC_STOP;    // abort if there is no posedge clk_i while lframe_i is low
	 end
	 else if (lframe_i==1'b1)
	 begin
	    if (lad == 4'h0) current_state_o <= `LPC_ST_CYCTYPE;
        else current_state_o <= `LPC_ST_IDLE;
	 end
        case(current_state_o)
              `LPC_ST_CYCTYPE:
               begin
                   if (lad_bus == 4'h0) current_state_o <= `LPC_ST_CYCTYPE_RD;
                   else if (lad_bus == 4'h2) current_state_o <= `LPC_ST_CYCTYPE_WR;
                   else current_state_o <= `LPC_ST_IDLE;
               end
              `LPC_ST_CYCTYPE_RD:
               current_state_o <= `LPC_ST_ADDR_RD_CLK1;
              `LPC_ST_ADDR_RD_CLK1:
               current_state_o <= `LPC_ST_ADDR_RD_CLK2;
              `LPC_ST_ADDR_RD_CLK2:
               current_state_o <= `LPC_ST_ADDR_RD_CLK3;
              `LPC_ST_ADDR_RD_CLK3:
               current_state_o <= `LPC_ST_ADDR_RD_CLK4;
              `LPC_ST_ADDR_RD_CLK4:
               begin
                   current_state_o <= `LPC_ST_TAR_RD_CLK1;
                   tar_F <= 1'b1;
               end
              `LPC_ST_TAR_RD_CLK1:
               begin
                   current_state_o <= `LPC_ST_TAR_RD_CLK2;
                   tar_F <= 1'b0;
               end
              `LPC_ST_TAR_RD_CLK2:
               begin
                   if (addr_hit_i == 1'b0) current_state_o <= `LPC_ST_IDLE;
                   if (addr_hit_i == 1'b1)
                   begin
                       current_state_o <= `LPC_ST_SYNC_RD;
                       sync_en <= 1'b1;
                   end
               end
              `LPC_ST_SYNC_RD:
               begin
                   current_state_o <= `LPC_ST_DATA_RD_CLK1;
                   sync_en <= 1'b0;
                   rd_data_en <= 2'b01;
               end
              `LPC_ST_DATA_RD_CLK1:
               begin
                   current_state_o <= `LPC_ST_DATA_RD_CLK2;
                   rd_data_en <= 2'b10;
               end
              `LPC_ST_DATA_RD_CLK2:
               begin
                   current_state_o <= `LPC_ST_FINAL_TAR_CLK1;
                   tar_F <= 1'b1;
                   rd_data_en <= 2'b00;
               end
              `LPC_ST_CYCTYPE_WR:
               current_state_o <= `LPC_ST_ADDR_WR_CLK1;
              `LPC_ST_ADDR_WR_CLK1:
               current_state_o <= `LPC_ST_ADDR_WR_CLK2;
              `LPC_ST_ADDR_WR_CLK2:
               current_state_o <= `LPC_ST_ADDR_WR_CLK3;
              `LPC_ST_ADDR_WR_CLK3:
               current_state_o <= `LPC_ST_ADDR_WR_CLK4;
              `LPC_ST_ADDR_WR_CLK4:
               current_state_o <= `LPC_ST_DATA_WR_CLK1;
              `LPC_ST_DATA_WR_CLK1:
               current_state_o <= `LPC_ST_DATA_WR_CLK2;
              `LPC_ST_DATA_WR_CLK2:
               current_state_o <= `LPC_ST_TAR_WR_CLK1;
              `LPC_ST_TAR_WR_CLK1:
               current_state_o <= `LPC_ST_TAR_WR_CLK2;
              `LPC_ST_TAR_WR_CLK2:
               begin
                   if (addr_hit_i == 1'b0) current_state_o <= `LPC_ST_IDLE;
                   if (addr_hit_i == 1'b1) current_state_o <= `LPC_ST_SYNC_WR;
                   sync_en <= 1'b1;
               end
              `LPC_ST_SYNC_WR:
               begin
                   current_state_o <= `LPC_ST_FINAL_TAR_CLK1;
                   sync_en <= 1'b0;
                   tar_F <= 1'b1;
               end
              `LPC_ST_FINAL_TAR_CLK1:
               begin
                   current_state_o <= `LPC_ST_FINAL_TAR_CLK2;
                   tar_F <= 1'b0;
               end
              `LPC_ST_FINAL_TAR_CLK2:
               begin
                   current_state_o <= `LPC_ST_IDLE;
               end
              default: begin end
        endcase
    end

 
    assign lad_bus = (sync_en == 1'b1) ? `LPC_SYNC_READY :
                     (tar_F == 1'b1 ) ? 4'hF :
                     (rd_data_en == 2'b01) ? din_i[3:0] :
                     (rd_data_en == 2'b10) ? din_i[7:4] :
                     4'hZ;
endmodule