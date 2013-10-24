`include "defines.v"
/*
|ecurve_add:ecp|	4377 (270)	3349 (225)	0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp	work
|processor:proc|	4107 (0)		3124 (0)		0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp|processor:proc	work
|processor_ram |	4107 (802)	3124 (583)	0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp|processor:proc|processor_ram:pr	work
*/
// Quartus II Verilog Template
// Single port RAM with single read/write address 
module single_port_ram 
(
	input [(`R_Bits-1):0] data,
	input [(3-1):0] addr,
	input we, clk,
	output [(`R_Bits-1):0] q
);

	// Declare the RAM variable
	reg [`R_Bits-1:0] ram[2**3-1:0];

	// Variable to hold the registered read address
	reg [3-1:0] addr_reg;

	always @ (posedge clk)
	begin
		// Write
		if (we)
			ram[addr] = data;

		addr_reg <= addr;
	end

	// Continuous assignment implies read returns NEW data.
	// This is the natural behavior of the TriMatrix memory
	// blocks in Single Port mode
	assign q = ram[addr];
endmodule


module processor_ram(input clk,
					input rst,
					input [`R_Bits-1:0] data_0,
					input [`R_Bits-1:0] data_1,
					input [`R_Bits-1:0] data_2,
					input [`R_Bits-1:0] data_3,
					output reg [7:0] instr_addr,
					input  [7:0] instruction,
					output done,
					output reg [`R_Bits-1:0] result_0,
					output reg [`R_Bits-1:0] result_1,
					output reg succeed
				);

///////////////////////////Register files ////////////////////////////////////////				
reg regs_we;
wire [`R_Bits-1:0] regs_out;
reg [`R_Bits-1:0] regs_in;
reg [2:0] regs_sel, op1_sel, op2_sel;
single_port_ram RegFile(.clk(clk), .data(regs_in), .addr(regs_sel), .we(regs_we), .q(regs_out) );
reg ZF;
reg [`R_Bits-1:0] op1, op2;
reg op1done;
///////////////////////////State Machine /////////////////////////////////////////
parameter state_idle = 0;
parameter state_init = 1;
parameter state_decode = 2;
parameter state_load_op = 3;
parameter state_post_arith_op = 7;
parameter state_arith_op = 4;
parameter state_pre_done = 5;
parameter state_done = 6;
reg [2:0] state;

/////////////////////////Arithmatic Units Instantiation ////////////////////////////
	/// AU_sub (Subtractor)
wire [`R_Bits-1:0] AU_sub_r;
reg AU_sub_rst;
reg AU_sub_done;
subtract_p_p AU_sub(.clk(clk), .a(op1), .b(op2), .r(AU_sub_r));
reg AU_sub_t;
always @(posedge clk or posedge AU_sub_rst) begin   //Generate done signals for sub. (2 clk cycles delay)
	if (AU_sub_rst) begin 
		AU_sub_t <= 1'b0; AU_sub_done <= 1'b0; 
	end else begin 
		AU_sub_t <= 1'b1; AU_sub_done <= AU_sub_t; 
	end
end

	/// AU_Mul (Montgomery Multiplier)
wire [`R_Bits-1:0] AU_mul_r;
reg AU_mul_rst;
wire AU_mul_done;
mont_mul_p AU_mul(.clk(clk), .reset(AU_mul_rst), .a(op1), .b(op2), .r(AU_mul_r), .done(AU_mul_done));

	//AU_Inv (Montgomery Inverter)
wire [`R_Bits-1:0] AU_inv_r;
reg AU_inv_rst;
wire AU_inv_done;
mont_invert_p AU_inv(.clk(clk), .reset(AU_inv_rst), .a(op1), .result(AU_inv_r), .done(AU_inv_done));

	//AU_Mov (Register Move Op)
wire [`R_Bits-1:0] AU_mov_r;
reg AU_mov_rst;
wire AU_mov_done;
assign AU_mov_r = op1;
assign AU_mov_done = 1'b1;

	//AU_TestZero (Register Test Zero)
wire AU_tstz_r;
reg AU_tstz_rst;
wire AU_tstz_done;
assign AU_tstz_r = ~(|op1);
assign AU_tstz_done = 1'b1;


/////////////////////////// Processor Connections /////////////////////////////////
assign done = (state == state_done);

/////////////////////////// Instruction Decoding /////////////////////////////////
reg [1:0] instr_type;
reg [2:0] instr_v0, instr_v1;
reg [2:0] AU_dest_reg, AU_sel;
reg [`R_Bits-1:0] AU_result;
reg [5:0] jmp_offset;
parameter instr_load_ops = 2'b00;
parameter instr_arith   = 2'b01;
parameter instr_jz		= 2'b10;
parameter instr_jnz		= 2'b11;

parameter op_sub  = 3'd0;
parameter op_mul  = 3'd1;
parameter op_inv  = 3'd2;
parameter op_mov  = 3'd3;
parameter op_tstz = 3'd4;

always @(*) begin
	instr_v1 <= instruction[2:0];
	instr_v0 <= instruction[5:3];
	instr_type <= instruction[7:6];
	jmp_offset <= instruction[5:0];
	// Arithmetic Op decoding
	AU_sel <= instr_v0;
	AU_dest_reg <= instr_v1;
	AU_result <= (AU_sel == op_sub) ? AU_sub_r
				:(AU_sel == op_mul) ? AU_mul_r
				:(AU_sel == op_inv) ? AU_inv_r
				: AU_mov_r;
end

//////////////////////////Main state Machine ///////////////////////////////////////	
always @(posedge clk or posedge rst) begin
	if (rst) begin
		ZF <= 1'b1;
		instr_addr <= 7'b0;
		state <= state_init;
		AU_sub_rst <= 1'b1; AU_mul_rst <= 1'b1; AU_inv_rst <= 1'b1; AU_mov_rst <= 1'b1;
		succeed <= 1'b0;
		regs_sel <= 3'b111;
	end else begin
		case (state)
			state_init: begin // Processor Initialisation
				regs_sel <= regs_sel+1'b1;
				regs_we <= 1'b1;
				case (regs_sel)
				7: regs_in <= data_0;
				0: regs_in <= data_1;
				1: regs_in <= data_2;
				2: regs_in <= data_3;
				3:;
				4:;
				5: regs_in <= (`Modulus-`EC_A);
				6:	begin
						regs_in <= 0;
						state <= state_decode;
					end
				endcase
			end
			state_load_op: begin //Load Operands before executing Arith Ops
				if(op1done == 1'b0) begin
					op1 <= regs_out;
					op1done <= 1'b1;
					regs_sel <= op2_sel;
				end else begin //Done Loading Operands, proceed with arith Ops
					op2 <= regs_out;
					state <= state_arith_op;
					case (AU_sel)
						op_sub: AU_sub_rst <= 1'b0; 
						op_mul: AU_mul_rst <= 1'b0;
						op_inv: AU_inv_rst <= 1'b0;
						op_mov: AU_mov_rst <= 1'b0;
					endcase
				end
			end
			state_decode: begin // Instruction Decode stage
				//instr_addr <= instr_addr + 1'b1;
				 regs_we <= 1'b0;
				 case (instr_type)
					instr_load_ops: begin //Load Operands
						op1_sel <= instr_v0;
						op2_sel <= instr_v1;
						instr_addr <= instr_addr + 1'b1;
					end 
					instr_arith: begin  //Begin Arithmetic Ops
						state <= state_load_op; //Load Operands First
						op1done <= 1'b0;
						regs_sel <= op1_sel;
					end
					instr_jz, instr_jnz: begin //Jump Instruction
						if (jmp_offset == 6'b0) begin
							regs_sel <= 3'b0;
							result_0 <= regs_out;
							state <= state_pre_done;
							succeed <= (instr_type[0]);
						end	else if(( (ZF == 1'b1)&&(instr_type ==  instr_jz) ) || 
								( (ZF == 1'b0)&&(instr_type == instr_jnz) )) begin
							instr_addr <= instr_addr + jmp_offset;
						end	else
							instr_addr <= instr_addr + 1'b1;
					end
					default:;
				endcase
			end //end state_docode
			state_post_arith_op: begin
			      regs_we <= 1'b0;
			      state <= state_decode;
				   AU_sub_rst <= 1'b1;
					AU_mul_rst <= 1'b1;
					AU_mov_rst <= 1'b1;
					AU_inv_rst <= 1'b1;
			      instr_addr <= instr_addr + 1'b1;
			end
			state_arith_op:begin
				regs_sel <= AU_dest_reg;
				if ((AU_sel == op_sub && AU_sub_done) ||
					(AU_sel == op_mul && AU_mul_done) ||
					(AU_sel == op_inv && AU_inv_done) ||
					(AU_sel == op_mov && AU_mov_done)) begin
						regs_we <= 1'b1;
						regs_in <= AU_result;
				      state <= state_post_arith_op;
/*
				$display("EIP:", instr_addr);
				$display("Register Files(Not Mont Domain): ");
				$display("%28x", reg_0*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_1*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_2*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_3*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_4*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_5*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_6*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
				$display("%28x", reg_7*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);
*/
				end else if ((AU_sel == op_tstz) && (AU_tstz_done)) begin
								ZF <= AU_tstz_r;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
				end

			end //end state_arith_op
			state_pre_done:begin
				regs_sel <= 3'b1;
				result_1 <= regs_out;
				state<= state_done;
			end//state_pre_done
			state_done:;//state_done
		endcase	
	end
end

endmodule
