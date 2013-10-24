`include "defines.v"
/*
|ecurve_add:ecp|	4789 (269)	3239 (225)	0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp	work
|processor:proc|	4520 (0)		3014 (0)		0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp|processor:proc	work
|processor_sr |	4520 (1763)	3014 (1353)	0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp|processor:proc|processor_sr:ps	work
*/
module processor_sr(input clk,
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
reg [2:0] op1_sel, op2_sel;
reg [`R_Bits-1:0] op1, op2;
reg op1done;

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
///////////////////////////Register files ////////////////////////////////////////				
reg [`R_Bits-1:0] SRA,SRB,SRC,SRD,SRE,SRF,SRG,SRH;
reg [2:0] sr_id;
reg ZF;

reg sr_we;
wire [`R_Bits-1:0] sr_out;
assign sr_out = SRH;
always @(posedge clk or posedge rst)
	if (rst) begin
		sr_id <= 3'b000;
		SRA <= 0; SRB <= (`Modulus-`EC_A);
		SRH <= data_0;
		SRG <= data_1;
		SRF <= data_2;
		SRE <= data_3;
	end else begin	//A=>B=>C=>D=>E=>F=>G=>H 
		SRA <= SRH; SRC <= SRB; SRD <= SRC; SRE <= SRD; SRF <= SRE; SRG <= SRF; SRH <= SRG;
		sr_id <= sr_id + 1'b1;
		if (sr_we)
			SRB <= AU_result;
		else
			SRB <= SRA;
	end

//////////////////////////Main state Machine ///////////////////////////////////////	
always @(posedge clk or posedge rst) begin
	if (rst) begin
		ZF <= 1'b1;
		instr_addr <= 7'b0;
		state <= state_init;
		AU_sub_rst <= 1'b1; AU_mul_rst <= 1'b1; AU_inv_rst <= 1'b1; AU_mov_rst <= 1'b1;
		succeed <= 1'b0;
	end else begin
		case (state)
			state_init: begin // Processor Initialisation
					state <= state_decode;
			end
			state_load_op: begin //Load Operands before executing Arith Ops
				if((op1done == 1'b0)&&(sr_id == op1_sel)) begin
					op1 <= sr_out;
					op1done <= 1'b1;
				end
				if((op1done == 1'b1)&&(sr_id == op2_sel)) begin //Done Loading Operands, proceed with arith Ops
					op2 <= sr_out;
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
				 sr_we <= 1'b0;
				 case (instr_type)
					instr_load_ops: begin //Load Operands
						op1_sel <= instr_v0;
						op2_sel <= instr_v1;
						instr_addr <= instr_addr + 1'b1;
					end 
					instr_arith: begin  //Begin Arithmetic Ops
						state <= state_load_op; //Load Operands First
						op1done <= 1'b0;
					end
					instr_jz, instr_jnz: begin //Jump Instruction
						if (jmp_offset == 6'b0) begin
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
			      sr_we <= 1'b0;
			      state <= state_decode;
				   AU_sub_rst <= 1'b1;
					AU_mul_rst <= 1'b1;
					AU_mov_rst <= 1'b1;
					AU_inv_rst <= 1'b1;
			      instr_addr <= instr_addr + 1'b1;
			end
			state_arith_op:begin
				if( ((AU_sel == op_sub && AU_sub_done) ||
					(AU_sel == op_mul && AU_mul_done) ||
					(AU_sel == op_inv && AU_inv_done) ||
					(AU_sel == op_mov && AU_mov_done))&&(sr_id == AU_dest_reg) ) begin
						sr_we <= 1'b1;
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
				if (sr_id == 3'b000) begin
					result_0 <= sr_out;
				end
				if (sr_id == 3'b001) begin
					result_1 <= sr_out;
					state<= state_done;
				end
			end//state_pre_done
			state_done:;//state_done
		endcase	
	end
end

endmodule
