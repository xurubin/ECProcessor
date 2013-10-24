`include "defines.v"
/*
|ecurve_add:ecp|	4521 (267)	3016 (225)	0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp	work
|processor:proc|	4254 (0)		2791 (0)		0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp|processor:proc	work
|processor_plai|	4254 (1498)	2791 (1130)	0	8	0	4	0	0	|DE2_FPGA|ecurve_add:ecp|processor:proc|processor_plain:pp	work
*/
module processor_selftest_rom(input [7:0] addr, output reg [7:0] code );
always @(*) begin
			   //00AAABBB: Load Operands - op1 <= AAA, op2 <= BBB
			   //01AAABBB: Arithmetic Op - AAA:000=sub,001=mul,010=inv,011=mov,100=tstZero	BBB:dest_register
			   //11AAAAAA: jnz
			   //10AAAAAA: jz
			   //11000000: terminate successful
			   //10000000: terminate unsuccessful
	case (addr)// 8'bAABBBCCC
	8'd0: code <= 8'b00010011; //Op <= (reg2, reg3)
	8'd1: code <= 8'b01000100; //reg4 <= reg2-reg3;
	8'd2: code <= 8'b01001101; //reg5 <= reg2*reg3;
	8'd3: code <= 8'b01010110; //reg6 <= reg2^(-1);
	8'd4: code <= 8'b01011111; //reg7 <= reg2;
	8'd5: code <= 8'b11000000; //terminate
	default: code <= 8'bx;
	endcase
end	
endmodule

module processor(	input clk,
					input rst,
					input [`R_Bits-1:0] data_0,
					input [`R_Bits-1:0] data_1,
					input [`R_Bits-1:0] data_2,
					input [`R_Bits-1:0] data_3,
					input pi,
					input qi,
					output [7:0] instr_addr,
					input  [7:0] instruction,
					output done,
					output [`R_Bits-1:0] result_0,
					output [`R_Bits-1:0] result_1,
					output succeed
				);

processor_plain pp(.clk(clk), .rst(rst), .done(done), .succeed(succeed),
				.data_0(data_0), .data_1(data_1), .data_2(data_2), .data_3(data_3),
				.pi(pi), .qi(qi),
				.instr_addr(instr_addr), .instruction(instruction),
				.result_0(result_0), .result_1(result_1));

/*
processor_ram pr(.clk(clk), .rst(rst), .done(done), .succeed(succeed),
				.data_0(data_0), .data_1(data_1), .data_2(data_2), .data_3(data_3),
				.instr_addr(instr_addr), .instruction(instruction),
				.result_0(result_0), .result_1(result_1));
*/
/*
processor_sr ps(.clk(clk), .rst(rst), .done(done), .succeed(succeed),
				.data_0(data_0), .data_1(data_1), .data_2(data_2), .data_3(data_3),
				.instr_addr(instr_addr), .instruction(instruction),
				.result_0(result_0), .result_1(result_1));
*/
endmodule

module processor_plain(	input clk,
					input rst,
					input [`R_Bits-1:0] data_0,
					input [`R_Bits-1:0] data_1,
					input [`R_Bits-1:0] data_2,
					input [`R_Bits-1:0] data_3,
					input pi,
					input qi,
					output reg [7:0] instr_addr,
					input  [7:0] instruction,
					output done,
					output [`R_Bits-1:0] result_0,
					output [`R_Bits-1:0] result_1,
					output reg succeed
				);

///////////////////////////Register files ////////////////////////////////////////				
///R6,R7 : Readonly Constants 
reg [`R_Bits-1:0] reg_0,reg_1,reg_2,reg_3,reg_4,reg_5;//,reg_6,reg_7;
reg ZF;
///////////////////////////State Machine /////////////////////////////////////////
parameter state_idle = 0;
parameter state_init = 1;
parameter state_decode = 2;
parameter state_load_op = 3; //Not used
parameter state_arith_op = 4;
parameter state_done = 5;
reg [2:0] state;

//////////////////////////Operand Selector ////////////////////////////////////////
reg [2:0] op1_sel, op2_sel, reg_sel;
reg [`R_Bits-1:0] op1, op2, selectedReg;
reg op1done;
always @(*) begin
	case (reg_sel)
		3'd0: selectedReg <= reg_0;		3'd1: selectedReg <= reg_1;		3'd2: selectedReg <= reg_2;
		3'd3: selectedReg <= reg_3;		3'd4: selectedReg <= reg_4;		3'd5: selectedReg <= reg_5;
		3'd6: selectedReg <= (`Modulus-`EC_A);
		3'd7: selectedReg <= 0;
	endcase
end

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
assign result_0 = reg_0;
assign result_1 = reg_1;

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
parameter op_null1 = 3'd5;
parameter op_null2 = 3'd6;
parameter op_null3 = 3'd7;

always @(*) begin
	instr_v1 <= instruction[2:0];
	instr_v0 <= instruction[5:3];
	instr_type <= instruction[7:6];
	jmp_offset <= instruction[5:0];
	// Arithmetic Op decoding
	AU_sel <= instr_v0;
	AU_dest_reg <= instr_v1;
	case (AU_sel[1:0])
	
		op_sub : AU_result <= AU_sub_r;
		op_inv : AU_result <= AU_inv_r;
		op_mul : AU_result <= AU_mul_r;
		op_mov : AU_result <= AU_mov_r;
	endcase
end

//////////////////////////Main state Machine ///////////////////////////////////////	
always @(posedge clk or posedge rst) begin
	if (rst) begin
		reg_0 <= 110'bx;reg_1 <= 110'bx;reg_2 <= 110'bx;reg_3 <= 110'bx;reg_4 <= 110'bx;reg_5 <= 110'bx;
		ZF <= 1'b1;
		instr_addr <= 7'b0;
		state <= state_init;
		AU_sub_rst <= 1'b1; AU_mul_rst <= 1'b1; AU_inv_rst <= 1'b1; AU_mov_rst <= 1'b1;
		succeed <= 1'b0;
	end else begin
		case (state)
			state_init: begin // Processor Initialisation
				reg_0 <= data_0; reg_1 <= data_1; 
				reg_2 <= data_2; reg_3 <= data_3;
				reg_4 <= pi; reg_5 <= qi;
				state <= state_decode;
			end
			state_load_op: begin //Load Operands before executing Arith Ops
				if(op1done == 1'b0) begin
					op1 <= selectedReg;
					op1done <= 1'b1;
					reg_sel <= op2_sel;
				end else begin //Done Loading Operands, proceed with arith Ops
					op2 <= selectedReg;
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
				 case (instr_type)
					instr_load_ops: begin //Load Operands
						op1_sel <= instr_v0;
						op2_sel <= instr_v1;
						instr_addr <= instr_addr + 1'b1;
					end 
					instr_arith: begin  //Begin Arithmetic Ops
						state <= state_load_op; //Load Operands First
						op1done <= 1'b0;
						if (AU_sel == op_tstz)
							reg_sel <= AU_dest_reg;
						else
							reg_sel <= op1_sel;
					end
					instr_jz, instr_jnz: begin //Jump Instruction
						if (jmp_offset == 6'b0) begin
							state <= state_done;
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
			state_arith_op:begin
				if ((AU_sel == op_sub && AU_sub_done) ||
					(AU_sel == op_mul && AU_mul_done) ||
					(AU_sel == op_inv && AU_inv_done) ||
					(AU_sel == op_mov && AU_mov_done)) begin
						case (AU_dest_reg) 
								0: reg_0 <= AU_result;1: reg_1 <= AU_result;2: reg_2 <= AU_result;
								3: reg_3 <= AU_result;4: reg_4 <= AU_result;5: reg_5 <= AU_result;
								default:$display("ERROR: Write to readonly Registers");
						endcase
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
				end
				case (AU_sel)
					op_sub: if (AU_sub_done) begin
								AU_sub_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_mul: if (AU_mul_done) begin
								AU_mul_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_inv: if (AU_inv_done) begin
								AU_inv_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_mov: if (AU_mov_done) begin
								AU_mov_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_tstz:if (AU_tstz_done) begin
								ZF <= AU_tstz_r;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
				endcase
			end //end state_arith_op
			state_done:;//state_done
		endcase	
	end
end

endmodule


