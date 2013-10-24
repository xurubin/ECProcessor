`include "defines.v"

module processor_advanced(	input clk,
					input rst,
					
					input [`R_Bits-1:0] rom_data,
					output reg [7:0] rom_addr,
					output reg rom_rqst,
					input rom_rdy,
					input random_bit,
					
					output reg [7:0] instr_addr,
					input  [10:0] instruction,
					
					
					output [`R_Bits-1:0] result_0,
					output [`R_Bits-1:0] result_1,
					output reg outputing,
					input  output_ack
				);

///////////////////////////Register files ////////////////////////////////////////				
reg [3:0] op1_sel,op2_sel;
wire [`R_Bits-1:0] op1,op2;
reg [`R_Bits-1:0] reg_wdata;
reg reg_we;
regfile RegFile(.clock(clk), .data_a(reg_wdata), .data_b(), 
		.address_a(op1_sel), .address_b(op2_sel),
		.wren_a(reg_we), .wren_b(1'b0),
		.q_a(op1), .q_b(op2));
/*
true_dual_port_ram_single_clock RegFile(.clk(clk), .data_a(reg_wdata), .data_b(), 
										.addr_a(op1_sel), .addr_b(op2_sel),
										.we_a(reg_we), .we_b(1'b0),
										.q_a(op1), .q_b(op2));
*/
reg ZF,IF;
						
///////////////////////////State Machine /////////////////////////////////////////
parameter state_idle = 0;
parameter state_init = 1;
parameter state_decode = 2;
parameter state_arith_op_delay = 3;
parameter state_arith_op_delay_2 = 5;
parameter state_arith_op = 4;
reg [2:0] state;

/////////////////////////Arithmatic Units Instantiation ////////////////////////////
	/// AU_sub (Subtractor)
wire [`R_Bits-1:0] AU_sub_r;
reg AU_sub_rst;
reg AU_sub_done;
reg AU_sub_am;
//subtract_p_p AU_sub(.clk(clk), .a(op1), .b(op2), .r(AU_sub_r));
DualModulusSubtractor AU_sub(.alternativeModulus(AU_sub_am), .clk(clk), .a(op1), .b(op2), .r(AU_sub_r));
reg AU_sub_t;
always @(posedge clk or posedge AU_sub_rst) begin   //Generate done signals for sub. (2 clk cycles delay)
	if (AU_sub_rst) begin 
		AU_sub_t <= 1'b0; AU_sub_done <= 1'b0; 
	end else begin 
		AU_sub_t <= 1'b1; AU_sub_done <= AU_sub_t; 
	end
end

	/// AU_addModOrder (Adder)
/*wire [`R_Bits-1:0] AU_amo_r;
reg AU_amo_rst;
reg AU_amo_done;
add_Mod_Order AU_amo(.clk(clk), .a(op1), .b(op2), .r(AU_amo_r));
reg AU_amo_t;
always @(posedge clk or posedge AU_amo_rst) begin   //Generate done signals for sub. (2 clk cycles delay)
	if (AU_amo_rst) begin 
		AU_amo_t <= 1'b0; AU_amo_done <= 1'b0; 
	end else begin 
		AU_amo_t <= 1'b1; AU_amo_done <= AU_amo_t; 
	end
end
*/
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

	//AU_TestDP (Test DP Point Property)
wire AU_tstdp_r;
reg AU_tstdp_rst;
wire AU_tstdp_done;
assign AU_tstdp_r = ~(|op1[`Mask_Bit_Len-1:0]);
assign AU_tstdp_done = 1'b1;

	//AU_MemRd (Load External Memory Content)
wire [`R_Bits-1:0] AU_memrd_r;
reg AU_memrd_rst;
wire AU_memrd_done;

reg [1:0] AU_memrd_state;
reg prev_rom_rdy;
parameter AU_memrd_start = 0;
parameter AU_memrd_wait = 1;
assign AU_memrd_r = rom_data;
assign AU_memrd_done = (rom_rqst <= 1'b0)&&(AU_memrd_state == AU_memrd_wait);

always @(posedge clk or posedge AU_memrd_rst) 
if (AU_memrd_rst) begin
	rom_rqst <= 1'b0;
	AU_memrd_state <= AU_memrd_start;
	prev_rom_rdy <= 1'b0;
end else begin 
  prev_rom_rdy <= rom_rdy;
  case (AU_memrd_state)
	AU_memrd_start: begin
		rom_rqst <= 1'b1;
		AU_memrd_state <= AU_memrd_wait;
	end
	AU_memrd_wait: if (prev_rom_rdy != rom_rdy) rom_rqst <= 1'b0;
  endcase
end

	//AU_Outputing (Output Distinguished Point)
reg AU_outputing_rst;
wire AU_outputing_done;
reg [1:0] AU_outputing_state;
reg prev_output_ack;
parameter AU_outputing_start = 0;
parameter AU_outputing_wait = 1;
assign AU_outputing_done =((outputing == 1'b0)&&(AU_outputing_state == AU_outputing_wait));

always @(posedge clk or posedge AU_outputing_rst) 
if (AU_outputing_rst) begin
	outputing <= 1'b0;
	AU_outputing_state <= AU_outputing_start;
	prev_output_ack <= 1'b0;
end else begin 
  prev_output_ack <= output_ack;
  case (AU_outputing_state)
	AU_outputing_start: begin
		outputing <= 1'b1;
		AU_outputing_state <= AU_outputing_wait;
	end
	AU_outputing_wait: if (prev_output_ack!=output_ack) outputing <= 1'b0;
  endcase
end

/////////////////////////// Processor Connections /////////////////////////////////
assign result_0 = op1;
assign result_1 = op2;

/////////////////////////// Instruction Decoding /////////////////////////////////
reg [2:0] instr_type;
reg [3:0] instr_v0, instr_v1;
reg [7:0] instr_immediate;
reg [7:0] jmp_offset;
reg [3:0] instr_flag_target;

reg [3:0] AU_dest_reg, AU_sel;
reg flag_src;
reg [`R_Bits-1:0] AU_result;

parameter instr_ldo		= 3'b000;
parameter instr_arith   = 3'b001;
parameter instr_jz		= 3'b010;
parameter instr_jnz		= 3'b011;
parameter instr_flag	= 3'b100;
parameter instr_adrge   = 3'b101;
parameter instr_adrinc	= 3'b110;
parameter instr_null	= 3'b111;

parameter op_sub	= 4'd0;
parameter op_mul  	= 4'd1;
parameter op_inv  	= 4'd2;
parameter op_mov  	= 4'd3;
parameter op_tstz 	= 4'd4;
parameter op_memrd 	= 4'd5;
parameter op_output = 4'd6;
parameter op_adrld	= 4'd7;
parameter op_tstdp	= 4'd8;
parameter op_sbo	= 4'd9;

always @(*) begin
	instr_type 	<= instruction[10:8];
	instr_v1 	<= instruction[3:0];
	instr_v0 	<= instruction[7:4];
	jmp_offset 	<= instruction[7:0];
	instr_immediate <= instruction[7:0];
	//Flag Operation Decoding
	instr_flag_target <= instr_v1;
	case (instr_v0)
		4'd0: flag_src <= 0;
		4'd1: flag_src <= 1;
		4'd2: flag_src <= ZF;
		4'd3: flag_src <= IF;
		4'd4: flag_src <= ~ZF;
		4'd5: flag_src <= ~IF;
		4'd6: flag_src <= random_bit;
		default: flag_src <= 1'bx;
	endcase
		
	// Arithmetic Op decoding
	AU_sel 		<= instr_v0;
	AU_dest_reg <= instr_v1;
	case (AU_sel)
		op_sub, op_sbo 	: AU_result <= AU_sub_r;
		// 	: AU_result <= AU_amo_r;
		op_inv 	: AU_result <= AU_inv_r;
		op_mul 	: AU_result <= AU_mul_r;
		op_mov 	: AU_result <= AU_mov_r;
		op_memrd: AU_result <= AU_memrd_r;
		default:  AU_result <= 110'bx;
	endcase
end
//assign reg_wdata = AU_result;

//////////////////////////Main state Machine ///////////////////////////////////////	
always @(posedge clk or posedge rst) begin
	if (rst) begin
		ZF <= 1'b0; IF <= 1'b0;
		instr_addr <= 7'b0;
		state <= state_init;
		AU_sub_rst <= 1'b1; AU_mul_rst <= 1'b1; AU_inv_rst <= 1'b1; AU_mov_rst <= 1'b1;
		AU_memrd_rst <= 1'b1; AU_outputing_rst <= 1'b1;//AU_amo_rst <= 1'b1;
	end else begin
		case (state)
			state_init: begin // Processor Initialisation
				state <= state_decode;
			end
			state_decode: begin // Instruction Decode stage
				reg_we <= 1'b0;
				//instr_addr <= instr_addr + 1'b1;
				 case (instr_type)
					instr_ldo: begin //Load Operands
						op1_sel <= instr_v0;
						op2_sel <= instr_v1;
						//$display("ldo r%d, r%d", instr_v0, instr_v1);
						instr_addr <= instr_addr + 1'b1;
					end 
					instr_arith: begin  //Begin Arithmetic Ops
						/*case (AU_sel)
							op_sub: $display("sub");
							op_sbo: $display("amo");
							op_mul: $display("mul");
							op_inv: $display("inv");
							op_mov: $display("mov");
							op_memrd: $display("memrd");
							op_output: ;
							op_tstz: $display("tstz");
							op_tstdp: $display("tstdp");
							op_adrld: $display("ardld");
						endcase*/
							
						if (AU_sel == op_sbo)
						   AU_sub_am <= 1'b1;
						else
						   AU_sub_am <= 1'b0;  
						if ((AU_sel == op_tstdp)||(AU_sel == op_tstz)||(AU_sel == op_adrld)) begin
							op1_sel <= AU_dest_reg; //Tstz op use v1 as source register
							//$display("src reg: overrided to r%d", AU_dest_reg);
						   state <= state_arith_op_delay_2;
						end else
   						   state <= state_arith_op_delay;
					end
					instr_jz, instr_jnz: begin //Jump Instruction
						if(( (ZF == 1'b1)&&(instr_type ==  instr_jz) ) || 
								( (ZF == 1'b0)&&(instr_type == instr_jnz) )) begin
							instr_addr <= instr_addr + jmp_offset;
							//$display("Jmp taken, target:%d", instr_addr + jmp_offset);
						end	else begin
							instr_addr <= instr_addr + 1'b1;
							//$display("Jmp NOT taken");
					   end
					end
					instr_flag: begin //Flag Manipulation Instruction
						if (instr_flag_target[0] == 1'b0) begin
							ZF <= flag_src;
							//$display("ZF set to %d",flag_src);
						end else begin
							IF <= flag_src;
							//$display("IF set to %d",flag_src);
						end
						instr_addr <= instr_addr + 1'b1;
					end
					instr_adrge: begin
						//$display("Compare addr(%x) >= %x: result %d",rom_addr, instr_immediate, rom_addr>=instr_immediate);
						ZF <= (rom_addr >= instr_immediate);
						instr_addr <= instr_addr + 1'b1;
					end
					instr_adrinc: begin
						//$display("addr(%x) += %x",rom_addr, instr_immediate);
						rom_addr <= rom_addr + instr_immediate;
						instr_addr <= instr_addr + 1'b1;
					end
					default:;
				endcase
			end //end state_docode
			state_arith_op_delay_2: state <= state_arith_op_delay;
			state_arith_op_delay: begin
			   state <= state_arith_op;
				case (AU_sel)
					op_sub, op_sbo: AU_sub_rst <= 1'b0; 
					//: AU_amo_rst <= 1'b0; 
					op_mul: AU_mul_rst <= 1'b0;
					op_inv: AU_inv_rst <= 1'b0;
					op_mov: AU_mov_rst <= 1'b0;
					op_memrd: AU_memrd_rst <= 1'b0;
					op_output: AU_outputing_rst <= 1'b0;
				endcase
			end
			state_arith_op:begin
				if ((AU_sel == op_sub && AU_sub_done) ||
					(AU_sel == op_sbo && AU_sub_done) ||
					(AU_sel == op_mul && AU_mul_done) ||
					(AU_sel == op_inv && AU_inv_done) ||
					(AU_sel == op_mov && AU_mov_done) ||
					(AU_sel == op_memrd && AU_memrd_done) )begin //||	(AU_sel == op_output && AU_outputing_done)) begin
						reg_we <= 1'b1;
						reg_wdata <= AU_result;
						op1_sel <= AU_dest_reg;
						//$display("Arith done: Write back to r%d",AU_dest_reg);
					
/*
						case (AU_dest_reg) 
								0: reg_0 <= AU_result;1: reg_1 <= AU_result;2: reg_2 <= AU_result;
								3: reg_3 <= AU_result;4: reg_4 <= AU_result;5: reg_5 <= AU_result;
								default:$display("ERROR: Write to readonly Registers");
						endcase
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
					op_sub, op_sbo: if (AU_sub_done) begin
								AU_sub_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					/*: if (AU_amo_done) begin
								AU_amo_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end*/
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
					op_memrd: if (AU_memrd_done) begin
								AU_memrd_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_output: if (AU_outputing_done) begin
								AU_outputing_rst <= 1'b1;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
//$display("%.2x DEBUG:1. %x (%x)", instr_addr, op1, op1*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);								
//$display("%.2x DEBUG:2. %x (%x)", instr_addr, op2, op2*256'h0BF0F3C55F4AA2A18DB7B588F916 % `Modulus);								
							end
					op_tstz:if (AU_tstz_done) begin
								//$display("tstz done: %d",AU_tstz_r);
								ZF <= AU_tstz_r;
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_tstdp:if (AU_tstdp_done) begin
								ZF <= AU_tstdp_r;
								//$display("tstdp done: %d %x",AU_tstdp_r, op1);
					
								instr_addr <= instr_addr + 1'b1;
								state <= state_decode;
							end
					op_adrld: begin //adrld is not an arithmatic op, special case needed
							instr_addr <= instr_addr + 1'b1;
							state <= state_decode;
							rom_addr <= {1'b0, op1[4:0],2'b0};//RandomWalkBranchBits
							//$display("adrld done: %x",op1);
							end
				endcase
			end //end state_arith_op
		endcase	
	end
end

endmodule


