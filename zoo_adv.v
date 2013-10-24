`include "defines.v"

module randomwalk_program(input [7:0] addr, output reg [10:0] code );
always @(*) begin
	case (addr)
	8'd0:	code <= 11'b00011111111; //		ldo r15 , r15
	8'd1:	code <= 11'b00100001111; //		sub r15		//initialise zero in r15 
	8'd2:	code <= 11'b00101111111; //		adrld r15	
	8'd3:	code <= 11'b00101011110; //		memrd r14	//load (-a) from 0x0 to r14
	8'd4:	code <= 11'b10000010011; //@Restart	setflg if, one	//initialise if(infinity_flag)
	8'd5:	code <= 11'b00011111111; //		ldo r15,r15
	8'd6:	code <= 11'b00100110000; //		mov r0		//initilise store [g,k,PX,PY] = [r0,r1,r2,r3]
	8'd7:	code <= 11'b00011111111; //		ldo r15,r15
	8'd8:	code <= 11'b00100110001; //		mov r1
	8'd9:	code <= 11'b00011111111; //		ldo r15,r15
	8'd10:	code <= 11'b00100110010; //		mov r2
	8'd11:	code <= 11'b00011111111; //		ldo r15,r15
	8'd12:	code <= 11'b00100110011; //		mov r3
	8'd13:	code <= 11'b00101011010; //@LoadPts	memrd r10	//load [g,k,PX,PY] to [r10,r11,r12,r13]
	8'd14:	code <= 11'b11000000001; //		adrinc 1
	8'd15:	code <= 11'b00101011011; //		memrd r11
	8'd16:	code <= 11'b11000000001; //		adrinc 1
	8'd17:	code <= 11'b00101011100; //		memrd r12
	8'd18:	code <= 11'b11000000001; //		adrinc 1
	8'd19:	code <= 11'b00101011101; //		memrd r13
	8'd20:	code <= 11'b11000000001; //		adrinc 1
	8'd21:	code <= 11'b00011111010; //		ldo r15, r10	//EC Point Addition [r0,r1,r2,r3,if] += [r10,r11,r12,r13]
	8'd22:	code <= 11'b00110011010; //		sbo r10		//[r0,r1]+=[r10,r11]
	8'd23:	code <= 11'b00000001010; //		ldo r0, r10
	8'd24:	code <= 11'b00110010000; //		sbo r0
	8'd25:	code <= 11'b00011111011; //		ldo r15, r11
	8'd26:	code <= 11'b00110011011; //		sbo r11
	8'd27:	code <= 11'b00000011011; //		ldo r1, r11
	8'd28:	code <= 11'b00110010001; //		sbo r1
	8'd29:	code <= 11'b10000110010; //		setflg zf,if
	8'd30:	code <= 11'b01100000111; //		jnz p_ni		//jmp if if=0
	8'd31:	code <= 11'b00011001100; //		ldo r12, r12	//[r0..] is inifinity, easy job
	8'd32:	code <= 11'b00100110010; //		mov r2
	8'd33:	code <= 11'b00011011101; //		ldo r13,r13
	8'd34:	code <= 11'b00100110011; //		mov r3
	8'd35:	code <= 11'b10000000011; //		setflg if, zero
	8'd36:	code <= 11'b01000110101; //		jz ADD_DONE
	8'd37:	code <= 11'b00000101100; //@p_ni		ldo r2, r12	//(p_x, q_x) Normal Add, still need to check if result is inf or double
	8'd38:	code <= 11'b00100000100; //		sub r4 		//(r4 = p_x - q_x)
	8'd39:	code <= 11'b00101000100; //		tstz r4		//p_x-q_x
	8'd40:	code <= 11'b01000001000; //		jz DOUBLE_PRE
	8'd41:	code <= 11'b00001000100; //		ldo r4,r4	
	8'd42:	code <= 11'b00100100100; //		inv r4 		//r4 = (p_x-q_x)^(-1)
	8'd43:	code <= 11'b00000111101; //		ldo r3, r13 	//(p_y, q_y)
	8'd44:	code <= 11'b00100000101; //		sub r5 		//r5 = p_y - q_y
	8'd45:	code <= 11'b00001000101; //		ldo r4, r5 	//[(p_x-q_x)^(-1), p_y - q_y]
	8'd46:	code <= 11'b00100010100; //		mul r4 		//r4 = s = (p_y - q_y)/(p_x-q_x)
	8'd47:	code <= 11'b01100011100; //		jnz COMMON
	8'd48:	code <= 11'b00000111101; //@DOUBLE_PRE	ldo r3, r13 	//(p_y, q_y)
	8'd49:	code <= 11'b00100000100; //		sub r4		//r4 = p_y - q_y
	8'd50:	code <= 11'b00101000100; //		tstz r4		//p_y - q_y
	8'd51:	code <= 11'b01000000111; //		jz DOUBLE
	8'd52:	code <= 11'b10000010011; //@R_INF		setflg if, one	//result is infinity
	8'd53:	code <= 11'b00011111111; //		ldo r15, r15
	8'd54:	code <= 11'b00100110010; //		mov r2
	8'd55:	code <= 11'b00011111111; //		ldo r15, r15
	8'd56:	code <= 11'b00100110011; //		mov r3
	8'd57:	code <= 11'b01100100000; //		jnz ADD_DONE
	8'd58:	code <= 11'b00011111101; //@DOUBLE		ldo r15, r13	//(0, q_y)
	8'd59:	code <= 11'b00100000100; //		sub r4		//r4 = -q_y
	8'd60:	code <= 11'b00011010100; //		ldo r13, r4	//(q_y, -q_y)
	8'd61:	code <= 11'b00100000011; //		sub r3		//r3 = q_y - (-q_y) = 2y
	8'd62:	code <= 11'b00100100011; //		inv r3		//r3 = (2y)^(-1)
	8'd63:	code <= 11'b00011001100; //		ldo r12, r12	//(q_x, q_x)
	8'd64:	code <= 11'b00100010101; //		mul r5		//r5 =q_x^2
	8'd65:	code <= 11'b00011110101; //		ldo r15, r5	//(0, q_x^2)
	8'd66:	code <= 11'b00100000100; //		sub r4		//r4 = -x^2
	8'd67:	code <= 11'b00001010100; //		ldo r5, r4	//(x^2, -x^2)
	8'd68:	code <= 11'b00100000101; //		sub r5		//r5 = 2*x^2
	8'd69:	code <= 11'b00001010100; //		ldo r5, r4	//(2*x^2, -x^2)
	8'd70:	code <= 11'b00100000101; //		sub r5		//r5 = 3*x^2
	8'd71:	code <= 11'b00001011110; //		ldo r5, r14	//(3x^2, -A)
	8'd72:	code <= 11'b00100000100; //		sub r4		//r4 = 3x^2+A
	8'd73:	code <= 11'b00000110100; //		ldo r3, r4	//(2y^-1, 3x^2+A)
	8'd74:	code <= 11'b00100010100; //		mul r4		//r4 = s = (3x^2+A)/2y;
	8'd75:	code <= 11'b00001000100; //@COMMON		ldo r4, r4	//(s, s)
	8'd76:	code <= 11'b00100010101; //		mul r5		//r5 = s*s
	8'd77:	code <= 11'b00001010010; //		ldo r5, r2	//s^2, p_x
	8'd78:	code <= 11'b00100000101; //		sub r5		//r5 = s^2-px
	8'd79:	code <= 11'b00001011100; //		ldo r5, r12	//s^2-px, qx
	8'd80:	code <= 11'b00100000010; //		sub r2		//r2 = s^2-px-qx 		<== X
	8'd81:	code <= 11'b00000101100; //		ldo r2, r12	//X, qx
	8'd82:	code <= 11'b00100000011; //		sub r3		//r3 = X-qx
	8'd83:	code <= 11'b00000110100; //		ldo r3, r4	//X-qx, s
	8'd84:	code <= 11'b00100010011; //		mul r3		//r3 = s*(X-qx)
	8'd85:	code <= 11'b00011111101; //		ldo r15, r13	//0, qy 
	8'd86:	code <= 11'b00100000100; //		sub r4		//r4 = -qy
	8'd87:	code <= 11'b00001000011; //		ldo r4, r3	//-qy, s*(X-qx)
	8'd88:	code <= 11'b00100000011; //		sub r3		//r3 = -[py+s*(X-px)]	<== Y
	8'd89:	code <= 11'b00110000010; //@ADD_DONE	tstdp r2	//Test DP Property, ZF=1 if DP
	8'd90:	code <= 11'b01100000101; //		jnz NOT_DP
	8'd91:	code <= 11'b00000000001; //		ldo r0,r1	//It's a DP, Output DP Coef pairs
	8'd92:	code <= 11'b00101100000; //		output r0
	8'd93:	code <= 11'b00101111111; //		adrld r15
	8'd94:	code <= 11'b01010100110; //		jz Restart
	8'd95:	code <= 11'b10101111111; //@NOT_DP		adrge 127
	8'd96:	code <= 11'b01100000100; //		jnz INITING
	8'd97:	code <= 11'b00101110010; //		adrld r2	//Continue Random Walking
	8'd98:	code <= 11'b11010000000; //		adrinc 128
	8'd99:	code <= 11'b01010101010; //		jz LoadPts
	8'd100:	code <= 11'b11000000100; //@INITING	adrinc 4	//Initialising RW in process
	8'd101:	code <= 11'b10001100010; //		setflg zf, rnd
	8'd102:	code <= 11'b01111111001; //		jnz NOT_DP
	8'd103:	code <= 11'b01010100110; //		jz LoadPts
	default: code <= 11'bx;
	endcase
end	
endmodule



module Zoo_adv(input clk, input rst, output[7:0] debug,  output[9:0] addr);
////////RandomWalk Configuration Database//////////////////////////
reg [7:0] RW_RomAddr;
wire [`R_Bits-1:0] RW_RomData;
Randomwalk_ROM rom(.clk(clk), .addr(RW_RomAddr), .data(RW_RomData));

////////Processor Instance 1///////////////////////////////////////
wire p1_outputing;
reg p1_output_ack;
wire [2*`R_Bits-1:0] p1_output;

wire p1_rom_rqst, p1_random_bit;
wire [7:0] p1_rom_addr;
reg p1_rom_rdy;
wire [7:0] p1_instr_addr;
wire [10:0] p1_instr_code;
wire [`R_Bits-1:0] p1_r0,p1_r1;
randomwalk_program rwp1(.addr(p1_instr_addr), .code(p1_instr_code));
processor_advanced p1(.clk(clk), .rst(rst), 
				.instr_addr(p1_instr_addr), .instruction(p1_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p1_rom_addr),
				.rom_rqst(p1_rom_rqst), .rom_rdy(p1_rom_rdy), 
				.random_bit(p1_random_bit),
				.result_0(p1_r0), .result_1(p1_r1), 
				.outputing(p1_outputing), .output_ack(p1_output_ack)
				);
assign p1_output = {p1_r0, p1_r1};
////////Processor Instance 2///////////////////////////////////////
wire p2_outputing;
reg p2_output_ack;
wire [2*`R_Bits-1:0] p2_output;

wire p2_rom_rqst, p2_random_bit;
wire [7:0] p2_rom_addr;
reg p2_rom_rdy;
wire [7:0] p2_instr_addr;
wire [10:0] p2_instr_code;
wire [`R_Bits-1:0] p2_r0,p2_r1;
randomwalk_program rwp2(.addr(p2_instr_addr), .code(p2_instr_code));
processor_advanced p2(.clk(clk), .rst(rst), 
				.instr_addr(p2_instr_addr), .instruction(p2_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p2_rom_addr),
				.rom_rqst(p2_rom_rqst), .rom_rdy(p2_rom_rdy), 
				.random_bit(p2_random_bit),
				.result_0(p2_r0), .result_1(p2_r1), 
				.outputing(p2_outputing), .output_ack(p2_output_ack)
				);
assign p2_output = {p2_r0, p2_r1};
////////Processor Instance 3///////////////////////////////////////
wire p3_outputing;
reg p3_output_ack;
wire [2*`R_Bits-1:0] p3_output;

wire p3_rom_rqst, p3_random_bit;
wire [7:0] p3_rom_addr;
reg p3_rom_rdy;
wire [7:0] p3_instr_addr;
wire [10:0] p3_instr_code;
wire [`R_Bits-1:0] p3_r0,p3_r1;
randomwalk_program rwp3(.addr(p3_instr_addr), .code(p3_instr_code));
processor_advanced p3(.clk(clk), .rst(rst), 
				.instr_addr(p3_instr_addr), .instruction(p3_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p3_rom_addr),
				.rom_rqst(p3_rom_rqst), .rom_rdy(p3_rom_rdy), 
				.random_bit(p3_random_bit),
				.result_0(p3_r0), .result_1(p3_r1), 
				.outputing(p3_outputing), .output_ack(p3_output_ack)
				);
assign p3_output = {p3_r0, p3_r1};
////////Processor Instance 4///////////////////////////////////////
wire p4_outputing;
reg p4_output_ack;
wire [2*`R_Bits-1:0] p4_output;

wire p4_rom_rqst, p4_random_bit;
wire [7:0] p4_rom_addr;
reg p4_rom_rdy;
wire [7:0] p4_instr_addr;
wire [10:0] p4_instr_code;
wire [`R_Bits-1:0] p4_r0,p4_r1;
randomwalk_program rwp4(.addr(p4_instr_addr), .code(p4_instr_code));
processor_advanced p4(.clk(clk), .rst(rst), 
				.instr_addr(p4_instr_addr), .instruction(p4_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p4_rom_addr),
				.rom_rqst(p4_rom_rqst), .rom_rdy(p4_rom_rdy), 
				.random_bit(p4_random_bit),
				.result_0(p4_r0), .result_1(p4_r1), 
				.outputing(p4_outputing), .output_ack(p4_output_ack)
				);
assign p4_output = {p4_r0, p4_r1};
////////Processor Instance 5///////////////////////////////////////
wire p5_outputing;
reg p5_output_ack;
wire [2*`R_Bits-1:0] p5_output;

wire p5_rom_rqst, p5_random_bit;
wire [7:0] p5_rom_addr;
reg p5_rom_rdy;
wire [7:0] p5_instr_addr;
wire [10:0] p5_instr_code;
wire [`R_Bits-1:0] p5_r0,p5_r1;
randomwalk_program rwp5(.addr(p5_instr_addr), .code(p5_instr_code));
processor_advanced p5(.clk(clk), .rst(rst), 
				.instr_addr(p5_instr_addr), .instruction(p5_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p5_rom_addr),
				.rom_rqst(p5_rom_rqst), .rom_rdy(p5_rom_rdy), 
				.random_bit(p5_random_bit),
				.result_0(p5_r0), .result_1(p5_r1), 
				.outputing(p5_outputing), .output_ack(p5_output_ack)
				);
assign p5_output = {p5_r0, p5_r1};
////////Processor Instance 6///////////////////////////////////////
wire p6_outputing;
reg p6_output_ack;
wire [2*`R_Bits-1:0] p6_output;

wire p6_rom_rqst, p6_random_bit;
wire [7:0] p6_rom_addr;
reg p6_rom_rdy;
wire [7:0] p6_instr_addr;
wire [10:0] p6_instr_code;
wire [`R_Bits-1:0] p6_r0,p6_r1;
randomwalk_program rwp6(.addr(p6_instr_addr), .code(p6_instr_code));
processor_advanced p6(.clk(clk), .rst(rst), 
				.instr_addr(p6_instr_addr), .instruction(p6_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p6_rom_addr),
				.rom_rqst(p6_rom_rqst), .rom_rdy(p6_rom_rdy), 
				.random_bit(p6_random_bit),
				.result_0(p6_r0), .result_1(p6_r1), 
				.outputing(p6_outputing), .output_ack(p6_output_ack)
				);
assign p6_output = {p6_r0, p6_r1};
////////Processor Instance 7///////////////////////////////////////
wire p7_outputing;
reg p7_output_ack;
wire [2*`R_Bits-1:0] p7_output;

wire p7_rom_rqst, p7_random_bit;
wire [7:0] p7_rom_addr;
reg p7_rom_rdy;
wire [7:0] p7_instr_addr;
wire [10:0] p7_instr_code;
wire [`R_Bits-1:0] p7_r0,p7_r1;
randomwalk_program rwp7(.addr(p7_instr_addr), .code(p7_instr_code));
processor_advanced p7(.clk(clk), .rst(rst), 
				.instr_addr(p7_instr_addr), .instruction(p7_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p7_rom_addr),
				.rom_rqst(p7_rom_rqst), .rom_rdy(p7_rom_rdy), 
				.random_bit(p7_random_bit),
				.result_0(p7_r0), .result_1(p7_r1), 
				.outputing(p7_outputing), .output_ack(p7_output_ack)
				);
assign p7_output = {p7_r0, p7_r1};
////////Processor Instance 8///////////////////////////////////////
wire p8_outputing;
reg p8_output_ack;
wire [2*`R_Bits-1:0] p8_output;

wire p8_rom_rqst, p8_random_bit;
wire [7:0] p8_rom_addr;
reg p8_rom_rdy;
wire [7:0] p8_instr_addr;
wire [10:0] p8_instr_code;
wire [`R_Bits-1:0] p8_r0,p8_r1;
randomwalk_program rwp8(.addr(p8_instr_addr), .code(p8_instr_code));
processor_advanced p8(.clk(clk), .rst(rst), 
				.instr_addr(p8_instr_addr), .instruction(p8_instr_code), 
				
				.rom_data(RW_RomData), .rom_addr(p8_rom_addr),
				.rom_rqst(p8_rom_rqst), .rom_rdy(p8_rom_rdy), 
				.random_bit(p8_random_bit),
				.result_0(p8_r0), .result_1(p8_r1), 
				.outputing(p8_outputing), .output_ack(p8_output_ack)
				);
assign p8_output = {p8_r0, p8_r1};
////////DP Point Storage RAM///////////////////////////////////////
reg[2*`R_Bits-1:0] mem_data;
reg[8:0] mem_addr;
reg mem_wren;
ram mem(.address(mem_addr),.clock(clk),.data(mem_data),.wren(mem_wren));
///////RW_Database Arbitator///////////////////////////////////////
reg [3:0] rom_arbitator_state;
parameter rom_arbitator_idle = 0;
parameter rom_arbitator_serv_p1 = 1;
parameter rom_arbitator_serv_p2 = 2;
parameter rom_arbitator_serv_p3 = 3;
parameter rom_arbitator_serv_p4 = 4;
parameter rom_arbitator_serv_p5 = 5;
parameter rom_arbitator_serv_p6 = 6;
parameter rom_arbitator_serv_p7 = 7;
parameter rom_arbitator_serv_p8 = 8;
always @(posedge clk or posedge rst)
if (rst) begin
	rom_arbitator_state <= rom_arbitator_idle;
	p1_rom_rdy <= 1'b0;
	p2_rom_rdy <= 1'b0;
	p3_rom_rdy <= 1'b0;
	p4_rom_rdy <= 1'b0;
	p5_rom_rdy <= 1'b0;
	p6_rom_rdy <= 1'b0;
	p7_rom_rdy <= 1'b0;
	p8_rom_rdy <= 1'b0;
end else begin
	case (rom_arbitator_state)
	  rom_arbitator_idle: begin
		if (p1_rom_rqst) begin
			RW_RomAddr <= p1_rom_addr;
			p1_rom_rdy <= !p1_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p1;
		end else
		if (p2_rom_rqst) begin
			RW_RomAddr <= p2_rom_addr;
			p2_rom_rdy <= !p2_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p2;
		end else
		if (p3_rom_rqst) begin
			RW_RomAddr <= p3_rom_addr;
			p3_rom_rdy <= !p3_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p3;
		end else
		if (p4_rom_rqst) begin
			RW_RomAddr <= p4_rom_addr;
			p4_rom_rdy <= !p4_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p4;
		end else
		if (p5_rom_rqst) begin
			RW_RomAddr <= p5_rom_addr;
			p5_rom_rdy <= !p5_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p5;
		end else
		if (p6_rom_rqst) begin
			RW_RomAddr <= p6_rom_addr;
			p6_rom_rdy <= !p6_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p6;
		end else
		if (p7_rom_rqst) begin
			RW_RomAddr <= p7_rom_addr;
			p7_rom_rdy <= !p7_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p7;
		end else
		if (p8_rom_rqst) begin
			RW_RomAddr <= p8_rom_addr;
			p8_rom_rdy <= !p8_rom_rdy;
			rom_arbitator_state <= rom_arbitator_serv_p8;
		end
	  end
	  rom_arbitator_serv_p1: if (p1_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p2: if (p2_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p3: if (p3_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p4: if (p4_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p5: if (p5_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p6: if (p6_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p7: if (p7_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	  rom_arbitator_serv_p8: if (p8_rom_rqst == 1'b0)
			rom_arbitator_state <= rom_arbitator_idle;
	endcase
end

//////DP Point Storage RAM Arbitator///////////////////////////////
reg [3:0] dpram_arbitator_state;
parameter dpram_arbitator_idle = 0;
parameter dpram_arbitator_serv_p1 = 1;
parameter dpram_arbitator_serv_p2 = 2;
parameter dpram_arbitator_serv_p3 = 3;
parameter dpram_arbitator_serv_p4 = 4;
parameter dpram_arbitator_serv_p5 = 5;
parameter dpram_arbitator_serv_p6 = 6;
parameter dpram_arbitator_serv_p7 = 7;
parameter dpram_arbitator_serv_p8 = 8;
always @(posedge clk or posedge rst)
if (rst) begin
	dpram_arbitator_state <= dpram_arbitator_idle;
	p1_output_ack <= 1'b0;
	p2_output_ack <= 1'b0;
	p3_output_ack <= 1'b0;
	p4_output_ack <= 1'b0;
	p5_output_ack <= 1'b0;
	p6_output_ack <= 1'b0;
	p7_output_ack <= 1'b0;
	p8_output_ack <= 1'b0;
	mem_addr <= -1;
end else begin
	case (dpram_arbitator_state)
	  dpram_arbitator_idle: begin
		if (p1_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p1_output;
			mem_wren <= 1'b1;
			p1_output_ack <= !p1_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p1;
			$display("P1 - Write to Mem:%x", p1_output);
		end else
		if (p2_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p2_output;
			mem_wren <= 1'b1;
			p2_output_ack <= !p2_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p2;
			$display("P2 - Write to Mem:%x", p2_output);
		end else
		if (p3_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p3_output;
			mem_wren <= 1'b1;
			p3_output_ack <= !p3_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p3;
			$display("P3 - Write to Mem:%x", p3_output);
		end else
		if (p4_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p4_output;
			mem_wren <= 1'b1;
			p4_output_ack <= !p4_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p4;
			$display("P4 - Write to Mem:%x", p4_output);
		end else
		if (p5_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p5_output;
			mem_wren <= 1'b1;
			p5_output_ack <= !p5_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p5;
			$display("P5 - Write to Mem:%x", p5_output);
		end else
		if (p6_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p6_output;
			mem_wren <= 1'b1;
			p6_output_ack <= !p6_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p6;
			$display("P6 - Write to Mem:%x", p6_output);
		end else
		if (p7_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p7_output;
			mem_wren <= 1'b1;
			p7_output_ack <= !p7_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p7;
			$display("P7 - Write to Mem:%x", p7_output);
		end else
		if (p8_outputing) begin
			mem_addr <= mem_addr+1'b1;
			mem_data <= p8_output;
			mem_wren <= 1'b1;
			p8_output_ack <= !p8_output_ack;
			dpram_arbitator_state <= dpram_arbitator_serv_p8;
			$display("P8 - Write to Mem:%x", p8_output);
		end
	  end
	  dpram_arbitator_serv_p1: begin
			mem_wren <= 1'b0;
			if (p1_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p2: begin
			mem_wren <= 1'b0;
			if (p2_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p3: begin
			mem_wren <= 1'b0;
			if (p3_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p4: begin
			mem_wren <= 1'b0;
			if (p4_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p5: begin
			mem_wren <= 1'b0;
			if (p5_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p6: begin
			mem_wren <= 1'b0;
			if (p6_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p7: begin
			mem_wren <= 1'b0;
			if (p7_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	  dpram_arbitator_serv_p8: begin
			mem_wren <= 1'b0;
			if (p8_outputing == 1'b0) dpram_arbitator_state <= dpram_arbitator_idle;
	  end
	endcase
end

/////RandomNumber Generator & Arbitator///////////////////////////
reg[109:0] rnd_reg;
reg[7:0] rnd_pool, rnd_t;
reg [2:0] reg_i;
always @(posedge clk or posedge rst)
if (rst) begin
	rnd_reg <= `SeedDuplicationFactor;
	reg_i <= 3'd0;
end else begin
	rnd_reg <= ((rnd_reg<<1)|(rnd_reg[109]^rnd_reg[108]^rnd_reg[97]^rnd_reg[96]));
	rnd_t <= {rnd_t[6:0], rnd_reg[109]};
	reg_i <= reg_i + 1'd1;
	if (reg_i == 3'b000)
		rnd_pool <= rnd_t;
end
assign p1_random_bit = rnd_pool[0];
assign p2_random_bit = rnd_pool[1];
assign p3_random_bit = rnd_pool[2];
assign p4_random_bit = rnd_pool[3];
assign p5_random_bit = rnd_pool[4];
assign p6_random_bit = rnd_pool[5];
assign p7_random_bit = rnd_pool[6];
assign p8_random_bit = rnd_pool[7];

assign debug = {p1_output_ack, p2_output_ack, p3_output_ack, p4_output_ack,
				p4_output_ack, p5_output_ack, p6_output_ack, p7_output_ack};
assign addr = mem_addr;
endmodule
