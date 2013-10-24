`include "defines.v"

module DE2_FPGA(
	input  CLOCK_27,
	input  CLOCK_50,
	input  [3:0] KEY,
	input  [17:0] SW,
	output [17:0] LEDR,
	output [7:0]  LEDG,
	output [9:0] VGA_R,
	output [9:0] VGA_G,
	output [9:0] VGA_B,
	output VGA_CLK,
	output VGA_BLANK,
	output VGA_HS,
	output VGA_VS,
	output VGA_SYNC,
	output TD_RESET,
	output [6:0] HEX0,
	output [6:0] HEX1,
	output [6:0] HEX2,
	output [6:0] HEX3,
	output [6:0] HEX4,
	output [6:0] HEX5,
	output [6:0] HEX6,
	output [6:0] HEX7
	);
	wire CLOCK_80;
	PLL80MHz pll2(.inclk0(CLOCK_50), .c0(CLOCK_80));
/*
wire [7:0] rom_addr, rom_data;
test_rom rom(.addr(rom_addr), .code(rom_data));
processor proc(.clk(CLOCK_80), .rst(~KEY[0]), .instr_addr(rom_addr), .instruction(rom_data), 
               .data_0(110'd1 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus),
               .data_1(110'd2 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus),
               .data_2(110'd3 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus),
               .data_3(110'd4 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus),
				.result_0(ecp_r_x), .result_1(ecp_r_y)
               );
*/        

wire [3:0] zoo_state;
wire [8:0] zoo_count;
/*
Zoo zoo(.clk(CLOCK_80),.reset(~KEY[0]), .resume(~KEY[1]),
        .masterSeed(110'h2A995E7EFE396E61B2D77E92FF2C),
        .GX(`EC_GX),
        .GY(`EC_GY),
        .PX(`EC_KX),
        .PY(`EC_KY),
		.T1({ecp_r_x,ecp_r_y}),
		.internal_state(zoo_state),
		.count(zoo_count));
*/
	
	reg [109:0] mont_r;
	wire[109:0] mont_r_wire;
	wire mont_done;
/*	
	always @(posedge CLOCK_50)
	mont_r <= mont_r_wire;
    mont_mul_p mont_mul( .clk(CLOCK_80), .reset(~KEY[0]), .done(mont_done),
						 .a(110'h2A995E7EFE396E61B2D77E92FF2C), 
						 .b(110'h2A995E7EFE396E61B2D77E92FF2C),
						 .r(mont_r_wire) );//Shoud be 14 BB 69 D2 E2 E5 79 41 FF E3 B7 45 BA D6 
	
	wire invert_done;
	wire[110:0] invert_r;
	mont_invert_p invert(.clk(CLOCK_80), .reset(~KEY[0]), .done(invert_done),
						 .a(SW[15:0]), .result(invert_r) ); 
*/	
					
	wire ecp_done;
	wire[109:0] ecp_r_x;
	wire[109:0] ecp_r_y;
	wire ecp_r_infinity;

	wire[`R_Bits-1:0] ecp_px,ecp_py,ecp_py1, ecp_qx,ecp_qy, ecp_qy1;
/*
	ecp_selector ecps1(.index(SW[3:2]), .negate(KEY[1]), .x(ecp_px), .y(ecp_py));
	ecp_selector ecps2(.index(SW[1:0]), .negate(KEY[2]), .x(ecp_qx), .y(ecp_qy));
/*
	ecurve_add ecp(.clk(video_clock), .reset(~KEY[0]), .done(ecp_done),
			   	.p_x(ecp_px),
				.p_y(ecp_py),
				.p_infinity(1'b0),
			   	.q_x(ecp_qx),
				.q_y(ecp_qy),
				.q_infinity(1'b0),
			   .r_x(ecp_r_x),.r_y(ecp_r_y),.r_infinity(ecp_r_infinity)
			);
*/
wire [9:0] zoo_addr;
Zoo_adv zoo(.clk(CLOCK_80), .rst(~KEY[0]), .addr(zoo_addr), .debug(LEDG[7:0]));
hex2leds led0(.hexval(zoo_addr[3:0]) , .ledcode(HEX0));
hex2leds led1(.hexval(zoo_addr[7:4]) , .ledcode(HEX1));
hex2leds led2(.hexval(zoo_addr[9:8]) , .ledcode(HEX2));

/*
	reg [109:0] a,b,add_a,add_b,sub_a,sub_b;
	wire [109:0]t1,t2 ;
	reg t;
	assign ecp_r_x = a;
	assign ecp_r_y = b;
	always @(posedge CLOCK_50) begin
	t <= ~t;
	if (t) begin
		add_a <= ecp_px;
		add_b <= ecp_py;
		//sub_a <= ecp_qx;
		//sub_b <= ecp_qy;
	end else begin
		a <= t1;
		b <= t2;
	end
	end
	add_p_p adder(.clk(CLOCK_50), .a(add_a), .b(add_b), .r(t1));
	subtract_p_p sub(.clk(CLOCK_50),.a(sub_a), .b(sub_b), .r(t2));
*//*
	hex2leds led0(.hexval(ecp_r_x[3:0]) , .ledcode(HEX0));
	hex2leds led1(.hexval(ecp_r_x[7:4]) , .ledcode(HEX1));
	hex2leds led2(.hexval(ecp_r_x[11:8]) , .ledcode(HEX2));
	hex2leds led3(.hexval(ecp_r_x[15:12]), .ledcode(HEX3));
	hex2leds led4(.hexval(zoo_count[3:0]) , .ledcode(HEX4));
	hex2leds led5(.hexval(zoo_count[7:4]) , .ledcode(HEX5));
	hex2leds led6(.hexval(zoo_count[8]) , .ledcode(HEX6));
	hex2leds led7(.hexval(0) 			, .ledcode(HEX7));
	*/
	//assign LEDG[3:0] = zoo_state;
	assign LEDR[17:0] = ecp_r_y;
	////////////////////VGA Stuff follows////////////////////	
	/*				
	wire video_clock;

	PLL108MHz pll(.inclk0(CLOCK_27), .c0(video_clock));
	
	assign VGA_CLK = video_clock;
	assign VGA_SYNC = 0;
	
	wire candraw; 
	wire start;
	wire ballon;
	wire [10:0] x;
	wire [10:0] y;
	
	assign TD_RESET = 1'b1;
 
	// VGA parameters
	params p(
		.clk(video_clock),
		.vsync(VGA_VS),
		.hsync(VGA_HS),
		.x(x),
		.y(y),
		.can_draw(candraw),
		.start_of_frame(start)
		);

	// Module that does the drawing on screen
	wire [287:0] dispaly_bytes;
	wire [5:0] display_lineNo;
	
	assign dispaly_bytes = 
	 (display_lineNo == 0) ? 288'h76543210FEDCBA9876543210FEDCBA9876543210FEDCBA9876543210FEDCBA9876543210
	:(display_lineNo == 2) ? SW[17:0]
	//:(display_lineNo == 3) ? { mul_r}
	//:(display_lineNo == 4) ? { mul_done}
	
	:(display_lineNo == 5) ? { mont_r_wire}
	:(display_lineNo == 6) ? { mont_done}
	//:(display_lineNo == 7) ? { invert_done}
	//:(display_lineNo == 8) ? { invert_r}
	:(display_lineNo == 9 ) ? { ecp_px}
	:(display_lineNo == 10) ? { ecp_py}
	:(display_lineNo == 11) ? { ecp_qx}
	:(display_lineNo == 12) ? { ecp_qy}
	:(display_lineNo == 14) ? { ecp_r_x}
	:(display_lineNo == 15) ? { ecp_r_y}
	:(display_lineNo == 16) ? { ecp_r_infinity}
	:x;
	
	renderer r(
		.clk(video_clock),
		.candraw(candraw),
		.x(x),
		.y(y),
		.line(display_lineNo),
		.data(dispaly_bytes),
		.red(VGA_R),
		.green(VGA_G),
		.blue(VGA_B),
		.vga_blank(VGA_BLANK)
		);
*/
endmodule

module ecp_selector(input [1:0] index,
					input negate,
					output reg [`R_Bits-1:0] x,
					output reg [`R_Bits-1:0] y
					);
reg[`R_Bits-1:0] y0;
always @(*) begin
	case (index) 
	0: begin
		x <= 	110'h0E4C68977A69F7FDC9F25A5FA283 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
		y0 <=	110'h26C30B539BB6DAA4B33A46889DD7 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
	end
	1: begin
		x <=	110'h274DE10F63E72CB2E7F04EC91542 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
		y0 <= 	110'h28A1A0868FCAB0087ADD2BE22036 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
	end
	2: begin
		x <=	110'h1848AE71AF935A7746607CC9D204 *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
		y0 <= 	110'h12C6ECF9E0764C9F836C6AA05E7A *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
	end
	3: begin
		x <=	110'h0B2BD256BA0A5EDA03DD049A822D *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
		y0 <= 	110'h1CDF30664B15E37732244BBE798F *256'h10BFDEE413D91FEC458A7A0D16 % `Modulus;
	end
	endcase
   y <= (negate == 0) ? y0 : (`Modulus - y0);
end
endmodule
