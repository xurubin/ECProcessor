module mul_16(input clk,
			  input reset,
			  output done,
			  input [15:0] a,
			  input [15:0] b,
			  output [31:0] r);
assign r = a*b;
assign done = 1'b1;
endmodule

module d_mul_32(input clk,
			  input reset,
			  output done,
			  input [31:0] a,
			  input [31:0] b,
			  output [63:0] r);
assign r = a*b;
assign done = 1'b1;
endmodule

module mul_32(input clk,
			  input reset,
			  output done,
			  input [31:0] a, //a_b
			  input [31:0] b, //c_d
			  output reg [63:0] r);
			
wire bd_done,ad_done,ac_done,bc_done;

wire [63:0] ac1;
wire [31:0] ac2,ac3;
wire [32:0] ac23;
assign ac23 = ac2 + ac3;

mul_16 mul_bd(.clk(clk), .reset(reset), .done(bd_done),
			 .a(a[15:0]), .b(b[15:0]), .r(ac1[31:0]));
mul_16 mul_ac(.clk(clk), .reset(reset), .done(ac_done),
			 .a(a[31:16]), .b(b[31:16]), .r(ac1[63:32]));

mul_16 mul_ad(.clk(clk), .reset(reset), .done(ad_done),
			 .a(a[15:0]), .b(b[31:16]), .r(ac2));
mul_16 mul_bc(.clk(clk), .reset(reset), .done(bc_done),
			 .b(b[15:0]), .a(a[31:16]), .r(ac3));
			
reg  state;
assign done = (state == 1);
always @(posedge clk or posedge reset) 
if (reset) begin
	state <= 0;
end else begin
	case (state)
	0: begin
		if (bd_done&ad_done&ac_done&bc_done)  begin
			r <=  {ac23+ac1[63:16], ac1[15:0]};
			state <= 1;
		end
	   end
	1:;
	endcase
end
endmodule


/*
module mult_110(
		input reset,
		input  [109:0] a,
		input  [109:0] b,
		input clk,
		output [219:0] r,
		output done);
		
reg [10:0]  state = 0;
reg [219:0] ac;
reg [219:0] op;
reg [109:0] factor;
parameter stop_state = 129;

assign done = (state == stop_state);
assign r = ac;
always @(posedge clk or posedge reset) 
if (reset) begin
			state <= 0;
end else begin
	if (state != stop_state) state <= state + 1;
	case (state)
	0:
		begin
			op <= b;
			ac <= 0;
			factor <= a;
			state <= 1;
		end
	stop_state: 
		state <= stop_state;
	default:
		begin
			if (factor[0] == 1) ac <= ac + op;
			factor <= factor >> 1;
			op <= op << 1;
			state <= state + 1;
		end
	endcase
end
endmodule		




module mul_64(input clk,
			  input reset,
			  output done,
			  input [63:0] a, //a_b
			  input [63:0] b, //c_d
			  output reg [127:0] r);
			
wire bd_done,ad_done,ac_done,bc_done;

wire [127:0] ac1;
wire [63:0] ac2,ac3;

mul_32 mul_bd(.clk(clk), .reset(reset), .done(bd_done),
			 .a(a[31:0]), .b(b[31:0]), .r(ac1[63:0]));
mul_32 mul_ac(.clk(clk), .reset(reset), .done(ac_done),
			 .a(a[63:32]), .b(b[63:32]), .r(ac1[127:64]));

mul_32 mul_ad(.clk(clk), .reset(reset), .done(ad_done),
			 .a(a[31:0]), .b(b[63:32]), .r(ac2));
mul_32 mul_bc(.clk(clk), .reset(reset), .done(bc_done),
			 .b(b[31:0]), .a(a[63:32]), .r(ac3));
			
reg  state;
assign done = (state == 1);
always @(posedge clk or posedge reset) 
if (reset) begin
	state <= 0;
end else begin
	case (state)
	0: begin
		if (bd_done&ad_done&ac_done&bc_done)  begin
			r <= ac1 + {ac2 + ac3,32'b0};//THIS WILL FAIL
			state <= 1;
		end
	   end
	1:;
	endcase
end
endmodule

module mul_128(input clk,
			  input reset,
			  output done,
			  input [127:0] a, //a_b
			  input [127:0] b, //c_d
			  output reg [255:0] r);
			
wire bd_done,ad_done,ac_done,bc_done;

wire [255:0] ac1;
wire [127:0] ac2,ac3;

mul_64 mul_bd(.clk(clk), .reset(reset), .done(bd_done),
			 .a(a[63:0]), .b(b[63:0]), .r(ac1[127:0]));
mul_64 mul_ac(.clk(clk), .reset(reset), .done(ac_done),
			 .a(a[127:64]), .b(b[127:64]), .r(ac1[255:128]));

mul_64 mul_ad(.clk(clk), .reset(reset), .done(ad_done),
			 .a(a[63:0]), .b(b[127:64]), .r(ac2));
mul_64 mul_bc(.clk(clk), .reset(reset), .done(bc_done),
			 .b(b[63:0]), .a(a[127:64]), .r(ac3));
			
reg  state;
assign done = (state == 1);
always @(posedge clk or posedge reset) 
if (reset) begin
	state <= 0;
end else begin
	case (state)
	0: begin
		if (bd_done&ad_done&ac_done&bc_done)  begin
			r <= ac1 + {ac2 + ac3,64'b0};//THIS WILL FAIL
			state <= 1;
		end
	   end
	1:;
	endcase
end
endmodule
*/