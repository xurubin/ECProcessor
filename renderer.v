
`include "defines.v"

module renderer(
	input clk,
	input candraw,
	input [10:0] x,
	input [10:0] y,
	input wire [287:0] data,
	output wire [5:0] line,
	output reg [9:0] red,
	output reg [9:0] green,
	output reg [9:0] blue,
	output vga_blank
	);

	reg n_vga_blank;
	assign vga_blank = !n_vga_blank;
	assign line = y[10:5];
	
	wire[3:0] font_x,font_y;
	wire pixel;
	reg[287:0] data_buf;
	
	reg[3:0] displayed_chars;
	 
	assign font_x = x[3:0];
	assign font_y = y[3:0];
	font_bitmap font(font_x, font_y, data_buf[287:284], pixel);

	always @(posedge clk) begin
		if (candraw) begin
		    n_vga_blank <= 1'b0;
			data_buf <= (x == 0) ? data 			//At line head, reload data
					  :( font_x == 4'b1111 & (displayed_chars != 8) ) ? data_buf << 4//Load next byte
					  : data_buf;

			displayed_chars <= (x == 0) ? 0  //Reset Counter
							  :(font_x == 4'b1111) ?  // About to do the next char
								   ( (displayed_chars == 8) ? 0: displayed_chars + 1)
							  :displayed_chars;
				
			//  Text
			if (y[4] == 1 && (displayed_chars != 8) ) begin //Odd line and display byte
				red 	<= (pixel == 1)? 10'd0 : 10'h3FF;
				green 	<= (pixel == 1)? 10'd0 : 10'h3FF;
				blue	<= (pixel == 1)? 10'd0 : 10'h3FF;
			// with a blue background
			end else begin
				red		<= 10'h3FF;
				green	<= 10'h3FF;
				blue	<= 10'h3FF;
			end
		end else begin
			// if we are not in the visible area, we must set the screen blank
			n_vga_blank <= 1'b1;
		end
	end
endmodule 