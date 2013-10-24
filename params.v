`include "defines.v"

module params(
	input clk,
	output vsync,
	output hsync,
	output [10:0] x,
	output [10:0] y,
	output can_draw,
	output start_of_frame
	); 

	assign x = h - `ha - `hb;
	assign y = v - `va - `vb;
	assign can_draw = (h >= (`ha + `hb)) && (h < (`ha + `hb + `hc))
				   && (v >= (`va + `vb)) && (v < (`va + `vb + `vc));
	assign vsync = vga_vsync;
	assign hsync = vga_hsync;
	assign start_of_frame = startframe;

	// horizontal and vertical counts
	reg [10:0] h;
	reg [10:0] v;
	reg vga_vsync;
	reg vga_hsync;
	reg startframe;
	
	always @(posedge clk) begin
	    // if we are not at the end of a row, increment h
		if (h < (`ha + `hb + `hc + `hd)) begin
			h <= h + 11'd1;
		// otherwise set h = 0 and increment v (unless we are at the bottom of the screen)
		end else begin
			h <= 11'd0;
			v <= (v < (`va + `vb + `vc + `vd)) ? v + 11'd1 : 11'd0;
		end
		vga_hsync <= h > `ha;
		vga_vsync <= v > `va;
		
		startframe <= (h == 11'd0) && (v == 11'd0);
	end
endmodule 